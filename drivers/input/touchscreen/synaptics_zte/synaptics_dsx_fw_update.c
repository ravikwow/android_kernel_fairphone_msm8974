/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include <linux/input/synaptics_dsx.h>
#include "synaptics_dsx_i2c.h"
#include <linux/proc_fs.h>
#include <linux/wakelock.h>

/** ZTE_MODIFY chenfei added for LCD driver 2013/05/21 */ 
#include <linux/seq_file.h>
#include <asm/uaccess.h>
/** ZTE_MODIFY end chenfei */ 
static unsigned short syna_fw_ver;
static unsigned short syna_fw_3508_Laibao_ver = {0x3048};
static unsigned short syna_fw_3508_Tpk_OGS_ver = {0x3252};
static unsigned short syna_fw_3508_Tpk_GFF_ver = {0x4137};
static unsigned short syna_fw_3508_Goworld_ver = {0x3036};

/** ZTE_MODIFY wangzhanmeng ,   wangzhanmeng modify for different rmi tp fw update comatable 20130411 */
 #if 0 
#define FW_IMAGE_NAME "DSX_P897A21_Tpk.img"
extern u8 firmware_information[4];
#else
extern u8 firmware_information[4];

#define DSX_IMAGE_NAME_TPK	 		"DSX_P897A21_Tpk.img"
#define DSX_IMAGE_NAME_TRULY			"DSX_P897A21_Truly.img"
#define DSX_IMAGE_NAME_SUCCESS	 	"DSX_P897A21_Success.img"
#define DSX_IMAGE_NAME_OFLIM			"DSX_P897A21_Oflim.img"
#define DSX_IMAGE_NAME_LEAD	 		"DSX_P897A21_Lead.img"
#define DSX_IMAGE_NAME_WINTEK		"DSX_P897A21_Wintek.img"
#define DSX_IMAGE_NAME_LAIBAO	 		"DSX_P897A21_Laibao.img"
#define DSX_IMAGE_NAME_CMI			"DSX_P897A21_CMI.img"
#define DSX_IMAGE_NAME_EALY	 		"DSX_P897A21_Eely.img"
#define DSX_IMAGE_NAME_GOWORLD		"DSX_P897A21_Goworld.img"
#define DSX_IMAGE_NAME_BAOMING	 	"DSX_P897A21_Baoming.img"
#define DSX_IMAGE_NAME_EACHOPTO		"DSX_P897A21_Eachopto.img"
#define DSX_IMAGE_NAME_MUTTO	 		"DSX_P897A21_Mutto.img"
#define DSX_IMAGE_NAME_JUNDA			"DSX_P897A21_Junda.img"
#define DSX_IMAGE_NAME_TPK_GFF	 	"DSX_P897A21_Tpk_GFF.img"
#define DSX_IMAGE_NAME_TPK_OGS	 	"DSX_P897A21_Tpk_OGS.img"
#endif
/** ZTE_MODIFY wangzhanmeng ,   wangzhanmeng modify for different rmi tp fw update comatable end */
/* ZTE_MODIFY lizhenhua modified for touch earlysuspend 20130825 */
bool is_flash;
/* ZTE_MODIFY lizhenhua end */
#define CHECKSUM_OFFSET 0x00
#define BOOTLOADER_VERSION_OFFSET 0x07
#define IMAGE_SIZE_OFFSET 0x08
#define CONFIG_SIZE_OFFSET 0x0C
#define PRODUCT_ID_OFFSET 0x10
#define PRODUCT_INFO_OFFSET 0x1E
#define FW_IMAGE_OFFSET 0x100
#define PRODUCT_ID_SIZE 10

#define BOOTLOADER_ID_OFFSET 0
#define FLASH_PROPERTIES_OFFSET 1
#define BLOCK_SIZE_OFFSET 2
#define BLOCK_COUNT_OFFSET 3

#define REG_MAP (1 << 0)
#define UNLOCKED (1 << 1)
#define HAS_CONFIG_ID (1 << 2)
#define HAS_PERM_CONFIG (1 << 3)
#define HAS_BL_CONFIG (1 << 4)
#define HAS_DISP_CONFIG (1 << 5)
#define HAS_CTRL1 (1 << 6)

#define BLOCK_NUMBER_OFFSET 0
#define BLOCK_DATA_OFFSET 1
#define FLASH_COMMAND_OFFSET 2
#define FLASH_STATUS_OFFSET 3

#define UI_CONFIG_AREA 0x00
#define PERM_CONFIG_AREA 0x01
#define BL_CONFIG_AREA 0x02
#define DISP_CONFIG_AREA 0x03

#define CMD_WRITE_FW_BLOCK 0x2
#define CMD_ERASE_ALL 0x3
#define CMD_READ_CONFIG_BLOCK 0x5
#define CMD_WRITE_CONFIG_BLOCK 0x6
#define CMD_ERASE_CONFIG 0x7
#define CMD_ERASE_BL_CONFIG 0x9
#define CMD_ERASE_DISP_CONFIG 0xa
#define CMD_ENABLE_FLASH_PROG 0xf

#define SLEEP_MODE_NORMAL (0x00)
#define SLEEP_MODE_SENSOR_SLEEP (0x01)
#define SLEEP_MODE_RESERVED0 (0x02)
#define SLEEP_MODE_RESERVED1 (0x03)

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (3 * 1000)
#define ERASE_WAIT_MS (5 * 1000)

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100
static struct wake_lock tpfwupdate_wake_lock;

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

struct image_header {
	unsigned int checksum;
	unsigned int image_size;
	unsigned int config_size;
	unsigned char options;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	u8 config_id[4];
};

struct pdt_properties {
	union {
		struct {
			unsigned char reserved_1:6;
			unsigned char has_bsr:1;
			unsigned char reserved_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_flash_status {
	union {
		struct {
			unsigned char status:6;
			unsigned char reserved:1;
			unsigned char program_enabled:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_fwu_handle {
	bool initialized;
	bool has_perm_config;
	bool has_bl_config;
	bool has_disp_config;
	unsigned int image_size;
	unsigned int data_pos;
	unsigned char *ext_data_source;
	unsigned char *read_config_buf;
	unsigned char intr_mask;
	unsigned char command;
	unsigned char bootloader_id[2];
	unsigned char flash_properties;
	unsigned char productinfo1;
	unsigned char productinfo2;
	unsigned short block_size;
	unsigned short fw_block_count;
	unsigned short config_block_count;
	unsigned short perm_config_block_count;
	unsigned short bl_config_block_count;
	unsigned short disp_config_block_count;
	unsigned short config_size;
	unsigned short config_area;
	char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	const unsigned char *firmware_data;
	const unsigned char *config_data;
	struct f34_flash_status flash_status;
	struct synaptics_rmi4_fn_desc f01_fd;
	struct synaptics_rmi4_fn_desc f34_fd;
	struct synaptics_rmi4_exp_fn_ptr *fn_ptr;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct bin_attribute dev_attr_data = {
	.attr = {
		.name = "data",
		.mode = 0644,//.mode = (S_IRUGO | S_IWUGO),/** ZTE_MODIFY wangzhanmeng ,  modify for cts,20130820*/	
	},
	.size = 0,
	.read = fwu_sysfs_show_image,
	.write = fwu_sysfs_store_image,
};

/** ZTE_MODIFY wangzhanmeng ,  modify for cts,20130820*/	
#if 0
static struct device_attribute attrs[] = {
	__ATTR(doreflash, S_IWUGO,
			synaptics_rmi4_show_error,
			fwu_sysfs_do_reflash_store),
	__ATTR(fwinform, S_IRUGO,
			synaptics_rmi4_fw_show,
			synaptics_rmi4_store_error),
	__ATTR(writeconfig, S_IWUGO,
			synaptics_rmi4_show_error,
			fwu_sysfs_write_config_store),
	__ATTR(readconfig, S_IWUGO,
			synaptics_rmi4_show_error,
			fwu_sysfs_read_config_store),
	__ATTR(configarea, S_IWUGO,
			synaptics_rmi4_show_error,
			fwu_sysfs_config_area_store),
	__ATTR(imagesize, S_IWUGO,
			synaptics_rmi4_show_error,
			fwu_sysfs_image_size_store),
	__ATTR(blocksize, S_IRUGO,
			fwu_sysfs_block_size_show,
			synaptics_rmi4_store_error),
	__ATTR(fwblockcount, S_IRUGO,
			fwu_sysfs_firmware_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(configblockcount, S_IRUGO,
			fwu_sysfs_configuration_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(permconfigblockcount, S_IRUGO,
			fwu_sysfs_perm_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(blconfigblockcount, S_IRUGO,
			fwu_sysfs_bl_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(dispconfigblockcount, S_IRUGO,
			fwu_sysfs_disp_config_block_count_show,
			synaptics_rmi4_store_error),
};
#else
static struct device_attribute attrs[] = {
	__ATTR(doreflash, 0644,
			synaptics_rmi4_show_error,
			fwu_sysfs_do_reflash_store),
	__ATTR(fwinform, S_IRUGO,
			synaptics_rmi4_fw_show,
			synaptics_rmi4_store_error),
	__ATTR(writeconfig, 0644,
			synaptics_rmi4_show_error,
			fwu_sysfs_write_config_store),
	__ATTR(readconfig, 0644,
			synaptics_rmi4_show_error,
			fwu_sysfs_read_config_store),
	__ATTR(configarea, 0644,
			synaptics_rmi4_show_error,
			fwu_sysfs_config_area_store),
	__ATTR(imagesize, 0644,
			synaptics_rmi4_show_error,
			fwu_sysfs_image_size_store),
	__ATTR(blocksize, S_IRUGO,
			fwu_sysfs_block_size_show,
			synaptics_rmi4_store_error),
	__ATTR(fwblockcount, S_IRUGO,
			fwu_sysfs_firmware_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(configblockcount, S_IRUGO,
			fwu_sysfs_configuration_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(permconfigblockcount, S_IRUGO,
			fwu_sysfs_perm_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(blconfigblockcount, S_IRUGO,
			fwu_sysfs_bl_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(dispconfigblockcount, S_IRUGO,
			fwu_sysfs_disp_config_block_count_show,
			synaptics_rmi4_store_error),
};
#endif
/** ZTE_MODIFY wangzhanmeng ,  modify for cts,end*/	

static struct synaptics_rmi4_fwu_handle *fwu;

static struct completion remove_complete;

static unsigned int extract_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header *header,
		const unsigned char *fw_image)
{
	header->checksum = extract_uint(&fw_image[CHECKSUM_OFFSET]);
	header->bootloader_version = fw_image[BOOTLOADER_VERSION_OFFSET];
	header->image_size = extract_uint(&fw_image[IMAGE_SIZE_OFFSET]);
	header->config_size = extract_uint(&fw_image[CONFIG_SIZE_OFFSET]);
	memcpy(header->product_id, &fw_image[PRODUCT_ID_OFFSET],
			SYNAPTICS_RMI4_PRODUCT_ID_SIZE);
	header->product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE] = 0;
	memcpy(header->product_info, &fw_image[PRODUCT_INFO_OFFSET],
			SYNAPTICS_RMI4_PRODUCT_INFO_SIZE);
	memcpy(header->config_id, &fw_image[0xef00], 4);

	return;
}

static int fwu_read_f01_device_status(struct f01_device_status *status)
{
	int retval;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f01_fd.data_base_addr,
			status->data,
			sizeof(status->data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read F01 device status\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_read_f34_queries(void)
{
	int retval;
	unsigned char count;
	unsigned char buf[10];

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + BOOTLOADER_ID_OFFSET,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read bootloader ID\n",
				__func__);
		return retval;
	}

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + FLASH_PROPERTIES_OFFSET,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	count = 4;

	if (fwu->flash_properties & HAS_PERM_CONFIG) {
		fwu->has_perm_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_BL_CONFIG) {
		fwu->has_bl_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_DISP_CONFIG) {
		fwu->has_disp_config = 1;
		count += 2;
	}

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + BLOCK_SIZE_OFFSET,
			buf,
			2);
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read block size info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->block_size, &(buf[0]));

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + BLOCK_COUNT_OFFSET,
			buf,
			count);
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read block count info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->fw_block_count, &(buf[0]));
	batohs(&fwu->config_block_count, &(buf[2]));

	count = 4;

	if (fwu->has_perm_config) {
		batohs(&fwu->perm_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_bl_config) {
		batohs(&fwu->bl_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_disp_config)
		batohs(&fwu->disp_config_block_count, &(buf[count]));

	return 0;
}

static int fwu_read_f34_flash_status(void)
{
	int retval;
	unsigned char status;
	unsigned char command;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + FLASH_STATUS_OFFSET,
			&status,
			sizeof(status));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read flash status\n",
				__func__);
		return retval;
	}

	/* Program enabled bit not available - force bit to be set */
	fwu->flash_status.program_enabled = 1;
	fwu->flash_status.status = status & MASK_3BIT;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + FLASH_COMMAND_OFFSET,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read flash command\n",
				__func__);
		return retval;
	}

	fwu->command = command & MASK_4BIT;

	return 0;
}

static int fwu_write_f34_command(unsigned char cmd)
{
	int retval;
	unsigned char command = cmd;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + FLASH_COMMAND_OFFSET,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write command 0x%02x\n",
				__func__, command);
		return retval;
	}

	fwu->command = cmd;

	return 0;
}

static int fwu_wait_for_idle(int timeout_ms)
{
	int count = 0;
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;

	do {
		usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);

		count++;
		if (count == timeout_count)
			fwu_read_f34_flash_status();

		if ((fwu->command == 0x00) &&
				(fwu->flash_status.status == 0x00))
			return 0;
	} while (count < timeout_count);

	dev_err(&fwu->rmi4_data->i2c_client->dev,
			"%s: Timed out waiting for idle status\n",
			__func__);

	return -ETIMEDOUT;
}

static int fwu_scan_pdt(void)
{
	int retval;
	unsigned char ii;
	unsigned char intr_count = 0;
	unsigned char intr_off;
	unsigned char intr_src;
	unsigned short addr;
	bool f01found = false;
	bool f34found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;

	for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
		retval = fwu->fn_ptr->read(fwu->rmi4_data,
				addr,
				(unsigned char *)&rmi_fd,
				sizeof(rmi_fd));
		if (retval < 0)
			return retval;

		if (rmi_fd.fn_number) {
			dev_dbg(&fwu->rmi4_data->i2c_client->dev,
					"%s: Found F%02x\n",
					__func__, rmi_fd.fn_number);
			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				f01found = true;
				fwu->f01_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f01_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f01_fd.data_base_addr =
						rmi_fd.data_base_addr;
				fwu->f01_fd.cmd_base_addr =
						rmi_fd.cmd_base_addr;
				break;
			case SYNAPTICS_RMI4_F34:
				f34found = true;
				fwu->f34_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f34_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f34_fd.data_base_addr =
						rmi_fd.data_base_addr;

				fwu->intr_mask = 0;
				intr_src = rmi_fd.intr_src_count;
				intr_off = intr_count % 8;
				for (ii = intr_off;
						ii < ((intr_src & MASK_3BIT) +
						intr_off);
						ii++) {
					fwu->intr_mask |= 1 << ii;
				}
				break;
			}
		} else {
			break;
		}

		intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
	}

	if (!f01found || !f34found) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to find both F01 and F34\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_write_blocks(unsigned char *block_ptr, unsigned short block_cnt,
		unsigned char command)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;

	block_offset[1] |= (fwu->config_area << 5);

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write to block number registers\n",
				__func__);
		return retval;
	}

	for (block_num = 0; block_num < block_cnt; block_num++) {
		retval = fwu->fn_ptr->write(fwu->rmi4_data,
				fwu->f34_fd.data_base_addr + BLOCK_DATA_OFFSET,
				block_ptr,
				fwu->block_size);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to write block data (block %d)\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_write_f34_command(command);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to write command for block %d\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to wait for idle status (block %d)\n",
					__func__, block_num);
			return retval;
		}

		block_ptr += fwu->block_size;
	}

	return 0;
}

static int fwu_write_firmware(void)
{
	return fwu_write_blocks((unsigned char *)fwu->firmware_data,
		fwu->fw_block_count, CMD_WRITE_FW_BLOCK);
}

static int fwu_write_configuration(void)
{
	return fwu_write_blocks((unsigned char *)fwu->config_data,
		fwu->config_block_count, CMD_WRITE_CONFIG_BLOCK);
}

static int fwu_write_bootloader_id(void)
{
	int retval;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_DATA_OFFSET,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write bootloader ID\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_enter_flash_prog(void)
{
	int retval;
	struct f01_device_status f01_device_status;
	struct f01_device_control f01_device_control;

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	retval = fwu_write_f34_command(CMD_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	retval = fwu_wait_for_idle(ENABLE_WAIT_MS);
	if (retval < 0)
		return retval;

	if (!fwu->flash_status.program_enabled) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Program enabled bit not set\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		return retval;

	retval = fwu_read_f01_device_status(&f01_device_status);
	if (retval < 0)
		return retval;

	if (!f01_device_status.flash_prog) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Not in flash prog mode\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_read_f34_queries();
	if (retval < 0)
		return retval;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f01_fd.ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read F01 device control\n",
				__func__);
		return retval;
	}

	f01_device_control.nosleep = true;
	f01_device_control.sleep_mode = SLEEP_MODE_NORMAL;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f01_fd.ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write F01 device control\n",
				__func__);
		return retval;
	}

	return retval;
}

static int fwu_do_reflash(void)
{
	int retval;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Entered flash prog mode\n",
			__func__);

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Bootloader ID written\n",
			__func__);

	retval = fwu_write_f34_command(CMD_ERASE_ALL);
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Erase all command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Idle status detected\n",
			__func__);

	if (fwu->firmware_data) {
		retval = fwu_write_firmware();
		if (retval < 0)
			return retval;
		pr_notice("%s: Firmware programmed\n", __func__);
	}

	if (fwu->config_data) {
		retval = fwu_write_configuration();
		if (retval < 0)
			return retval;
		pr_notice("%s: Configuration programmed\n", __func__);
	}

	return retval;
}

static int fwu_start_reflash(void)
{
	int retval;
	unsigned short f01_cmd_base_addr;
	struct image_header header;
	const unsigned char *fw_image;
	const struct firmware *fw_entry = NULL;

	char *firmware_name=NULL;/** ZTE_MODIFY wangzhanmeng ,   wangzhanmeng modify for different rmi tp fw update comatable 20130711 */
/* ZTE_MODIFY lizhenhua modified for touch earlysuspend 20130825 */
	is_flash=1;
/* ZTE_MODIFY lizhenhua end */
	f01_cmd_base_addr = fwu->f01_fd.cmd_base_addr;

	if (fwu->rmi4_data->sensor_sleep) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Sensor sleeping\n",
				__func__);
		return -ENODEV;
	}

	fwu->rmi4_data->stay_awake = true;

	pr_notice("%s: Start of reflash process\n", __func__);

	if (fwu->ext_data_source) {
		fw_image = fwu->ext_data_source;
	} else {
/** ZTE_MODIFY wangzhanmeng ,   wangzhanmeng modify for different rmi tp fw update comatable 20130711 */
	#if 0
			dev_dbg(&fwu->rmi4_data->i2c_client->dev,
				"%s: Requesting firmware image %s\n",
				__func__, FW_IMAGE_NAME);

		retval = request_firmware(&fw_entry, FW_IMAGE_NAME,
				&fwu->rmi4_data->i2c_client->dev);
		if (retval != 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Firmware image %s not available\n",
					__func__, FW_IMAGE_NAME);
			return -EINVAL;
		}
	#else



switch (firmware_information[1]){
     	case 0x31:
		if(firmware_information[2] == 0x41)	
			firmware_name = DSX_IMAGE_NAME_TPK_GFF;
		else if(((firmware_information[2] >> 4) & 0x0f) == 0x03)
			firmware_name = DSX_IMAGE_NAME_TPK_OGS;
		else
			firmware_name = DSX_IMAGE_NAME_TPK;
		break ;
	case 0x32:
		firmware_name=DSX_IMAGE_NAME_TRULY;
		break ;	 
	case 0x33:
		firmware_name= DSX_IMAGE_NAME_SUCCESS;
		break ;
	case 0x34:
		firmware_name=DSX_IMAGE_NAME_OFLIM;
		break ;
	case 0x35:
		firmware_name=DSX_IMAGE_NAME_LEAD;
		break ;
	case 0x36:
		firmware_name= DSX_IMAGE_NAME_WINTEK;
		break ;
	case 0x37:
		firmware_name=DSX_IMAGE_NAME_LAIBAO;
		break ;
	case 0x38:
		firmware_name= DSX_IMAGE_NAME_CMI;
		break ;
	case 0x39:
		firmware_name= DSX_IMAGE_NAME_EALY;
		break ;
	case 0x41:
		firmware_name= DSX_IMAGE_NAME_GOWORLD;
		break ;
	case 0x42:
		firmware_name= DSX_IMAGE_NAME_BAOMING;
		break ;
	case 0x43:
		firmware_name=DSX_IMAGE_NAME_EACHOPTO;
		break ;
	case 0x44:
		firmware_name= DSX_IMAGE_NAME_MUTTO;
		break ;
	case 0x45:
		firmware_name= DSX_IMAGE_NAME_JUNDA;
		break ;	 
	 default:
	 break;
}
if(NULL==firmware_name){
	firmware_name= DSX_IMAGE_NAME_TPK;
	printk("\n====www[rmi-tp]:%s()===,line=%d: module type is 0x%x,I do not know the module!!!!!maybe update fail last time???.....use Laibao img for it\n",__func__,__LINE__,firmware_information[1]);
	//return;
	}
//else

	printk("\n====www[rmi-tp]:%s()===,line=%d: Requesting firmware image name:%s\n",__func__,__LINE__,firmware_name);//wangzhanmeng add for test 20130325


	
		dev_dbg(&fwu->rmi4_data->i2c_client->dev,
				"%s: Requesting firmware image %s\n",
				__func__, firmware_name);

		retval = request_firmware(&fw_entry, firmware_name,
				&fwu->rmi4_data->i2c_client->dev);
		if (retval != 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Firmware image %s not available\n",
					__func__, firmware_name);
			//return -EINVAL;
			goto reflash_failed;
		}
	#endif
		dev_dbg(&fwu->rmi4_data->i2c_client->dev,
				"%s: Firmware image size = %d\n",
				__func__, fw_entry->size);

		fw_image = fw_entry->data;
	}

	parse_header(&header, fw_image);

	if (header.image_size)
		fwu->firmware_data = fw_image + FW_IMAGE_OFFSET;
	if (header.config_size) {
		fwu->config_data = fw_image + FW_IMAGE_OFFSET +
				header.image_size;
	}

	printk("\n====www[rmi-tp]:%s()===,line=%d: FW version id=0x%x 0x%x 0x%x 0x%x  \n",__func__,__LINE__,firmware_information[0],firmware_information[1],firmware_information[2],firmware_information[3]);//wangzhanmeng add for test 20130325


	printk("\n====www[rmi-tp]:%s()===,line=%d: Image version id=0x%x 0x%x 0x%x 0x%x  \n",__func__,__LINE__,header.config_id[0], header.config_id[1],header.config_id[2], header.config_id[3]);//wangzhanmeng add for test 20130325
#if 0
	if((firmware_information[0]!=header.config_id[0])&&((firmware_information[0]>=0x31)&&(firmware_information[0]<=0x41))){
		printk("\n====www[rmi-tp]:%s()===,line=%d:The img file is not for this chip type,So we can not update fw!!!!  \n",__func__,__LINE__);//wangzhanmeng add for test 20130325
			return -ENODEV;
	}
	if((firmware_information[1]!=header.config_id[1])&&((firmware_information[1]>=0x31)&&(firmware_information[1]<=0x45))){
		printk("\n====www[rmi-tp]:%s()===,line=%d:The img file is not for this module,So we can not update fw!!!!  \n",__func__,__LINE__);//wangzhanmeng add for test 20130325
			return -ENODEV;
	}

	if((firmware_information[2]==header.config_id[2])&&(firmware_information[3]==header.config_id[3]))
		{
		printk("\n====www[rmi-tp]:%s()===,line=%d: FW version is same as Image file version,so not need update!!!  \n",__func__,__LINE__);//wangzhanmeng add for test 20130325
			return -EEXIST;
	}
#endif
	if(firmware_information[0] == 0x41)//syna 3408
	{
		syna_fw_ver = (firmware_information[3] & 0xff) | ((firmware_information[2] & 0xff)<<8);
		printk("TP firmware update, sensor is 3408\n");
		printk("S3408 firmware no need to update\n");
		goto reflash_failed;		
		/*
		if((firmware_information[2]==syna_fw_3408_Tpk_ver[0])&&(firmware_information[3]==syna_fw_3408_Tpk_ver[1]))
		{
			printk("\n====www[rmi-tp]:%s()===,line=%d: FW version is same as Image file version,so not need update!!!  \n",__func__,__LINE__);
			return -EEXIST;
		}
		if((firmware_information[2]==syna_fw_3408_Truly_ver[0])&&(firmware_information[3]==syna_fw_3408_Truly_ver[1]))
		{
			printk("\n====www[rmi-tp]:%s()===,line=%d: FW version is same as Image file version,so not need update!!!  \n",__func__,__LINE__);
			return -EEXIST;
		}
		*/
	}
	else if(firmware_information[0] == 0x42)
	{
		printk("TP firmware update, sensor is 3508\n");
		syna_fw_ver = (firmware_information[3] & 0xff) | ((firmware_information[2] & 0xff)<<8);
		printk("niugang : 0x%x\n", syna_fw_ver);

		if(firmware_information[1] == 0x37)
		{
			if(syna_fw_ver >= syna_fw_3508_Laibao_ver)
			{
				printk("\n====www[rmi-tp]:%s()===,line=%d: FW version is same as Image file version,so not need update!!!  \n",__func__,__LINE__);
				goto reflash_failed;		
			}
		}
		else if(firmware_information[1] == 0x31)
		{

		if(firmware_information[2] == 0x41)//GFF
		{
		
			if(syna_fw_ver >= syna_fw_3508_Tpk_GFF_ver)
			{
				printk("\n====www[rmi-tp]:%s()===,line=%d: FW version is same as Image file version,so not need update!!!  \n",__func__,__LINE__);
				goto reflash_failed;		
			}
		}
		else if(((firmware_information[2] >> 4) & 0x0f) == 0x03)//OGS
		{
			if(syna_fw_ver >= syna_fw_3508_Tpk_OGS_ver)
			{
				printk("\n====www[rmi-tp]:%s()===,line=%d: FW version is same as Image file version,so not need update!!!  \n",__func__,__LINE__);
				goto reflash_failed;		
			}
		}
		
		}
		else if(firmware_information[1] == 0x41)
		{
			if(syna_fw_ver >= syna_fw_3508_Goworld_ver)
			{
				printk("\n====www[rmi-tp]:%s()===,line=%d: FW version is same as Image file version,so not need update!!!  \n",__func__,__LINE__);
				goto reflash_failed;		
			}
		}
		else
			goto reflash_failed;		
	}

	retval = fwu_do_reflash();
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to do reflash\n",
				__func__);
	}

	fwu->rmi4_data->reset_device(fwu->rmi4_data, f01_cmd_base_addr);
       synaptics_touchscreen_reset();

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.ctrl_base_addr,
			firmware_information,
			4);
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read fw information after reflash\n",
				__func__);
	}

	if (fw_entry)
		release_firmware(fw_entry);

	pr_notice("%s: End of reflash process\n", __func__);

	fwu->rmi4_data->stay_awake = false;
/* ZTE_MODIFY lizhenhua modified for touch earlysuspend 20130825 */
	is_flash=0;
/* ZTE_MODIFY lizhenhua end */
	return retval;

reflash_failed:
	fwu->rmi4_data->reset_device(fwu->rmi4_data, f01_cmd_base_addr);
       synaptics_touchscreen_reset();
	if (fw_entry)
		release_firmware(fw_entry);
	fwu->rmi4_data->stay_awake = false;
	is_flash=0;
	return 0;
}

static int fwu_do_write_config(void)
{
	int retval;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Entered flash prog mode\n",
			__func__);

	if (fwu->config_area == PERM_CONFIG_AREA) {
		fwu->config_block_count = fwu->perm_config_block_count;
		goto write_config;
	}

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Bootloader ID written\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_CONFIG);
		break;
	case BL_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_BL_CONFIG);
		fwu->config_block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_DISP_CONFIG);
		fwu->config_block_count = fwu->disp_config_block_count;
		break;
	}
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Erase command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Idle status detected\n",
			__func__);

write_config:
	retval = fwu_write_configuration();
	if (retval < 0)
		return retval;

	pr_notice("%s: Config written\n", __func__);

	return retval;
}

static int fwu_start_write_config(void)
{
	int retval;
	unsigned short f01_cmd_base_addr;
	struct image_header header;

	f01_cmd_base_addr = fwu->f01_fd.cmd_base_addr;

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config)
			return -EINVAL;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config)
			return -EINVAL;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (fwu->ext_data_source)
		fwu->config_data = fwu->ext_data_source;
	else
		return -EINVAL;

	if (fwu->config_area == UI_CONFIG_AREA) {
		parse_header(&header, fwu->ext_data_source);

		if (header.config_size) {
			fwu->config_data = fwu->ext_data_source +
					FW_IMAGE_OFFSET +
					header.image_size;
		} else {
			return -EINVAL;
		}
	}

	pr_notice("%s: Start of write config process\n", __func__);

	retval = fwu_do_write_config();
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write config\n",
				__func__);
	}

	fwu->rmi4_data->reset_device(fwu->rmi4_data, f01_cmd_base_addr);

	pr_notice("%s: End of write config process\n", __func__);

	return retval;
}

static int fwu_do_read_config(void)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;
	unsigned short block_count;
	unsigned short index = 0;
	unsigned short f01_cmd_base_addr;

	f01_cmd_base_addr = fwu->f01_fd.cmd_base_addr;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		goto exit;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Entered flash prog mode\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->disp_config_block_count;
		break;
	default:
		retval = -EINVAL;
		goto exit;
	}

	fwu->config_size = fwu->block_size * block_count;

	kfree(fwu->read_config_buf);
	fwu->read_config_buf = kzalloc(fwu->config_size, GFP_KERNEL);

	block_offset[1] |= (fwu->config_area << 5);

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write to block number registers\n",
				__func__);
		goto exit;
	}

	for (block_num = 0; block_num < block_count; block_num++) {
		retval = fwu_write_f34_command(CMD_READ_CONFIG_BLOCK);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to write read config command\n",
					__func__);
			goto exit;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to wait for idle status\n",
					__func__);
			goto exit;
		}

		retval = fwu->fn_ptr->read(fwu->rmi4_data,
				fwu->f34_fd.data_base_addr + BLOCK_DATA_OFFSET,
				&fwu->read_config_buf[index],
				fwu->block_size);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to read block data (block %d)\n",
					__func__, block_num);
			goto exit;
		}

		index += fwu->block_size;
	}

exit:
	fwu->rmi4_data->reset_device(fwu->rmi4_data, f01_cmd_base_addr);

	return retval;
}

int synaptics_fw_updater(unsigned char *fw_data)
{
	int retval;

	if (!fwu)
		return -ENODEV;

	if (!fwu->initialized)
		return -ENODEV;

	fwu->ext_data_source = fw_data;
	fwu->config_area = UI_CONFIG_AREA;

	retval = fwu_start_reflash();

	return retval;
}
EXPORT_SYMBOL(synaptics_fw_updater);

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (count < fwu->config_size) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Not enough space (%d bytes) in buffer\n",
				__func__, count);
		return -EINVAL;
	}

	memcpy(buf, fwu->read_config_buf, fwu->config_size);

	return fwu->config_size;
}

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	memcpy((void *)(&fwu->ext_data_source[fwu->data_pos]),
			(const void *)buf,
			count);

	fwu->data_pos += count;

	return count;
}

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input != 1) {
		retval = -EINVAL;
		goto exit;
	}
	wake_lock(&tpfwupdate_wake_lock);
	retval = synaptics_fw_updater(fwu->ext_data_source);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to do reflash\n",
				__func__);
		goto exit;
	}

	retval = count;
	wake_unlock(&tpfwupdate_wake_lock);

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	return retval;
}

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input != 1) {
		retval = -EINVAL;
		goto exit;
	}

	retval = fwu_start_write_config();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write config\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	return retval;
}

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	retval = fwu_do_read_config();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read config\n",
				__func__);
		return retval;
	}

	return count;
}

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long config_area;

	retval = sstrtoul(buf, 10, &config_area);
	if (retval)
		return retval;

	fwu->config_area = config_area;

	return count;
}

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long size;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = sstrtoul(buf, 10, &size);
	if (retval)
		return retval;

	fwu->image_size = size;
	fwu->data_pos = 0;

	kfree(fwu->ext_data_source);
	fwu->ext_data_source = kzalloc(fwu->image_size, GFP_KERNEL);
	if (!fwu->ext_data_source) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for image data\n",
				__func__);
		return -ENOMEM;
	}

	return count;
}


static ssize_t synaptics_rmi4_fw_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int len = 0;

	len += sprintf(buf + len, "IC i2c address: 0x%x\n", 0x22);
switch (firmware_information[0]){
     	case 0x31:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s2200 series");
		break ;
	case 0x32:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s2202 series");
		break ;
	case 0x33:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s3200 series");
		break ;
	case 0x34:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s3202 series");
		break ;
	case 0x35:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s3203 series");
		break ;
	case 0x36:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s7020 series");
		break ;
	case 0x37:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s7300 series");
		break ;
	case 0x41:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s3408 series");
		break;
       case 0x42:
		len += sprintf(buf + len, "synaptics IC type: %s\n", "s3508 series");		
		break ;
	 default:
	 	len += sprintf(buf + len, "synaptics IC type : %s\n", "I do not know");
}
switch (firmware_information[1]){
     
	case 0x31:
	 	len += sprintf(buf + len, "module : %s\n", "TPK");
		break ;
	case 0x32:
	 	len += sprintf(buf + len, "module : %s\n", "Truly");
		break ;
	case 0x33:
	 	len += sprintf(buf + len, "module : %s\n", "Success");
		break ;
	case 0x34:
	 	len += sprintf(buf + len, "module : %s\n", "Oflim");
		break ;
	case 0x35:
	 	len += sprintf(buf + len, "module : %s\n", "Lead");
		break ;
	case 0x36:
	 	len += sprintf(buf + len, "module : %s\n", "Wintek");
		break ;
	case 0x37:
	 	len += sprintf(buf + len, "module : %s\n", "Laibao");
		break ;
	case 0x38:
	 	len += sprintf(buf + len, "module : %s\n", "CMI");
		break ;
	case 0x39:
	 	len += sprintf(buf + len, "module : %s\n", "Eely");
		break ;
	case 0x41:
	 	len += sprintf(buf + len, "module : %s\n", "Goworld");
		break ;
	case 0x42:
	 	len += sprintf(buf + len, "module : %s\n", "Baoming");
		break ;
	case 0x43:
	 	len += sprintf(buf + len, "module : %s\n", "Eachopto");
		break ;
	case 0x44:
	 	len += sprintf(buf + len, "module : %s\n", "Mutto");
		break ;
	case 0x45:
	 	len += sprintf(buf + len, "module : %s\n", "Junda");
		break ;	 
	 default:
	 	len += sprintf(buf + len, "module : %s\n", "I do not know");
}

	len += sprintf(buf + len, "firmware version : 0x%02x,0x%02x\n", firmware_information[2],firmware_information[3]);
	return len;
}

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->block_size);
}

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->fw_block_count);
}

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->config_block_count);
}

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->perm_config_block_count);
}

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->bl_config_block_count);
}

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->disp_config_block_count);
}

static void synaptics_rmi4_fwu_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	if (fwu->intr_mask & intr_mask)
		fwu_read_f34_flash_status();

	return;
}

/** ZTE_MODIFY chenfei TP info 2013/08/30 */ 

//use /proc/touchscreen/TouchScreen_info to show TP info
static int TP_info_proc_show(struct seq_file *m, void *v)
{
	int len = 0;
	char buf[256];

	len += sprintf(buf + len, "Name: Synaptics\n");

	len += sprintf(buf + len, "I2C address: 0x%x\n", 0x22);
	
	switch (firmware_information[0]){
		case 0x41:
			len += sprintf(buf + len, "IC type: s3408\n");
			break ;
		case 0x42:
			len += sprintf(buf + len, "IC type: s3508\n");
			break ;	
	 	default:
	 		len += sprintf(buf + len, "IC type: unkown yet\n");
	}

	switch (firmware_information[1]){
     
	case 0x31:
	 	len += sprintf(buf + len, "Module: TPK\n");
		break ;
	case 0x32:
	 	len += sprintf(buf + len, "Module: Truly\n");
		break ;
	case 0x37:
	 	len += sprintf(buf + len, "Module: Laibao\n");
		break ;
	case 0x41:
	 	len += sprintf(buf + len, "Module: Goworld\n");
		break ;
	default:
	 	len += sprintf(buf + len, "Module: unkown yet\n");
	}	

	len += sprintf(buf + len, "Firmware version: %x\n", (firmware_information[3] & 0xff) | ((firmware_information[2] & 0xff)<<8));

	seq_printf(m, "%s", buf);
	
	return 0;
}

static int TP_info_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, TP_info_proc_show, NULL);
}

static const struct file_operations TP_info_proc_fops = {
	.open		= TP_info_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int TP_info_proc_init(void)
{
 	struct proc_dir_entry *res, *dir;

	dir = proc_mkdir("touchscreen", NULL);
	if (!dir)
	{
		printk(KERN_INFO "failed to create dir /proc/touchscreen\n");
		return -ENOMEM;
	}
	
	res = proc_create("TouchScreen_Info",  S_IRUGO, dir,
			  &TP_info_proc_fops);
	if (!res)
	{
		printk(KERN_INFO "failed to create /proc/touchscreen/TouchScreen_info\n");
		return -ENOMEM;
	}

	printk(KERN_INFO "created /proc/touchscreen/TouchScreen_Info\n");
	return 0;
}
/** ZTE_MODIFY end chenfei */ 


static int synaptics_rmi4_fwu_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char attr_count;
	struct pdt_properties pdt_props;

	fwu = kzalloc(sizeof(*fwu), GFP_KERNEL);
	if (!fwu) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fwu\n",
				__func__);
		goto exit;
	}

	fwu->fn_ptr = kzalloc(sizeof(*(fwu->fn_ptr)), GFP_KERNEL);
	if (!fwu->fn_ptr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fn_ptr\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_fwu;
	}

	fwu->rmi4_data = rmi4_data;
	fwu->fn_ptr->read = rmi4_data->i2c_read;
	fwu->fn_ptr->write = rmi4_data->i2c_write;
	fwu->fn_ptr->enable = rmi4_data->irq_enable;

	retval = fwu->fn_ptr->read(rmi4_data,
			PDT_PROPS,
			pdt_props.data,
			sizeof(pdt_props.data));
	if (retval < 0) {
		dev_dbg(&rmi4_data->i2c_client->dev,
				"%s: Failed to read PDT properties, assuming 0x00\n",
				__func__);
	} else if (pdt_props.has_bsr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Reflash for LTS not currently supported\n",
				__func__);
		goto exit_free_mem;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		goto exit_free_mem;

	fwu->productinfo1 = rmi4_data->rmi4_mod_info.product_info[0];
	fwu->productinfo2 = rmi4_data->rmi4_mod_info.product_info[1];
	memcpy(fwu->product_id, rmi4_data->rmi4_mod_info.product_id_string,
			SYNAPTICS_RMI4_PRODUCT_ID_SIZE);
	fwu->product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE] = 0;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: F01 product info: 0x%04x 0x%04x\n",
			__func__, fwu->productinfo1, fwu->productinfo2);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"%s: F01 product ID: %s\n",
			__func__, fwu->product_id);

	retval = fwu_read_f34_queries();
	if (retval < 0)
		goto exit_free_mem;

	fwu->initialized = true;

	retval = sysfs_create_bin_file(&rmi4_data->input_dev->dev.kobj,
			&dev_attr_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs bin file\n",
				__func__);
		goto exit_free_mem;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto exit_remove_attrs;
		}
	}
	
	TP_info_proc_init();/** ZTE_MODIFY chenfei TP info 2013/08/30 */ 
	
	init_completion(&remove_complete);
	wake_lock_init(&tpfwupdate_wake_lock, WAKE_LOCK_SUSPEND, "tp_fwupdate");

	return 0;

exit_remove_attrs:
for (attr_count--; attr_count >= 0; attr_count--) {
	sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
			&attrs[attr_count].attr);
}

sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

exit_free_mem:
	kfree(fwu->fn_ptr);

exit_free_fwu:
	kfree(fwu);

exit:
	return 0;
}

static void synaptics_rmi4_fwu_remove(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char attr_count;

	sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	kfree(fwu->fn_ptr);
	kfree(fwu);

	complete(&remove_complete);

	return;
}

static int __init rmi4_fw_update_module_init(void)
{
	synaptics_rmi4_new_function(RMI_FW_UPDATER, true,
			synaptics_rmi4_fwu_init,
			synaptics_rmi4_fwu_remove,
			synaptics_rmi4_fwu_attn);
	return 0;
}

static void __exit rmi4_fw_update_module_exit(void)
{
	synaptics_rmi4_new_function(RMI_FW_UPDATER, false,
			synaptics_rmi4_fwu_init,
			synaptics_rmi4_fwu_remove,
			synaptics_rmi4_fwu_attn);
	wait_for_completion(&remove_complete);
	return;
}

module_init(rmi4_fw_update_module_init);
module_exit(rmi4_fw_update_module_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX FW Update Module");
MODULE_LICENSE("GPL v2");
