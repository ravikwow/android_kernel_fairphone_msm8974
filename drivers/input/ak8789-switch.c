/**Ak8989 switch interface*/
#include	<linux/fs.h>
#include	<linux/gpio.h>
#include	<linux/irq.h>
#include	<linux/err.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>

#define ak8789_gpio 68

struct ak8789_chip {
	struct mutex lock;
	struct input_dev *ak8789_idev;
	struct work_struct work;
	int irq_sensor;
	bool ak8789_enabled;
}*ak8789_chip_data;

extern void synaptics_touchscreen_gloved_finger(bool enable); /*ZTE_MODIFY wamgzhanmeng,add for changing  gloved function dynamic 20130817 */
extern int g_is_touchscreen_window;


static struct work_struct init_irq_work;
static struct timer_list hall_timer;
static int m_igpio_value = -1;

static int m_isystemboot = -1;


static void ak8789_timer_delet(void);
static irqreturn_t ak8789_enable_irq(int irq, void *handle);


static void ak8789_timer_func(void)
{
    schedule_work(&init_irq_work);
    return;
}

/*
static void init_irq_func(struct work_struct * work) 
{
	int ret = 0;
	// switch to irq 

	ret = request_threaded_irq(ak8789_chip_data->irq_sensor, NULL, ak8789_enable_irq,
			  IRQ_TYPE_EDGE_BOTH,
			  "ak8789", NULL);

	printk("%s: Ak8789 gavin ======switch to irq ret:%d.\n", __func__, ret);
	if (ret)
	{
		printk("%s:ak8789 irq error=======\n",__func__);
		if (ak8789_chip_data->irq_sensor > 0)
			free_irq(ak8789_chip_data->irq_sensor, NULL);
	}
	else
	{
		enable_irq_wake(ak8789_chip_data->irq_sensor);
	}

}
*/



static ssize_t hall_system_store(struct kobject *kobj,
	struct kobj_attribute *attr,
	const char *buf,
	size_t count)
{
	sscanf(buf, "%d\n", &m_isystemboot);
	printk("ak8789 hall_system_store m_isystemboot:%d\n", m_isystemboot);

	return count;
}

static ssize_t hall_system_show(struct kobject *kobj,
	struct kobj_attribute *attr,
	char *buf)
{

	int ret = sprintf(buf, "%d\n", m_isystemboot);
	printk("ak8789 hall_system_show m_isystemboot:%d\n", m_isystemboot);
	
	return ret;
}


static struct kobject *hall_system_obj;
static struct kobj_attribute hall_system_attribute = __ATTR(hall, 0660, hall_system_show, hall_system_store);
static struct attribute *attrs[] = {
	&hall_system_attribute.attr,
	NULL,
};
static struct attribute_group attr_group = {
	.attrs = attrs,
};


static void ak8789_sys_hall_init(void)
{
	int ret = 0;
	hall_system_obj = NULL;

	hall_system_obj = kobject_create_and_add("hall_system", kernel_kobj);
	
	if (!hall_system_obj) {
		printk("%s: sysfs create and add failed\n", __func__);
		goto error_return;
	}

	ret = sysfs_create_group(hall_system_obj, &attr_group);
	if (ret) {
		printk("%s: sysfs create group failed %d\n", __func__, ret);
		goto error_return;
	}

	printk("%s : ak8789 ak8789_sys_hall_init success=================================\n", __func__);
	return;

error_return:


	printk("%s : ak8789 ak8789_sys_hall_init failed=================================\n", __func__);
	if (hall_system_obj) {
		kobject_del(hall_system_obj);
		hall_system_obj = NULL;
	}

	return;
}

static void init_irq_func(struct work_struct * work) 
{
	int ret = 0;
	int value = 0;

	printk("%s : ak8789====init_irq_func=m_isystemboot:%d\n", __func__, m_isystemboot);

	if (1 != m_isystemboot)
	{
		mod_timer(&hall_timer, jiffies + 1*HZ);
		return;
	}

	value = gpio_get_value(ak8789_gpio);	
	printk("%s : ak8789====gpio_get_value=value:%d\n", __func__, value);

	// input hall key event

	if(ak8789_chip_data->ak8789_enabled)
	{
		printk("%s : ak8789 input_report_key=%d=======================================\n", __func__, value);

		if(value==1){
			//synaptics_touchscreen_gloved_finger(false);/*ZTE_MODIFY wamgzhanmeng,add for changing  gloved function dynamic 20130817 */
			//g_is_touchscreen_window = 0;
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_UP, 1);
			input_sync(ak8789_chip_data->ak8789_idev);
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_UP, 0);
		} else {
			//synaptics_touchscreen_gloved_finger(true);/*ZTE_MODIFY wamgzhanmeng,add for changing  gloved function dynamic 20130817 */
			//g_is_touchscreen_window = 1;
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_DOWN, 1);
			input_sync(ak8789_chip_data->ak8789_idev);
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_DOWN, 0);
		}
		input_sync(ak8789_chip_data->ak8789_idev);
	}

	// delet timer
	ak8789_timer_delet();

	// switch to irq 
	ret = request_threaded_irq(ak8789_chip_data->irq_sensor, NULL, ak8789_enable_irq,
			  IRQ_TYPE_EDGE_BOTH,
			  "ak8789", NULL);

	printk("%s: Ak8789 gavin ======switch to irq ret:%d.\n", __func__, ret);
	if (ret)
	{
		printk("%s:ak8789 irq error=======\n",__func__);
		if (ak8789_chip_data->irq_sensor > 0)
			free_irq(ak8789_chip_data->irq_sensor, NULL);

		mod_timer(&hall_timer, jiffies + 1*HZ);
	}
	else
	{
		/* Enable GPIO_49 as wakeup source */ 
		enable_irq_wake(ak8789_chip_data->irq_sensor);
	}


}




static void hall_timer_callback(unsigned long data)
{
	// process hall proc
	//ak8789_init_irq();
	ak8789_timer_func();


/*
	int ret = 0;
	int value = 0;
	value = gpio_get_value(ak8789_gpio);	
	printk("ak8789================gpio_get_value=value:%d", value);

	if (m_igpio_value == value)  // when gpio is still 
	{
		// input hall key event

		if(ak8789_chip_data->ak8789_enabled)
		{
			if(value==1)
			{
				input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_UP, 1);
				input_sync(ak8789_chip_data->ak8789_idev);
				input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_UP, 0);
			} else {
				input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_DOWN, 1);
				input_sync(ak8789_chip_data->ak8789_idev);
				input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_DOWN, 0);
			}
			input_sync(ak8789_chip_data->ak8789_idev);
		}

		// delet timer
		ak8789_timer_delet();

		// init irq 
		ak8789_init_irq();
	}
	else
	{
		printk("%s: Ak8789 gavin ======first get gpio m_igpio_value:%d.\n", __func__, m_igpio_value);
		m_igpio_value = value;
		mod_timer(&hall_timer, jiffies + 1*HZ);
	}
	*/

}


static void ak8789_timer_init(void)
{

	init_timer(&hall_timer);
	hall_timer.expires = jiffies + 15*HZ;
	hall_timer.data = (unsigned long)0;
	hall_timer.function = hall_timer_callback;

	m_igpio_value = -1;
	add_timer(&hall_timer);
}

static void ak8789_timer_delet(void)
{
	del_timer(&hall_timer);
	printk("ak8789====ak8789_timer_delet==============\n");
}

static irqreturn_t ak8789_enable_irq(int irq, void *handle)
{
    schedule_work(&ak8789_chip_data->work);
    return IRQ_HANDLED;
}

static void ak8789_work_func(struct work_struct * work) 
{
	int value;
	
	value=gpio_get_value(ak8789_gpio);	
	printk("%s: Ak8789 ==========value:%d.\n", __func__, value);

	if(ak8789_chip_data->ak8789_enabled){
/** ZTE_MODIFY yuehongliang add hall sensor, 2013-07-09 */ 
		if(value==1){
			synaptics_touchscreen_gloved_finger(false);/*ZTE_MODIFY wamgzhanmeng,add for changing  gloved function dynamic 20130817 */
			g_is_touchscreen_window = 0;
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_UP, 1);
			input_sync(ak8789_chip_data->ak8789_idev);
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_UP, 0);
		} else {
			synaptics_touchscreen_gloved_finger(true);/*ZTE_MODIFY wamgzhanmeng,add for changing  gloved function dynamic 20130817 */
			g_is_touchscreen_window = 1;
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_DOWN, 1);
			input_sync(ak8789_chip_data->ak8789_idev);
			input_report_key(ak8789_chip_data->ak8789_idev, KEY_HALL_SENSOR_DOWN, 0);
		}
		input_sync(ak8789_chip_data->ak8789_idev);
/** ZTE_MODIFY end yuehongliang, 2013-07-09 */
	}
}


static ssize_t ak8789_device_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int val;
	val=gpio_get_value(ak8789_gpio);
	return sprintf(buf, "%d\n", val);
}

static ssize_t  ak8789_device_enable(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{

	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	//printk("%s:zhangxiaolei test value=%d,size=%d\n",__func__,value,size);
	if (value)	
	{
		if(!ak8789_chip_data->ak8789_enabled)
		{
			ak8789_chip_data->ak8789_enabled = true;
		}
	}
	else
	{
		if(ak8789_chip_data->ak8789_enabled)
		{
			ak8789_chip_data->ak8789_enabled=false;
		}
    }	
	return size;
}

static struct device_attribute ak8789_attrs[] = {
	__ATTR(ak8789_enable, 0444, ak8789_device_enable_show, ak8789_device_enable),
};

static int add_sysfs_interfaces(struct device *dev,
	struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	printk("%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}


static int __init ak8789_init(void)
{

	int err,ret;
	//printk(KERN_INFO "zhangxiaolei ak8789 : init\n");

	ak8789_chip_data = kzalloc(sizeof(struct ak8789_chip), GFP_KERNEL);
	if (!ak8789_chip_data) {
		ret = -ENOMEM;
		printk("%s:chip kzalloc failed\n",__func__);
		return ret;
	}
	mutex_init(&ak8789_chip_data->lock);

	printk("%s: Ak8789 ===1============ak8789_init.\n", __func__);

	INIT_WORK(&(ak8789_chip_data->work),ak8789_work_func);

	INIT_WORK(&init_irq_work, init_irq_func);
	
	ak8789_chip_data->ak8789_idev = input_allocate_device();
	if (!ak8789_chip_data->ak8789_idev) {
		printk("%s:ak8789 error1\n",__func__);
		ret = -ENODEV;
		return ret;
	}
/** ZTE_MODIFY yuehongliang add hall sensor, 2013-07-09 */ 
	ak8789_chip_data->ak8789_idev->name = "AK8789";
	set_bit(KEY_POWER, ak8789_chip_data->ak8789_idev->keybit);
        set_bit(KEY_HALL_SENSOR_DOWN, ak8789_chip_data->ak8789_idev->keybit);
        set_bit(KEY_HALL_SENSOR_UP, ak8789_chip_data->ak8789_idev->keybit);              
	set_bit(EV_KEY, ak8789_chip_data->ak8789_idev->evbit);
/** ZTE_MODIFY end yuehongliang, 2013-07-09 */	
	ret = input_register_device(ak8789_chip_data->ak8789_idev);
	if (ret) {
		input_free_device(ak8789_chip_data->ak8789_idev);
		printk("%s:ak8789 error2\n",__func__);
		return ret;
	}

	ret = add_sysfs_interfaces(&ak8789_chip_data->ak8789_idev->dev,
			ak8789_attrs, ARRAY_SIZE(ak8789_attrs));
	if (ret)
		goto input_a_sysfs_failed;
	
	ak8789_chip_data->irq_sensor = gpio_to_irq(ak8789_gpio); 
		/* Check if GPIO_49 pin is in use */ 
	err=gpio_request(ak8789_gpio, "ak8789"); 

	if (err) {
		printk("%s:ak8789 error3\n",__func__);
		}
		/* Set GPIO_49 pin as input */ 
	err=gpio_direction_input(ak8789_gpio); 

		/*
	ret = request_threaded_irq(ak8789_chip_data->irq_sensor, NULL, ak8789_enable_irq,
		      IRQ_TYPE_EDGE_BOTH,
		      "ak8789", NULL);
*/

/*
	ret = request_threaded_irq(ak8789_chip_data->irq_sensor, NULL, ak8789_enable_irq,
		      IRQ_TYPE_LEVEL_LOW,
		      "ak8789", NULL);
*/

	ak8789_sys_hall_init();
		
	ak8789_timer_init();

	printk("%s:Ak8789 gavin Init timer init, irq_sensor!\n",__func__);


	if (ret) {
		printk("%s:ak8789 error4\n",__func__);
			goto input_a_sysfs_failed;
		}
		/* Enable GPIO_49 as wakeup source */ 
	//enable_irq_wake(ak8789_chip_data->irq_sensor);
	ak8789_chip_data->ak8789_enabled=true;


	
	printk("%s:Ak8789 Init OK!\n",__func__);
	return 0;

input_a_sysfs_failed:
	input_unregister_device(ak8789_chip_data->ak8789_idev);
	printk("%s:Ak8789 Init Failed!\n",__func__);
	return ret;
}

		
static void __exit ak8789_exit(void)
{
	gpio_free(ak8789_gpio);
}

module_init(ak8789_init);
module_exit(ak8789_exit);
