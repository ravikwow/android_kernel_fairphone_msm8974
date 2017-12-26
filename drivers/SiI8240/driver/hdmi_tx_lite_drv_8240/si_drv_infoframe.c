/**********************************************************************************
 * Si8240 Linux Driver
 *
 * Copyright (C) 2011-2012 Silicon Image Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed .as is. WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the
 * GNU General Public License for more details.
 *
 **********************************************************************************/

#include "si_memsegsupport.h"
#include "si_common.h"
#ifdef __KERNEL__
#include "sii_hal.h"
#else
#include "hal_timers.h"
#endif
#include "si_tpi_regs.h"
#include "si_cra.h"
#include "si_cra_cfg.h"
#include "si_mhl_defs.h"
#include "si_mhl_tx_base_drv_api.h"
#include "si_hdmi_tx_lite_api.h"
#include "si_app_devcap.h"
#include "si_bitdefs.h"
#include "si_drv_hdmi_tx_lite_edid.h"
extern unsigned char VIDEO_CAPABILITY_D_BLOCK_found;


uint8_t CalculateGenericCheckSum(uint8_t *infoFrameData,uint8_t checkSum,uint8_t length)
{
uint8_t i;
    for (i = 0; i < length; i++)
    {
        checkSum += infoFrameData[i];
    }
    checkSum = 0x100 - checkSum;

    INFO_DEBUG_PRINT(("checkSum: %02x\n",checkSum));
	return checkSum;
}

#define SIZE_AUDIO_INFOFRAME 14
//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  SendAudioInfoFrame()
//
// PURPOSE       :  Load Audio InfoFrame data into registers and send to sink
//
// INPUT PARAMS  :  (1) Channel count (2) speaker configuration per CEA-861D
//                  Tables 19, 20 (3) Coding type: 0x09 for DSD Audio. 0 (refer
//                                      to stream header) for all the rest (4) Sample Frequency. Non
//                                      zero for HBR only (5) Audio Sample Length. Non zero for HBR
//                                      only.
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  None
//
// RETURNS       :  true
//
//////////////////////////////////////////////////////////////////////////////
void SendAudioInfoFrame (void)
{
    if (SiiMhlTxOutputModeIsHDMI())
    {
    uint8_t ifData[SIZE_AUDIO_INFOFRAME];
    uint8_t i;


        ifData[0] = 0x84;
        ifData[1] = 0x01;
        ifData[2] = 0x0A;
        for (i = 3; i < SIZE_AUDIO_INFOFRAME;++i)
        {
            ifData[i] = 0;
        }

        ifData[3] = CalculateGenericCheckSum(ifData,0,SIZE_AUDIO_INFOFRAME);


    	SiiRegWrite(REG_TPI_INFO_FSEL
    	        , BIT_TPI_INFO_EN
    	        | BIT_TPI_INFO_RPT
    	        | BIT_TPI_INFO_READ_FLAG_NO_READ
    	        | BIT_TPI_INFO_SEL_Audio
    	        );
        SiiRegWriteBlock(REG_TPI_INFO_BYTE00,ifData,SIZE_AUDIO_INFOFRAME);

        INFO_DEBUG_PRINT(("REG_TPI_INFO_BYTE13: %02x\n",REG_TPI_INFO_BYTE13));
    }
}
#define SIZE_AVI_INFOFRAME				14
static uint8_t CalculateAviInfoFrameChecksum (PHwAviPayLoad_t pPayLoad)
{
	uint8_t checksum;

	checksum = 0x82 + 0x02 + 0x0D;  // these are set by the hardware
    return CalculateGenericCheckSum(pPayLoad->ifData,checksum,SIZE_AVI_INFOFRAME);
}

HwAviPayLoad_t aviPayLoad;
void SiiHmdiTxLiteDrvSendHwAviInfoFrame( void )
{
    if (SiiMhlTxOutputModeIsHDMI())
    {
    	SiiRegWrite(REG_TPI_INFO_FSEL
    	        , BIT_TPI_INFO_EN
    	        | BIT_TPI_INFO_RPT
    	        | BIT_TPI_INFO_READ_FLAG_NO_READ
    	        | BIT_TPI_INFO_SEL_AVI
    	        );
    	SiiRegWriteBlock(REG_TPI_AVI_CHSUM, (uint8_t*)&aviPayLoad.ifData, sizeof(aviPayLoad.ifData));
        INFO_DEBUG_PRINT((" outgoing info frame: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n"
                    ,aviPayLoad.ifData[0x0]
                    ,aviPayLoad.ifData[0x1]
                    ,aviPayLoad.ifData[0x2]
                    ,aviPayLoad.ifData[0x3]
                    ,aviPayLoad.ifData[0x4]
                    ,aviPayLoad.ifData[0x5]
                    ,aviPayLoad.ifData[0x6]
                    ,aviPayLoad.ifData[0x7]
                    ,aviPayLoad.ifData[0x8]
                    ,aviPayLoad.ifData[0x9]
                    ,aviPayLoad.ifData[0xA]
                    ,aviPayLoad.ifData[0xB]
                    ,aviPayLoad.ifData[0xC]
                    ,aviPayLoad.ifData[0xD]
                    ));
    }
}

static void SiiHdmiTxLiteDrvPrepareHwAviInfoFrame( PHwAviPayLoad_t pPayLoad )
{
//uint8_t                 outputClrSpc;  //del by wangzhanmeng for compile error 20130121
AviInfoFrameDataByte2_t colorimetryAspectRatio;
uint8_t inputVideoCode;
QuantizationSettings_e  settings;

    //outputClrSpc            = SiiMhlTxDrvGetOutputColorSpace();  // originates from EDID

    colorimetryAspectRatio  = SiiMhlTxDrvGetColorimetryAspectRatio();
    inputVideoCode          = SiiMhlTxDrvGetInputVideoCode();

    pPayLoad->namedIfData.checksum = 0; // the checksum itself is included in the calculation.

    //pPayLoad->namedIfData.ifData_u.bitFields.pb1.colorSpace = outputClrSpc&0x7F;
    pPayLoad->namedIfData.ifData_u.bitFields.colorimetryAspectRatio = colorimetryAspectRatio;
    pPayLoad->namedIfData.ifData_u.bitFields.VIC.VIC = inputVideoCode;

    settings = qsAutoSelectByColorSpace;
	#if 0
    if (EDID_Data.VideoCapabilityFlags & 0x80)
    {
        switch(outputClrSpc)
        {
        case acsRGB:
			switch (VIDEO_CAPABILITY_D_BLOCK_found)
            {
            case 1:
                settings = qsLimitedRange;
                break;
            case 0:
                settings = qsFullRange;
                break;
				}
			break;
        case acsYCbCr422:
        case acsYCbCr444:
            switch (pPayLoad->namedIfData.ifData_u.bitFields.pb5.quantization)
            {
            case aqLimitedRange:
                settings = qsLimitedRange;
                break;
            case aqFullRange:
                settings = qsFullRange;
                break;
            case aqReserved0:
            case aqReserved1:
                    // undefined by CEA-861-E
                break;
            }
            break;
        }
    }
	#endif
	switch (VIDEO_CAPABILITY_D_BLOCK_found)
            {
            case 1:
                settings = qsLimitedRange;
				pPayLoad->namedIfData.ifData_u.bitFields.pb3.RGBQuantizationRange=1;
                break;
            case 0:
                settings = qsFullRange;
				pPayLoad->namedIfData.ifData_u.bitFields.pb3.RGBQuantizationRange=0;
                break;
			default:
				settings = qsFullRange;
				pPayLoad->namedIfData.ifData_u.bitFields.pb3.RGBQuantizationRange=0;
				break;
				}
		//pPayLoad->namedIfData.ifData_u.bitFields.pb3.RGBQuantizationRange=1;	
	TXD_DEBUG_PRINT(("Drv: VIDEO_CAPABILITY_D_BLOCK_found settings:%d ,VIDEO_CAPABILITY_D_BLOCK_found=0x%02x\n",settings,VIDEO_CAPABILITY_D_BLOCK_found));
        
    SiiMhlTxDrvSetOutputQuantizationRange(settings);
    SiiMhlTxDrvSetInputQuantizationRange(qsFullRange);

    pPayLoad->namedIfData.checksum = CalculateAviInfoFrameChecksum(pPayLoad);
    //INFO_DEBUG_PRINT(("Drv: outputClrSpc: %02x\n",outputClrSpc));

    DumpIncomingInfoFrame(pPayLoad,sizeof(*pPayLoad));


    // output color space value chosen by EDID parser
    //  and input settings from incoming info frame
    SiiMhlTxDrvApplyColorSpaceSettings();

    aviPayLoad = *pPayLoad;
    // don't send the info frame until we un-mute the output
    //SiiHmdiTxLiteDrvSendHwAviInfoFrame( );

}

void SiiHmdiTxLiteDrvPrepareAviInfoframe ( void )
{
#ifdef AVI_PASSTHROUGH //(
    INFO_DEBUG_PRINT(("PrepareAviInfoframe\n"));
   // SiiHdmiTxLiteDrvSendHwAviInfoFrame(&aviPayLoad);
	SiiHmdiTxLiteDrvSendHwAviInfoFrame();
#else //)(
HwAviPayLoad_t hwIfData;
    INFO_DEBUG_PRINT(("PrepareAviInfoframe\n"));
	hwIfData.namedIfData.ifData_u.bitFields.VIC.VIC = SiiMhlTxDrvGetInputVideoCode();
		TXD_DEBUG_PRINT(("inputVideoCode=%02X,hwIfData.namedIfData.ifData_u.bitFields.VIC.VIC=0x%x\n", 
		SiiMhlTxDrvGetInputVideoCode(),hwIfData.namedIfData.ifData_u.bitFields.VIC.VIC));
		
	if((SiiMhlTxDrvGetInputVideoCode()==0x10)|(SiiMhlTxDrvGetInputVideoCode()==0x1F))
		hwIfData.namedIfData.ifData_u.bitFields.pb1.colorSpace = 1;//Ycbcr422
	else
		hwIfData.namedIfData.ifData_u.bitFields.pb1.colorSpace = 0;//RGB
	hwIfData.namedIfData.ifData_u.bitFields.pb1.futureMustBeZero = 0; // originates from incoming infoframe
	hwIfData.namedIfData.ifData_u.bitFields.pb1.ScanInfo = 2; // originates from incoming infoframe
	hwIfData.namedIfData.ifData_u.bitFields.pb1.ActiveFormatInfoPresent = 0; // originates from incoming infoframe
	hwIfData.namedIfData.ifData_u.bitFields.colorimetryAspectRatio = SiiMhlTxDrvGetColorimetryAspectRatio();
	hwIfData.ifData[0x3] = 0x00;

	hwIfData.namedIfData.ifData_u.bitFields.pb3.RGBQuantizationRange=1;

	TXC_DEBUG_PRINT(("\t\thwIfData.namedIfData.ifData_u.bitFields.VIC.VIC:%d\n, hwIfData.namedIfData.ifData_u.bitFields.pb1.colorSpace=0x%x,\nhwIfData.namedIfData.ifData_u.bitFields.colorimetryAspectRatio=0x%x\n",
		(uint16_t)hwIfData.namedIfData.ifData_u.bitFields.VIC.VIC,hwIfData.namedIfData.ifData_u.bitFields.pb1.colorSpace,hwIfData.namedIfData.ifData_u.bitFields.colorimetryAspectRatio));
	hwIfData.ifData[0x5] = 0x00;
	hwIfData.ifData[0x6] = 0x00;
	hwIfData.ifData[0x7] = 0x00;
	hwIfData.ifData[0x8] = 0x00;
	hwIfData.ifData[0x9] = 0x00;
	hwIfData.ifData[0xA] = 0x00;
	hwIfData.ifData[0xB] = 0x00;
	hwIfData.ifData[0xC] = 0x00;
	hwIfData.ifData[0xD] = 0x00;
#if 0
	if(VIDEO_CAPABILITY_D_BLOCK_found){
		hwIfData.ifData[6] = 0x04; 
		TXC_DEBUG_PRINT(("VIDEO_CAPABILITY_D_BLOCK_found = true, limited range\n"));
		}
	else{
    	hwIfData.ifData[6] = 0x00;  
		TXC_DEBUG_PRINT(("VIDEO_CAPABILITY_D_BLOCK_found= false. defult range\n"));
		}
#endif
    SiiHdmiTxLiteDrvPrepareHwAviInfoFrame(&hwIfData);
#endif //)
}



