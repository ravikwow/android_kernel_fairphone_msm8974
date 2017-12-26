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

#ifdef __KERNEL__
#include "sii_hal.h"
#else
#endif

#include "si_common.h"
#include "si_mhl_defs.h"
#include "si_hdmi_tx_lite_api.h"
#include "si_drv_hdmi_tx_lite_hdcp.h"
#include "si_drv_hdmi_tx_lite_edid.h"
#include "si_mhl_tx_base_drv_api.h"


#ifdef ENABLE_LITE_DEBUG_PRINT //(
	#define LITE_DEBUG_PRINT(x)  TXD_DEBUG_PRINT(x) 
#else //)(
	#define LITE_DEBUG_PRINT(x)   /* nothing */
#endif //)

static bool_t TxSupportsHdcp;
static bool_t TxValidAksv;

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SiiHdmiTxLiteInitialize()
//
// PURPOSE      :   Tests Tx and Rx support of HDCP. If found, checks if
//                  and attempts to set the security level accordingly.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :	TxSupportsHdcp - initialized to false, set to true if supported by this device
//					TxValidAksv - initialized to false, set to true if valid AKSVs are read from this device
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void SiiHdmiTxLiteInitialize (bool_t enableHdcp)
{
	TxSupportsHdcp = false;
	TxValidAksv = false;

	// If the caller does not want HDCP enabled, no need to execute the rest of the function.
	if (enableHdcp == false)
	{
		// The rest of the code will execute as if TX does not support HDCP,
		// so authentication will never be attempted.
		// Video will be shown as soon as TMDS is enabled.
		return;
	}

	// This is TX-related... need only be done once.
    if (!SiiDrvHdmiTxLiteIsHdcpSupported())
    {
		// The TX does not support HDCP, so authentication will never be attempted.
		// Video will be shown as soon as TMDS is enabled.
		return;
	}

	TxSupportsHdcp = true;

	// This is TX-related... need only be done once.
    if (!SiiDrvHdmiTxLiteIsAksvValid())
    {
		// The TX supports HDCP, but does not have valid AKSVs.
		// Video will not be shown.
        return;
    }

	TxValidAksv = true;

	SiiDrvHdmiTxLiteHdcpInitialize();

	LITE_DEBUG_PRINT(("HDCP -> Supported by TX, AKSVs valid\n"));
}


void SiiHdmiTxLiteHandleEvents (uint8_t HdcpStatus,uint8_t queryData)
{

    LITE_DEBUG_PRINT(("%s %s\n",(TxSupportsHdcp == true)? "TxSupportsHdcp":"No HDCP here",(TxValidAksv == true)?"TxValidAksv":"No valid Aksv"));
	if ((TxSupportsHdcp == true) && (TxValidAksv == true))
	{
        LITE_DEBUG_PRINT(("%s %s\n",(TxSupportsHdcp == true)? "TxSupportsHdcp":"No HDCP here",(TxValidAksv == true)?"TxValidAksv":"No valid Aksv"));
		SiiDrvHdmiTxLiteHandleHdcpEvents(HdcpStatus,queryData);
	}
}

void SiiHdmiTxLiteDisableEncryption (void)
{
	SiiDrvHdmiTxLiteDisableEncryption();
}








