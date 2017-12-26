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
/*
 *****************************************************************************
 * @file  HDCP.c
 *
 * @brief Implementation of the HDCP Module.
 *
 *****************************************************************************
*/

#include "si_memsegsupport.h"
#include "si_common.h"
#ifdef __KERNEL__ //(
#include "sii_hal.h"
#else //)(
#include "hal_timers.h"
#endif //)

#include "si_cra.h"
#include "si_cra_cfg.h"
#include "si_drv_hdmi_tx_lite_hdcp.h"
#include "si_bitdefs.h"
#include "si_mhl_defs.h"
#include "si_mhl_tx_api.h"
#include "si_mhl_tx_base_drv_api.h"
#include "si_tpi_regs.h"



#define AKSV_SIZE              5
#define NUM_OF_ONES_IN_KSV    20


void HDCP_On (void);
void HDCP_Restart (void);

#ifdef CHECKREVOCATIONLIST //(
static bool_t CheckRevocationList (void);
#endif //)

#ifdef READKSV //(
static bool_t GetKSV (void);
#endif //)


void SiiDrvHdmiTxLiteHdcpInitialize (void)
{
	HDCP_DEBUG_PRINT(("Drv: >>SiiDrvHdmiTxLiteHdcpInitialize()\n"));
#ifdef KSVFORWARD //(
	// Enable the KSV Forwarding feature and the KSV FIFO Intererrupt
	ReadModifyWriteTPI(TPI_HDCP_CONTROL_DATA_REG, KSV_FORWARD_MASK, KSV_FORWARD_ENABLE);
	ReadModifyWriteTPI(TPI_KSV_FIFO_READY_INT_EN, KSV_FIFO_READY_EN_MASK, KSV_FIFO_READY_ENABLE);
#endif //)
}

//===========================================================
// Madhuresh
#ifdef	DEBUG_RI_VALUES
extern	int	g_ri_display;
#endif	// DEBUG_RI_VALUES


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   IsRepeater()
//
// PURPOSE      :   Test if sink is a repeater
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   TRUE if sink is a repeater. FALSE if not.
//
//////////////////////////////////////////////////////////////////////////////
int IsRepeater(uint8_t queryData)
{
    uint8_t RegImage;

	HDCP_DEBUG_PRINT(("Drv: >>IsRepeater()\n"));

    RegImage = queryData;

    if (RegImage & HDCP_REPEATER_MASK)
        return true;

    return false;           // not a repeater
}


//===========================================================

void SiiDrvHdmiTxLiteHandleHdcpEvents (uint8_t HdcpIntStatus,uint8_t queryData)
{
	uint8_t LinkStatus;
	uint8_t RegImage;
	uint8_t NewLinkProtectionLevel;
#ifdef READKSV //(
	uint8_t RiCnt;
#endif //)
#ifdef KSVFORWARD //(
	uint8_t ksv;
#endif //)
    static uint8_t renegPending=0;

    HDCP_DEBUG_PRINT(("HDCP: SiiDrvHdmiTxLiteHandleHdcpEvents HdcpIntStatus: 0x%02x\n",(int)HdcpIntStatus));

    if (!renegPending)
    {
    	// Check if Link Status has changed:
    	if (BIT_TPI_INTR_ST0_HDCP_SECURITY_CHANGE_EVENT & HdcpIntStatus )
    	{
    #ifdef ENABLE_HDCP_DEBUG_PRINT //(
        PLACE_IN_CODE_SEG char szFormatString[]="HDCP (%2X) -> Link = (%2X) %s";
    #endif //)
            HDCP_DEBUG_PRINT(("HDCP: \n"));

            LinkStatus = queryData;
    		LinkStatus &= LINK_STATUS_MASK;

    		// SiiRegWrite(TPI_INTERRUPT_STATUS_REG, HDCP_SECURITY_CHANGE_EVENT_MASK);


    		switch (LinkStatus)
    		{
    			case LINK_STATUS_NORMAL:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Normal\n"));
    				break;

    			case LINK_STATUS_LINK_LOST:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Lost\n"));

    				// AV Unmute if DVI and no HDCP in 29 and Link Lost
    				if(    (0 == SiiMhlTxOutputModeIsHDMI())
    				    && (0 == (queryData& 0x08)         )
    				    )

    				{
    					ERROR_DEBUG_PRINT(("Unmute AV coz it is DVI and no HDCP is supported\n"));
                        SiiMhlTxNotifyAuthentication();
    					// Do nothing, send out video
    					return;
    				}
    				HDCP_Restart();
                    // we've just reset the HDCP engine, wait until next interrupt to do more processing.
                    return;
    				break;

    			case LINK_STATUS_RENEGOTIATION_REQ:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Renegotiation Required\n"));
    #if 1 //(
                    // old code set the AV_MUTE bit,
                    //  then cleared 2A
                    //  then re-enabled encryption
                    SiiMhlTxDrvVideoMute();
                    SiiDrvHdmiTxLiteDisableEncryption();
    #define WAIT_FOR_BKSV_DONE_ON_RENEG
    #ifdef WAIT_FOR_BKSV_DONE_ON_RENEG //(

                    renegPending = 1;
    #else //)(
                    HDCP_On();
    #endif //)

    #else //)(
    				HDCP_Restart();
    #endif //)

                    // we've just reset the HDCP engine, wait until next interrupt to do more processing.
                    return;
    				break;

    			case LINK_STATUS_LINK_SUSPENDED:
    				HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) LinkStatus,"Suspended\n"));
    				HDCP_On();
    				break;
    		}
    	}

    	// Check if HDCP state has changed:
    	if (BIT_TPI_INTR_ST0_HDCP_AUTH_STATUS_CHANGE_EVENT & HdcpIntStatus)
    	{

            RegImage = queryData;

            HDCP_DEBUG_PRINT(("HDCP: QueryData (TPI:29) = %02X\n", (int)RegImage));
    		NewLinkProtectionLevel = RegImage & (EXTENDED_LINK_PROTECTION_MASK | LOCAL_LINK_PROTECTION_MASK);
    		{
    #ifdef ENABLE_HDCP_DEBUG_PRINT //(
            PLACE_IN_CODE_SEG char szFormatString[]="HDCP (%2X) -> Protection = (%2X) %s";
    #endif //)

                HDCP_DEBUG_PRINT(("HDCP: New prot level = %02X\n", (int)NewLinkProtectionLevel));

    			switch (NewLinkProtectionLevel)
    			{
    				case (EXTENDED_LINK_PROTECTION_NONE | LOCAL_LINK_PROTECTION_NONE):
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"None\n"));
    					HDCP_Restart();
                        return;

    				case LOCAL_LINK_PROTECTION_SECURE:
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"Local, Unmuting\n"));
                        SiiMhlTxNotifyAuthentication();
    #ifdef	DEBUG_RI_VALUES //(
    					g_ri_display = true;
    #endif	//)
    					break;

    				case (EXTENDED_LINK_PROTECTION_SECURE | LOCAL_LINK_PROTECTION_SECURE):
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"Extended\n"));
    					if (IsRepeater(queryData))
    					{
    #ifdef READKSV //(
    						RiCnt = ReadIndexedRegister(INDEXED_PAGE_0, 0x25);
    						while (RiCnt > 0x70)  // Frame 112
    						{
    							RiCnt = ReadIndexedRegister(INDEXED_PAGE_0, 0x25);
    						}
    						SiI_RegisterReadModifyWrite(TPI_SYSTEM_CONTROL_DATA_REG
    						    , DDC_BUS_REQUEST_MASK      | DDC_BUS_GRANT_MASK
    						    , DDC_BUS_REQUEST_REQUESTED | DDC_BUS_GRANT_GRANTED
    						    );
    						GetKSV();
    						RiCnt = SiiRegRead(TPI_SYSTEM_CONTROL_DATA_REG);
    #endif //)
                            SiiMhlTxNotifyAuthentication();
    					}
    					break;

    				default:
    					HDCP_DEBUG_PRINT((szFormatString, (int) HdcpIntStatus, (int) NewLinkProtectionLevel,"Extended but not Local?\n"));
    					HDCP_Restart();
                        return;
    			}
    		}

    #ifdef KSVFORWARD //(
    		// Check if KSV FIFO is ready and forward - Bug# 17892
    		// If interrupt never goes off:
    	 	//   a- KSV formwarding is not enabled
    		//   b- not a repeater
    		//   c- a repeater with device count == 0
    		// and therefore no KSV list to forward
    		if ((SiI_RegisterRead(TPI_KSV_FIFO_READY_INT) & KSV_FIFO_READY_MASK) == KSV_FIFO_READY_YES)
    		{
    			ReadModifyWriteTPI(TPI_KSV_FIFO_READY_INT, KSV_FIFO_READY_MASK, KSV_FIFO_READY_YES);
    			HDCP_DEBUG_PRINT(("KSV Fwd: KSV FIFO has data...\n"));
    			{
    				// While !(last byte has been read from KSV FIFO)
    				// if (count = 0) then a byte is not in the KSV FIFO yet, do not read
    				//   else read a byte from the KSV FIFO and forward it or keep it for revocation check
    				do
    				{
    					ksv = SiiRegRead(TPI_KSV_FIFO_STATUS_REG);
    					if (ksv & KSV_FIFO_COUNT_MASK)
    					{
    						TPI_DEBUG_PRINT((TPI_DEBUG_CHANNEL,"KSV Fwd: KSV FIFO Count = %d, ", (int)(ksv & KSV_FIFO_COUNT_MASK)));
    						ksv = SiI_RegisterRead(TPI_KSV_FIFO_VALUE_REG);	// Forward or store for revocation check
    						HDCP_DEBUG_PRINT(("Value = %d\n", (int)ksv));
    					}
    				} while ((ksv & KSV_FIFO_LAST_MASK) == KSV_FIFO_LAST_NO);
    				HDCP_DEBUG_PRINT(("KSV Fwd: Last KSV FIFO forward complete\n"));
    			}
    		}
    #endif //)
    		// SiiRegWrite(TPI_INTERRUPT_STATUS_REG, HDCP_AUTH_STATUS_CHANGE_EVENT_MASK);
    	}
    }
    if ( BIT_TPI_INTR_ST0_BKSV_DONE & HdcpIntStatus)
    {
        HDCP_DEBUG_PRINT(("HDCP: BKSV read done\n"));

        HDCP_DEBUG_PRINT(("HDCP: QueryData: 0x%02x 2A==%02x\n"
            ,(int)queryData
            ,SiiRegRead(TPI_HDCP_CONTROL_DATA_REG)
            ));

        if (queryData & PROTECTION_TYPE_MASK)   // Is HDCP available
        {
            HDCP_DEBUG_PRINT(("HDCP: \n"));
            renegPending = 0;
        	HDCP_On();
        }
    }
    if ( BIT_TPI_INTR_ST0_BKSV_ERR & HdcpIntStatus)
    {
        HDCP_DEBUG_PRINT(("HDCP: BKSV error - be sure that your sink supports HDCP - restarting\n"));
		//HDCP_Restart();
        return;
    }

}

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SiiDrvHdmiTxLiteIsHdcpSupported()
//
// PURPOSE      :   Check TX HDCP revision number to determine if HDCP is
//                  supported by reading TPI_HDCP_REVISION_DATA_REG.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   True if TX supports HDCP, false otherwise.
//
//////////////////////////////////////////////////////////////////////////////

bool_t SiiDrvHdmiTxLiteIsHdcpSupported (void)
{
    uint8_t revision;
	bool_t supported;

	supported = true;

	// Check Device ID
    revision = SiiRegRead(TPI_HDCP_REVISION_DATA_REG);

    if (revision != (HDCP_MAJOR_REVISION_VALUE | HDCP_MINOR_REVISION_VALUE))
	{
    	supported = false;
		//HDCP_DEBUG_PRINT(("%sHdmiTxDrv: HDCP not supported. revision:0x%02x\n%s",PlatformGPIOGet(pinDoHdcp)?"\n\n\n":"",(uint16_t)revision,PlatformGPIOGet(pinDoHdcp)?"\n\n\n":"" ));
		HDCP_DEBUG_PRINT(("HdmiTxDrv: HDCP not supported. revision:0x%02x\n%s\n\n\n",(uint16_t)revision));
	}

	return supported;
}



//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   HDCP_On()
//
// PURPOSE      :   Switch hdcp on.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void HDCP_On (void)
{
   	HDCP_DEBUG_PRINT(("HDCP -> Started. TPI:2A = 05\n"));

#if 1 //(
   	SiiRegModify(TPI_HDCP_CONTROL_DATA_REG
   	    , BIT_TPI_HDCP_CONTROL_DATA_DOUBLE_RI_CHECK_MASK   | BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MASK
   	    , BIT_TPI_HDCP_CONTROL_DATA_DOUBLE_RI_CHECK_ENABLE | BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MAX
           );
#else //)(
   	SiiRegModify(TPI_HDCP_CONTROL_DATA_REG
   	    , BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MASK
   	    , BIT_TPI_HDCP_CONTROL_DATA_COPP_PROTLEVEL_MAX
           );
#endif //)

}

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   SiiDrvHdmiTxLiteDisableEncryption()
//
// PURPOSE      :   Switch hdcp off.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void SiiDrvHdmiTxLiteDisableEncryption (void)
{
    HDCP_DEBUG_PRINT(("HDCP -> Stopped. 2A = 0\n"));
    SiiRegWrite(TPI_HDCP_CONTROL_DATA_REG, 0);
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   HDCP_Restart()
//
// PURPOSE      :   Switch hdcp off and then back on.
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   None
//
// RETURNS      :   None
//
//////////////////////////////////////////////////////////////////////////////

void HDCP_Restart (void)
{
	HDCP_DEBUG_PRINT(("HDCP -> Restart --- TOGGLE TMDS\n"));

	SiiDrvHdmiTxLiteDisableEncryption();
	SiiToggleTmdsForHdcpAuthentication();
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION     :   GetKSV()
//
// PURPOSE      :   Collect all downstrean KSV for verification
//
// INPUT PARAMS :   None
//
// OUTPUT PARAMS:   None
//
// GLOBALS USED :   KSV_Array[]
//
// RETURNS      :   true if KSVs collected successfully. False if not.
//
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// The buffer is limited to KSV_ARRAY_SIZE due to the 8051 implementation.
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//////////////////////////////////////////////////////////////////////////////
#ifdef READKSV //(
static bool_t GetKSV(void)
{
	uint8_t i;
    uint16_t KeyCount;
    uint8_t KSV_Array[KSV_ARRAY_SIZE];

    ReadBlockHDCP(DDC_BSTATUS_ADDR_L, 1, &i);
    KeyCount = (i & DEVICE_COUNT_MASK) * 5;
	if (KeyCount != 0)
	{
		ReadBlockHDCP(DDC_KSV_FIFO_ADDR, KeyCount, KSV_Array);
	}

	/*
	TPI_TRACE_PRINT((TPI_TRACE_CHANNEL,"KeyCount = %d\n", (int) KeyCount));
	for (i=0; i<KeyCount; i++)
	{
		TPI_TRACE_PRINT((TPI_TRACE_CHANNEL,"KSV[%2d] = %02X\n", (int) i, (int) KSV_Array[i]));
	}
	*/

	 return true;
}
#endif //)

//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  SiiDrvHdmiTxLiteIsAksvValid()
//
// PURPOSE       :  Check if AKSVs contain 20 '0' and 20 '1'
//
// INPUT PARAMS  :  None
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  TBD
//
// RETURNS       :  true if 20 zeros and 20 ones found in AKSV. false OTHERWISE
//
//////////////////////////////////////////////////////////////////////////////
bool_t SiiDrvHdmiTxLiteIsAksvValid (void)
{
    uint8_t B_Data[AKSV_SIZE];
    uint8_t NumOfOnes = 0;
    uint8_t i;

    SiiRegReadBlock(TPI_AKSV_1_REG, B_Data, AKSV_SIZE);

    for (i=0; i < AKSV_SIZE; i++)
    {
        while (B_Data[i] != 0x00)
        {
            if (B_Data[i] & 0x01)
            {
                NumOfOnes++;
            }
            B_Data[i] >>= 1;
        }
    }

    if (NumOfOnes != NUM_OF_ONES_IN_KSV)
	{
        HDCP_DEBUG_PRINT(("HDCP -> Illegal AKSV\n"));
        return false;
	}

    return true;
}


//////////////////////////////////////////////////////////////////////////////
//
// FUNCTION      :  CheckRevocationList()
//
// PURPOSE       :  Compare KSVs to those included in a revocation list
//
// INPUT PARAMS  :  None
//
// OUTPUT PARAMS :  None
//
// GLOBALS USED  :  TBD
//
// RETURNS       :  true if no illegal KSV found in BKSV list
//
// NOTE			 :	Currently this function is implemented as a place holder only
//
//////////////////////////////////////////////////////////////////////////////
#ifdef CHECKREVOCATIONLIST //(
static bool CheckRevocationList(void)
{
    return true;
}
#endif //)


