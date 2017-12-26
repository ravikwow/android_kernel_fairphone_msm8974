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
@file: si_mhl_tx_base_drv_api.h
 */
//------------------------------------------------------------------------------
// Driver API typedefs
//------------------------------------------------------------------------------
//
// structure to hold command details from upper layer to CBUS module
//
typedef struct
{
    uint8_t reqStatus;       // CBUS_IDLE, CBUS_PENDING
    uint8_t retryCount;
    uint8_t command;         // VS_CMD or RCP opcode
    uint8_t offsetData;      // Offset of register on CBUS or RCP data
    uint8_t length;          // Only applicable to write burst. ignored otherwise.
    union
    {
    uint8_t msgData[ 16 ];   // Pointer to message data area.
	unsigned char	*pdatabytes;			// pointer for write burst or read many bytes
    }payload_u;

} cbus_req_t;

//
// Functions that driver exposes to the upper layer.
//

/*
    SiiMhlTxDrvPreInitSetModeFlags
*/
void SiiMhlTxDrvPreInitSetModeFlags(uint8_t modeFlags);
typedef enum
{
    mfdrvTranscodeMode = 0x01
}SiiMhlTxDrvModeFlags_e;

//////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxChipInitialize
//
// Chip specific initialization.
// This function is for SiI 9244 Initialization: HW Reset, Interrupt enable.
//
//
//////////////////////////////////////////////////////////////////////////////
bool_t 	SiiMhlTxChipInitialize( void );

///////////////////////////////////////////////////////////////////////////////
// SiiMhlTxDeviceIsr
//
// This function must be called from a master interrupt handler or any polling
// loop in the host software if during initialization call the parameter
// interruptDriven was set to true. SiiMhlTxGetEvents will not look at these
// events assuming firmware is operating in interrupt driven mode. MhlTx component
// performs a check of all its internal status registers to see if a hardware event
// such as connection or disconnection has happened or an RCP message has been
// received from the connected device. Due to the interruptDriven being true,
// MhlTx code will ensure concurrency by asking the host software and hardware to
// disable interrupts and restore when completed. Device interrupts are cleared by
// the MhlTx component before returning back to the caller. Any handling of
// programmable interrupt controller logic if present in the host will have to
// be done by the caller after this function returns back.

// This function has no parameters and returns nothing.
//
// This is the master interrupt handler for 9244. It calls sub handlers
// of interest. Still couple of status would be required to be picked up
// in the monitoring routine (Sii9244TimerIsr)
//
// To react in least amount of time hook up this ISR to processor's
// interrupt mechanism.
//
// Just in case environment does not provide this, set a flag so we
// call this from our monitor (Sii9244TimerIsr) in periodic fashion.
//
// Device Interrupts we would look at
//		RGND		= to wake up from D3
//		MHL_EST 	= connection establishment
//		CBUS_LOCKOUT= Service USB switch
//		RSEN_LOW	= Disconnection deglitcher
//		CBUS 		= responder to peer messages
//					  Especially for DCAP etc time based events
//
void 	SiiMhlTxDeviceIsr( void );

///////////////////////////////////////////////////////////////////////////////
// SiiMhlTxDrvSendCbusCommand
//
// Write the specified Sideband Channel command to the CBUS.
// Command can be a MSC_MSG command (RCP/RAP/RCPK/RCPE/RAPK), or another command
// such as READ_DEVCAP, SET_INT, WRITE_STAT, etc.
//
// Parameters:
//              pReq    - Pointer to a cbus_req_t structure containing the
//                        command to write
// Returns:     true    - successful write
//              false   - write failed
///////////////////////////////////////////////////////////////////////////////
bool_t	SiiMhlTxDrvSendCbusCommand ( cbus_req_t *pReq  );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvCBusBusy
//
//  returns false when the CBus is ready for the next command
bool_t SiiMhlTxDrvCBusBusy(void);

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDriverTmdsControl
//
// Control the TMDS output. MhlTx uses this to support RAP content on and off.
//
void	SiiMhlTxDrvTmdsControl( bool_t enable );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvPowBitChange
//
// This function or macro is invoked from MhlTx component to
// control the VBUS power. If powerOn is sent as non-zero, one should assume
// peer does not need power so quickly remove VBUS power.
//
// if value of "powerOn" is 0, then this routine must turn the VBUS power on
// within 50ms of this call to meet MHL specs timing.
//
void	SiiMhlTxDrvPowBitChange( bool_t powerOn );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvNotifyEdidChange
//
// MhlTx may need to inform upstream device of a EDID change. This can be
// achieved by toggling the HDMI HPD signal or by simply calling EDID read
// function.
//
void	SiiMhlTxDrvNotifyEdidChange ( void );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxReadDevcap
//
// This function sends a READ DEVCAP MHL command to the peer.
// It  returns true if successful in doing so.
//
// The value of devcap should be obtained by making a call to SiiMhlTxGetEvents()
//
// offset		Which byte in devcap register is required to be read. 0..0x0E
//
bool_t SiiMhlTxReadDevcap( uint8_t offset );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxDrvGetScratchPad
//
// This function reads the local scratchpad into a local memory buffer
//
void SiiMhlTxDrvGetScratchPad(uint8_t startReg,uint8_t *pData,uint8_t length);


/*
	SiMhlTxDrvSetClkMode
	-- Set the hardware this this clock mode.
 */
void SiMhlTxDrvSetClkMode(uint8_t clkMode);


/*
    SiiMhlTxDrvGetPackedPixelStatus
        take care of transition to packed pixel mode
*/
uint8_t SiiMhlTxDrvGetPackedPixelStatus( void );

/*
    SiiMhlTxDrvSetPackedPixel
        take care of transition to packed pixel mode
*/
void	SiiMhlTxDrvSetPackedPixelStatus( int supportPackedPixel );


/*
    SiiMhlTxDrvGetDeviceId
    returns chip Id
 */
uint16_t SiiMhlTxDrvGetDeviceId(void);

/*
    SiiMhlTxDrvGetDeviceRev
    returns chip revision
 */
uint8_t SiiMhlTxDrvGetDeviceRev(void);

/*
*/
uint16_t SiiMhlTxDrvGetIncomingHorzTotal(void);

/*
*/
uint16_t SiiMhlTxDrvGetIncomingVertTotal(void);

//
// Functions that driver expects from the upper layer.
//

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxNotifyDsHpdChange
//
// Informs MhlTx component of a Downstream HPD change (when h/w receives
// SET_HPD or CLEAR_HPD).
//
extern	void	SiiMhlTxNotifyDsHpdChange( uint8_t dsHpdStatus );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxNotifyConnection
//
// This function is called by the driver to inform of connection status change.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern	void	SiiMhlTxNotifyConnection( bool_t mhlConnected );

/*
	SiiMhlTxNotifyRgndMhl
	The driver calls this when it has determined that the peer device is an MHL device.
	This routine will give the application layer the first crack at handling VBUS power
	at this point in the discovery process.
 */
extern void SiiMhlTxNotifyRgndMhl( void );

/*
    SiiMhlTxNotifyUpstreamMute
*/

void SiiMhlTxNotifyUpstreamMute( void );
/*
    SiiMhlTxNotifyUpstreamUnMute
*/
void SiiMhlTxNotifyUpstreamUnMute( void );

/*
    SiiMhlTxNotifyAuthentication
*/
void SiiMhlTxNotifyAuthentication( void );


/*
    SiiMhlTxNotifySyncDetect
*/
void SiiMhlTxNotifySyncDetect(uint8_t state);

/*
    SiiMhlTxNotifyUpStreamHDP
*/
void SiiMhlTxNotifyUpStreamHPD(uint8_t state);


/*
    SiiMhlTxDrvGetPeerDevCapRegs
*/
void SiiMhlTxDrvGetPeerDevCapRegs(PMHLDevCap_u pDevCap);

/*
    SiiMhlTxReadyForVideo
*/

uint8_t SiiMhlTxReadyForVideo(void);

/*
 SiiMhlTxDrvMscMsgNacked
    returns:
        0 - message was not NAK'ed
        non-zero message was NAK'ed
 */
extern int SiiMhlTxDrvMscMsgNacked( void );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxMscCommandDone
//
// This function is called by the driver to inform of completion of last command.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern	void	SiiMhlTxMscCommandDone( uint8_t data1 );

///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxMscWriteBurstDone
//
// This function is called by the driver to inform of completion of a write burst.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern	void	SiiMhlTxMscWriteBurstDone( uint8_t data1 );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlIntr
//
// This function is called by the driver to inform of arrival of a MHL INTERRUPT.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern void SiiMhlTxGotMhlIntr( uint8_t intr_0, uint8_t intr_1 );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlStatus
//
// This function is called by the driver to inform of arrival of a MHL STATUS.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
extern void SiiMhlTxGotMhlStatus( uint8_t status_0, uint8_t status_1 );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlMscMsg
//
// This function is called by the driver to inform of arrival of a MHL STATUS.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
// Application shall not call this function.
//
extern void SiiMhlTxGotMhlMscMsg( uint8_t subCommand, uint8_t cmdData );
///////////////////////////////////////////////////////////////////////////////
//
// SiiMhlTxGotMhlWriteBurst
//
// This function is called by the driver to inform of arrival of a scratchpad data.
//
// It is called in interrupt context to meet some MHL specified timings, therefore,
// it should not have to call app layer and do negligible processing, no printfs.
//
// Application shall not call this function.
//
extern	void	SiiMhlTxGotMhlWriteBurst( uint8_t *spadArray );

/*
	SiiMhlTxProcessAviInfoFrameChange
		optionally called by the MHL Tx driver when a
        new AVI info frame is received from upstream.
*/
extern void SiiMhlTxProcessAviInfoFrameChange(PAVIInfoFrame_t pAviInfoFrame);

/*
	SiiMhlTxProcessVendorSpecificInfoFrameChange
		optionally called by the MHL Tx driver when a
        new Vendor specific info frame is received from upstream.
*/
extern void SiiMhlTxProcessVendorSpecificInfoFrameChange(PVendorSpecificInfoFrame_t pVsInfoFrame);


uint8_t SiiMhlTxOutputModeIsHDMI( void );


/*
*/
void SiiMhlTxDrvGetAviInfoFrame(PAVIInfoFrame_t pAviInfoFrame);

/*
*/
void SiiMhlTxDrvGetVendorSpecificInfoFrame(PVendorSpecificInfoFrame_t pVendorSpecificInfoFrame);

/*
    SiiMhlTxDrv Get Input Color Space
*/
uint8_t SiiMhlTxDrvGetInputColorSpace( void );

/*
    SiiMhlTxDrv Get Output Color Space
*/
uint8_t SiiMhlTxDrvGetOutputColorSpace( void );


typedef enum
{
    qsAutoSelectByColorSpace    = 0
    ,qsFullRange                = 1
    ,qsLimitedRange             = 2
    ,qsReserved                 = 3
}QuantizationSettings_e;

#ifdef ENABLE_COLOR_SPACE_DEBUG_PRINT //(

/*
    SiiMhlTxDrv Set Input Color Space
*/
void SiiMhlTxDrvSetInputColorSpaceWrapper(char *pszId,int iLine,uint8_t inputClrSpc);
#define SiiMhlTxDrvSetInputColorSpace(inputClrSpc) SiiMhlTxDrvSetInputColorSpaceWrapper(__FILE__" SiiMhlTxDrvSetInputColorSpace",__LINE__,inputClrSpc)
/*
    SiiMhlTxDrv Set Output Color Space
*/
void SiiMhlTxDrvSetOutputColorSpaceWrapper(char *pszId,int iLine,uint8_t  outputClrSpc);
#define SiiMhlTxDrvSetOutputColorSpace(outputClrSpc) SiiMhlTxDrvSetOutputColorSpaceWrapper(__FILE__" SiiMhlTxDrvSetOutputColorSpace",__LINE__,outputClrSpc)
#else //)(


/*
    SiiMhlTxDrv Set Input Color Space
*/
void SiiMhlTxDrvSetInputColorSpaceImpl(uint8_t inputClrSpc);
#define SiiMhlTxDrvSetInputColorSpace(inputClrSpc) SiiMhlTxDrvSetInputColorSpaceImpl(inputClrSpc)

/*
    SiiMhlTxDrv Set Output Color Space
*/
void SiiMhlTxDrvSetOutputColorSpaceImpl(uint8_t  outputClrSpc);
#define SiiMhlTxDrvSetOutputColorSpace(outputClrSpc) SiiMhlTxDrvSetOutputColorSpaceImpl(outputClrSpc)

#endif //)

/*
    Set Input Quantization Range
*/
void SiiMhlTxDrvSetInputQuantizationRange(QuantizationSettings_e settings);

/*
    Set Output Quantization Range
*/
void SiiMhlTxDrvSetOutputQuantizationRange(QuantizationSettings_e settings);


/*
*/
void  SiiMhlTxDrvSetOutputType( void );
/*
*/

void SiiMhlTxDrvSet3DMode(uint8_t do3D,uint8_t three3ModeParm);

/*
*/
void SiiMhlTxDrvApplyColorSpaceSettings(void );

/*
*/

void SiiMhlTxDrvSetPixelClockFrequency(uint32_t pixelClockFrequencyParm);

/*
    SiiMhlTxDrv Set Input Video Code
*/
void SiiMhlTxDrvSetInputVideoCode (uint8_t inputVIC);

/*
    SiiMhlTxDrv Get Input Video Code
*/
uint8_t SiiMhlTxDrvGetInputVideoCode ( void );

void SiiMhlTxDrvSetColorimetryAspectRatio (AviInfoFrameDataByte2_t colAspRat );

AviInfoFrameDataByte2_t SiiMhlTxDrvGetColorimetryAspectRatio ( void );

void SiiMhlTxDrvVideoMute( void );
void	SiiMhlTxDrvVideoUnmute( void );


/*
    SiiMhlTxProcessEDID

    Read EDID from sink, correlate with MHL 3D data and filter timings
    that the link cannot support.
*/
uint8_t SiiMhlTxProcessEDID(uint8_t *pEdidBlockData);


/*
	SiiMhlTxDrvProcessRgndMhl
		optionally called by the MHL Tx Component after giving the OEM layer the
		first crack at handling the event.
*/
extern void SiiMhlTxDrvProcessRgndMhl( void );

#ifndef INTERRUPT_DRIVEN_EDID //(
typedef enum
{
    gebSuccess =0
    ,gebAcquisitionFailed
    ,gebReleaseFailed
    ,gebTimedOut
}SiiMhlTxDrvGetEdidBlockResult_e;
SiiMhlTxDrvGetEdidBlockResult_e SiiMhlTxDrvGetEdidBlock(uint8_t *pEdidBlockData,uint8_t blockNumber,uint8_t blockSize);
#else //)(
    #ifdef REQUEST_EDID_ATOMICALLY //(
        uint8_t SiiMhlTxGetNumCea861Extensions(uint8_t blockNumber);
    #endif //)
int SiiMhlTxDrvGetEdidFifoNextBlock(uint8_t *pBufEdid);
#endif //)

void SiiMhlTxDrvSetUpstreamEDID(uint8_t *pEDID,uint16_t length);

#define EDID_DUMP_SW_BUFFER
#ifdef EDID_DUMP_SW_BUFFER//(
void DumpEdidBlockImpl(char *pszFile, int iLineNum,uint8_t *pData,uint16_t length);
#define DumpEdidBlock(pData,length) DumpEdidBlockImpl(__FILE__,__LINE__,(uint8_t *)pData,length);
#else //)(
#define DumpEdidBlock(pData,length) /* nothing to do */
#endif //)

#ifdef EDID_CLEAR_HW_BUFFER //(

void ClearEdidBlockImpl(uint8_t *pData);
#define ClearEdidBlock(pData) ClearEdidBlockImpl(pData);

#else //)(

#define ClearEdidBlock(pData) /* nothing to do */

#endif //)

#ifdef	EDID_DUMP_8240_BUFFER //(

void	DumpEdidFifoImpl( int blockNumber );
#define DumpEdidFifo(blockNumber) DumpEdidFifoImpl( blockNumber );

#else //)(

#define DumpEdidFifo(blockNumber) /* do nothing */

#endif //)


