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
   @file si_osdebug.h
 */

#ifndef __SI_OSDEBUG_H__
#define __SI_OSDEBUG_H__

typedef enum
{
	 SII_OS_DEBUG_FORMAT_SIMPLE   	= 0x0000u
	,SII_OS_DEBUG_FORMAT_FILEINFO 	= 0x0001u
	,SII_OS_DEBUG_FORMAT_CHANNEL  	= 0x0002u
	,SII_OS_DEBUG_FORMAT_TIMESTAMP	= 0x0004u
}SiiOsDebugFormat_e;


#define MODULE_SET(name) \
    SII_OSAL_DEBUG_##name, \
    SII_OSAL_DEBUG_##name##_DBG =SII_OSAL_DEBUG_##name, \
    SII_OSAL_DEBUG_##name##_ERR, \
    SII_OSAL_DEBUG_##name##_TRACE, \
    SII_OSAL_DEBUG_##name##_ALWAYS, \
    SII_OSAL_DEBUG_##name##_USER1, \
    SII_OSAL_DEBUG_##name##_USER2, \
    SII_OSAL_DEBUG_##name##_USER3, \
    SII_OSAL_DEBUG_##name##_USER4, \
    SII_OSAL_DEBUG_##name##_MASK = SII_OSAL_DEBUG_##name##_USER4,

//The list above must produce values in groups of powers of two (2, 4, 8, 16,...)

typedef enum
{
	MODULE_SET(APP)
	MODULE_SET(TRACE)
    MODULE_SET(POWER_MAN)
    MODULE_SET(TX)
    MODULE_SET(EDID)
    MODULE_SET(HDCP)
    MODULE_SET(AV_CONFIG)
    MODULE_SET(ENTRY_EXIT)
    MODULE_SET(CBUS)
    MODULE_SET(SCRATCHPAD)
    MODULE_SET(SCHEDULER)
    MODULE_SET(CRA)
    MODULE_SET(MIPI)
    MODULE_SET(HDMI)

    // this one MUST be last in the list
    SII_OSAL_DEBUG_NUM_CHANNELS
}SiiOsalDebugChannels_e;

#ifndef SII_DEBUG_CONFIG_RESOURCE_CONSTRAINED //(
typedef void SiiOsDebugChannel_t;
uint32_t SiiOsDebugChannelAdd(uint32_t numChannels, SiiOsDebugChannel_t *paChannelList);
#endif //)

void SiiOsDebugChannelEnable(SiiOsalDebugChannels_e channel);
#define SI_OS_ENABLE_DEBUG_CHANNEL(channel) SiiOsDebugChannelEnable(channel)

void SiiOsDebugChannelDisable(SiiOsalDebugChannels_e channel);
#define SI_OS_DISABLE_DEBUG_CHANNEL(channel) SiiOsDebugChannelDisable(channel)

bool_t SiiOsDebugChannelIsEnabled(SiiOsalDebugChannels_e channel);
void SiiOsDebugSetConfig(uint16_t flags);
#define SiiOsDebugConfig(flags) SiiOsDebugSetConfig(flags)
uint16_t SiiOsDebugGetConfig(void);

void SiiOsDebugPrintAlways(char *pszFormat,...);
void SiiOsDebugPrintAlwaysShort(char *pszFormat,...);
#ifndef C99_VA_ARG_SUPPORT //(
extern unsigned int g_debugLineNo;
extern char *g_debugFileName;
extern SiiOsalDebugChannels_e g_channelArg;
void SiiOsDebugPrintUseGlobal(
#ifndef __KERNEL__
const
#endif
char *pszFormat, ...);
#endif //)

void SiiOsDebugPrintSimple(SiiOsalDebugChannels_e channel, char *pszFormat,...);
void SiiOsDebugPrintSimpleAlways(char *pszFormat,...);
void SiiOsDebugPrintShort(SiiOsalDebugChannels_e channel, char *pszFormat,...);
void SiiOsDebugPrint(const char *pFileName, uint32_t iLineNum, SiiOsalDebugChannels_e channel, const char *pszFormat, ...);
void SiiOsDebugPrintSimpleUseGlobal(char *pszFormat,...);

#ifdef ENABLE_ERROR_DEBUG_PRINT
#ifdef C99_VA_ARG_SUPPORT //(
    #define ERROR_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define ERROR_DEBUG_PRINT(x)  ERROR_DEBUG_PRINT_WRAPPER x
#else //)(
    #define ERROR_DEBUG_PRINT(x) g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x
#endif //)
#else
#define ERROR_DEBUG_PRINT(x) 
#endif

#ifdef C99_VA_ARG_SUPPORT //(
#ifdef SII_DEBUG_CONFIG_NO_FILE_LINE //(
#define SII_DEBUG_PRINT(channel,...) SiiOsDebugPrintShort(channel,__VA_ARGS__)
#else //)(
#define SII_DEBUG_PRINT(channel,...) SiiOsDebugPrint(__FILE__,__LINE__,channel,__VA_ARGS__)
#endif //)

#define SII_PRINT_FULL(channel,...) SiiOsDebugPrint(__FILE__,__LINE__,channel,__VA_ARGS__)
#define SII_PRINT(channel,...) SiiOsDebugPrintShort(channel,__VA_ARGS__)
#define SII_PRINT_PLAIN(channel,...) SiiOsDebugPrintSimple(channel,__VA_ARGS__)

#else //)(

#define SII_PRINT_FULL(channel,x) SiiOsDebugPrintUseGlobal x
#define SII_PRINT(channel,x) SiiOsDebugPrintShortUseGlobal x
#define SII_PRINT_PLAIN(channel,x) SiiOsDebugPrintSimpleUseGlobal x

#ifdef SII_DEBUG_CONFIG_NO_FILE_LINE //(
#define SII_DEBUG_PRINT(channel,x) {  g_channelArg = channel; SiiOsDebugPrintShortUseGlobal x ; }
#else //)(
#define SII_DEBUG_PRINT(channel,x) { g_debugLineNo = __LINE__; g_debugFileName = __FILE__; g_channelArg = channel; SiiOsDebugPrintUseGlobal x; }
#endif //)

#endif //)
#define SII_DEBUG(channel,x) if (SiiOsDebugChannelIsEnabled(channel) {x}


#define SII_ASSERT(x,y)  if (!(x)) {ERROR_DEBUG_PRINT(y);}

#ifdef ENABLE_EDID_DEBUG_PRINT //(

#ifdef C99_VA_ARG_SUPPORT //(

#define EDID_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define EDID_DEBUG_PRINT(x)  EDID_DEBUG_PRINT_WRAPPER x
    #define EDID_DEBUG_PRINT_SIMPLE_WRAPPER(...) SiiOsDebugPrintSimple(SII_OSAL_DEBUG_TX,__VA_ARGS__)
	#define EDID_DEBUG_PRINT_SIMPLE(...) EDID_DEBUG_PRINT_SIMPLE_WRAPPER x
#else //)(
    #define EDID_DEBUG_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; g_channelArg=SII_OSAL_DEBUG_TX;SiiOsDebugPrintUseGlobal x
    #define EDID_DEBUG_PRINT_SIMPLE(x) g_channelArg=SII_OSAL_DEBUG_TX;SiiOsDebugPrintSimpleUseGlobal x
#endif //)

#else //)(

#ifdef C99_VA_ARG_SUPPORT //(
    #define EDID_DEBUG_PRINT(...)	/* nothing */
    #define EDID_DEBUG_PRINT_SIMPLE(...) /* nothing */
#else //)(
    #define EDID_DEBUG_PRINT(x)   /* nothing */
    #define EDID_DEBUG_PRINT_SIMPLE(x) /* nothing */
#endif //)

#endif //)

#ifdef ENABLE_EDID_TRACE_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define EDID_TRACE_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define EDID_TRACE_PRINT(x)  EDID_TRACE_PRINT_WRAPPER x
#else //)(

#define EDID_TRACE_PRINT(x) SII_DEBUG_PRINT(SII_OSAL_DEBUG_EDID_TRACE,x)
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define EDID_TRACE_PRINT(...)	/* nothing */

#else //)(

#define EDID_TRACE_PRINT(x) /* nothing */

#endif //)
#endif //)

#ifdef ENABLE_TX_EDID_PRINT //(

#ifdef C99_VA_ARG_SUPPORT //(

#define TX_EDID_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define TX_EDID_PRINT(x)  TX_EDID_PRINT_WRAPPER x
    #define TX_EDID_PRINT_SIMPLE_WRAPPER(...) SiiOsDebugPrintSimple(SII_OSAL_DEBUG_TX,__VA_ARGS__)
	#define TX_EDID_PRINT_SIMPLE(x)  TX_EDID_PRINT_SIMPLE_WRAPPER x
#else //)(
    #define TX_EDID_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; g_channelArg=SII_OSAL_DEBUG_TX;SiiOsDebugPrintUseGlobal x
    #define TX_EDID_PRINT_SIMPLE(x) g_channelArg=SII_OSAL_DEBUG_TX;SiiOsDebugPrintSimpleUseGlobal x
#endif //)

#else //)(

#ifdef C99_VA_ARG_SUPPORT //(
    #define TX_EDID_PRINT(...)	//SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define TX_EDID_PRINT_SIMPLE(...) /* nothing */
#else //)(
    #define TX_EDID_PRINT(x)   /* nothing */
    #define TX_EDID_PRINT_SIMPLE(x) /* nothing */
#endif //)

#endif //)

#ifdef ENABLE_HDCP_DEBUG_PRINT //(

#ifdef C99_VA_ARG_SUPPORT //(
#define HDCP_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrint(__FILE__,__LINE__,SII_OSAL_DEBUG_HDCP_DBG,__VA_ARGS__)
    #define HDCP_DEBUG_PRINT(x)  HDCP_DEBUG_PRINT_WRAPPER x
#else //)(
    #define HDCP_DEBUG_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define HDCP_DEBUG_PRINT(...)	/* nothing */

#else //)(

#define HDCP_DEBUG_PRINT(x) /* nothing */

#endif //)
#endif //)

#ifdef ENABLE_HDCP_TRACE_PRINT //(

#ifdef C99_VA_ARG_SUPPORT //(
#define HDCP_TRACE_PRINT_WRAPPER(...) SiiOsDebugPrint(__FILE__,__LINE__,SII_OSAL_DEBUG_HDCP_TRACE,__VA_ARGS__)
    #define HDCP_TRACE_PRINT(x)  HDCP_TRACE_PRINT_WRAPPER x
#else //)(
    #define HDCP_TRACE_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define HDCP_TRACE_PRINT(...)	/* nothing */

#else //)(

#define HDCP_TRACE_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x

#endif //)
#endif //)

#ifdef ENABLE_INFO_DEBUG_PRINT //(

#ifdef C99_VA_ARG_SUPPORT //(
#define INFO_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrint(__FILE__,__LINE__,SII_OSAL_DEBUG_HDMI_DBG,__VA_ARGS__)
    #define INFO_DEBUG_PRINT(x)  INFO_DEBUG_PRINT_WRAPPER x
#else //)(

#define INFO_DEBUG_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define INFO_DEBUG_PRINT(...)	/* nothing */

#else //)(

#define INFO_DEBUG_PRINT(x) /* nothing */

#endif //)
#endif //)


#ifdef ENABLE_TXC_DEBUG_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define TXC_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define TXC_DEBUG_PRINT(x)  TXC_DEBUG_PRINT_WRAPPER x
#else //)(
    #define TXC_DEBUG_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define TXC_DEBUG_PRINT(...)	/* nothing */
#else //)(
    #define TXC_DEBUG_PRINT(x)   /* nothing */
#endif //)
#endif //)

#ifdef ENABLE_TXD_DEBUG_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define TXD_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define TXD_DEBUG_PRINT(x)  TXD_DEBUG_PRINT_WRAPPER x
#else //)(
    #define TXD_DEBUG_PRINT(x)  g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define TXD_DEBUG_PRINT(...)	/* nothing */
#else //)(
    #define TXD_DEBUG_PRINT(x)   /* nothing */
#endif //)
#endif //)

#ifdef ENABLE_PIXCLK_DEBUG_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define PIXCLK_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define PIXCLK_DEBUG_PRINT(x)  PIXCLK_DEBUG_PRINT_WRAPPER x
#else //)(
    #define PIXCLK_DEBUG_PRINT(x)   TX_EDID_PRINT(x)
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define PIXCLK_DEBUG_PRINT(...)	/* nothing */
#else //)(
    #define PIXCLK_DEBUG_PRINT(x)   /* nothing */
#endif //)
#endif //)

#ifdef ENABLE_PP_DEBUG_PRINT //(
#define PP_DEBUG_PRINT(x)  TXC_DEBUG_PRINT(x)
#else //)(
#define PP_DEBUG_PRINT(x)
#endif //)

#ifdef ENABLE_TX_PRUNE_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define TX_PRUNE_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define TX_PRUNE_PRINT(x)  TX_PRUNE_PRINT_WRAPPER x
#else //)(
    #define TX_PRUNE_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; g_channelArg=SII_OSAL_DEBUG_TX;SiiOsDebugPrintUseGlobal x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define TX_PRUNE_PRINT(...)	/* nothing */
#else //)(
    #define TX_PRUNE_PRINT(x)   /* nothing */
#endif //)
#endif //)

#ifdef ENABLE_CBUS_DEBUG_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define CBUS_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrintAlways(/*__FILE__,__LINE__,SII_OSAL_DEBUG_TX,*/__VA_ARGS__)
    #define CBUS_DEBUG_PRINT(x)  CBUS_DEBUG_PRINT_WRAPPER x
#else //)(
    #define CBUS_DEBUG_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; g_channelArg=SII_OSAL_DEBUG_CBUS;SiiOsDebugPrintUseGlobal x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define CBUS_DEBUG_PRINT(...)	/* nothing */
#else //)(
    #define CBUS_DEBUG_PRINT(x)   /* nothing */
#endif //)
#endif //)

#ifdef ENABLE_SCRPAD_DEBUG_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define SCRPAD_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrint(__FILE__,__LINE__,SII_OSAL_DEBUG_CBUS,__VA_ARGS__)
    #define SCRPAD_DEBUG_PRINT(x)  SCRPAD_DEBUG_PRINT_WRAPPER x
#else //)(
    #define SCRPAD_DEBUG_PRINT(x)   g_debugLineNo = __LINE__; g_debugFileName = __FILE__; g_channelArg=SII_OSAL_DEBUG_CBUS;SiiOsDebugPrintUseGlobal x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define SCRPAD_DEBUG_PRINT(...)	/* nothing */
#else //)(
    #define SCRPAD_DEBUG_PRINT(x)   /* nothing */
#endif //)
#endif //)

#ifdef ENABLE_COLOR_SPACE_DEBUG_PRINT //(
#ifdef C99_VA_ARG_SUPPORT //(
#define COLOR_SPACE_DEBUG_PRINT_WRAPPER(...) SiiOsDebugPrint(__FILE__,__LINE__,SII_OSAL_DEBUG_TX,__VA_ARGS__)
    #define COLOR_SPACE_DEBUG_PRINT(x)  COLOR_SPACE_DEBUG_PRINT_WRAPPER x
#else //)(
    #define COLOR_SPACE_DEBUG_PRINT(x)  g_debugLineNo = __LINE__; g_debugFileName = __FILE__; g_channelArg=SII_OSAL_DEBUG_CBUS;SiiOsDebugPrintUseGlobal x
#endif //)
#else //)(
#ifdef C99_VA_ARG_SUPPORT //(
    #define COLOR_SPACE_DEBUG_PRINT(...)	/* nothing */
#else //)(
    #define COLOR_SPACE_DEBUG_PRINT(x)   /* nothing */
#endif //)
#endif //)

#ifdef ENABLE_PLATFORM_DEBUG_PRINT //(

#ifdef C99_VA_ARG_SUPPORT //(
#define SII_PLATFORM_DEBUG_PRINT(...) SII_DEBUG_PRINT(SII_OSAL_DEBUG_PLATFORM,__VA_ARGS__)
#else //)(
#define SII_PLATFORM_DEBUG_PRINT(x) g_debugLineNo = __LINE__; g_debugFileName = __FILE__; SiiOsDebugPrintAlways x
#endif //)

#else //)(

#ifdef C99_VA_ARG_SUPPORT //(
#define SII_PLATFORM_DEBUG_PRINT(...) /*nothing*/
#else //)(
#define SII_PLATFORM_DEBUG_PRINT(x) /*nothing*/
#endif //)

#endif //)

#endif // #ifndef __SI_OSDEBUG_H__



