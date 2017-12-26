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


void SiiDrvHdmiTxLiteHdcpInitialize (void);

bool_t SiiDrvHdmiTxLiteIsHdcpSupported (void);
bool_t SiiDrvHdmiTxLiteIsAksvValid (void);

void SiiDrvHdmiTxLiteHandleHdcpEvents (uint8_t HdcpStatus,uint8_t queryData);

void SiiDrvHdmiTxLiteDisableEncryption (void);
