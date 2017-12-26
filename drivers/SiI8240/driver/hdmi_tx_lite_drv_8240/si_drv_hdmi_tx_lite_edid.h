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


#define MAX_V_DESCRIPTORS			21
#define MAX_A_DESCRIPTORS			10
#define MAX_SPEAKER_CONFIGURATIONS	 4

typedef struct
{								// for storing EDID parsed data
	uint8_t VideoDescriptor[MAX_V_DESCRIPTORS];	// maximum number of video descriptors
	uint8_t AudioDescriptor[MAX_A_DESCRIPTORS][3];	// maximum number of audio descriptors
	uint8_t SpkrAlloc[MAX_SPEAKER_CONFIGURATIONS];	// maximum number of speaker configurations
	bool_t UnderScan;								// "1" if DTV monitor underscans IT video formats by default
	bool_t BasicAudio;							// Sink supports Basic Audio
	bool_t YCbCr_4_4_4;							// Sink supports YCbCr 4:4:4
	bool_t YCbCr_4_2_2;							// Sink supports YCbCr 4:2:2
	bool_t HDMI_Sink;								// "1" if HDMI signature found
	uint8_t CEC_A_B;								// CEC Physical address. See HDMI 1.3 Table 8-6
	uint8_t CEC_C_D;
    uint8_t VideoCapabilityFlags;
	uint8_t ColorimetrySupportFlags;				// IEC 61966-2-4 colorimetry support: 1 - xvYCC601; 2 - xvYCC709
	uint8_t MetadataProfile;
	bool_t _3D_Supported;
} Type_EDID_Descriptors;

enum EDID_ErrorCodes
{
	EDID_OK,
	EDID_INCORRECT_HEADER,
	EDID_CHECKSUM_ERROR,
	EDID_NO_861_EXTENSIONS,
	EDID_SHORT_DESCRIPTORS_OK,
	EDID_LONG_DESCRIPTORS_OK,
	EDID_EXT_TAG_ERROR,
	EDID_REV_ADDR_ERROR,
	EDID_V_DESCR_OVERFLOW,
	EDID_UNKNOWN_TAG_CODE,
	EDID_NO_DETAILED_DESCRIPTORS,
	EDID_DDC_BUS_REQ_FAILURE,
	EDID_DDC_BUS_RELEASE_FAILURE,
    EDID_READ_TIMEOUT
};


