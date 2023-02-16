/***************************************************************************//**
 *   @file   nhd_c12832a1z.c
 *   @brief  Implementation of nhd_c12832a1z Driver.
 *   @author Antoniu Miclaus (antoniu.miclaus@analog.com)
********************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdbool.h>
#include "nhd_c12832a1z.h"
#include "no_os_error.h"
#include "no_os_spi.h"
#include "no_os_delay.h"
#include <string.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/

static const uint8_t ASC16[256][8] = {
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x3E, 0x5B, 0x4F, 0x5B, 0x3E, 0x00, 0x00},
	{0x00, 0x3E, 0x6B, 0x4F, 0x6B, 0x3E, 0x00, 0x00},
	{0x00, 0x1C, 0x3E, 0x7C, 0x3E, 0x1C, 0x00, 0x00},
	{0x00, 0x18, 0x3C, 0x7E, 0x3C, 0x18, 0x00, 0x00},
	{0x00, 0x1C, 0x57, 0x7D, 0x57, 0x1C, 0x00, 0x00},
	{0x00, 0x1C, 0x5E, 0x7F, 0x5E, 0x1C, 0x00, 0x00},
	{0x00, 0x00, 0x18, 0x3C, 0x18, 0x00, 0x00, 0x00},
	{0x00, 0xFF, 0xE7, 0xC3, 0xE7, 0xFF, 0x00, 0x00},
	{0x00, 0x00, 0x18, 0x24, 0x18, 0x00, 0x00, 0x00},
	{0x00, 0xFF, 0xE7, 0xDB, 0xE7, 0xFF, 0x00, 0x00},
	{0x00, 0x30, 0x48, 0x3A, 0x06, 0x0E, 0x00, 0x00},
	{0x00, 0x26, 0x29, 0x79, 0x29, 0x26, 0x00, 0x00},
	{0x00, 0x40, 0x7F, 0x05, 0x05, 0x07, 0x00, 0x00},
	{0x00, 0x40, 0x7F, 0x05, 0x25, 0x3F, 0x00, 0x00},
	{0x00, 0x5A, 0x3C, 0xE7, 0x3C, 0x5A, 0x00, 0x00},
	{0x00, 0x7F, 0x3E, 0x1C, 0x1C, 0x08, 0x00, 0x00},
	{0x00, 0x08, 0x1C, 0x1C, 0x3E, 0x7F, 0x00, 0x00},
	{0x00, 0x14, 0x22, 0x7F, 0x22, 0x14, 0x00, 0x00},
	{0x00, 0x5F, 0x5F, 0x00, 0x5F, 0x5F, 0x00, 0x00},
	{0x00, 0x06, 0x09, 0x7F, 0x01, 0x7F, 0x00, 0x00},
	{0x00, 0x00, 0x66, 0x89, 0x95, 0x6A, 0x00, 0x00},
	{0x00, 0x60, 0x60, 0x60, 0x60, 0x60, 0x00, 0x00},
	{0x00, 0x94, 0xA2, 0xFF, 0xA2, 0x94, 0x00, 0x00},
	{0x00, 0x08, 0x04, 0x7E, 0x04, 0x08, 0x00, 0x00},
	{0x00, 0x10, 0x20, 0x7E, 0x20, 0x10, 0x00, 0x00},
	{0x00, 0x08, 0x08, 0x2A, 0x1C, 0x08, 0x00, 0x00},
	{0x00, 0x08, 0x1C, 0x2A, 0x08, 0x08, 0x00, 0x00},
	{0x00, 0x1E, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00},
	{0x00, 0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 0x00, 0x00},
	{0x00, 0x30, 0x38, 0x3E, 0x38, 0x30, 0x00, 0x00},
	{0x00, 0x06, 0x0E, 0x3E, 0x0E, 0x06, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x5F, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00},
	{0x00, 0x14, 0x7F, 0x14, 0x7F, 0x14, 0x00, 0x00},
	{0x00, 0x24, 0x2A, 0x7F, 0x2A, 0x12, 0x00, 0x00},
	{0x00, 0x23, 0x13, 0x08, 0x64, 0x62, 0x00, 0x00},
	{0x00, 0x36, 0x49, 0x56, 0x20, 0x50, 0x00, 0x00},
	{0x00, 0x00, 0x08, 0x07, 0x03, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x1C, 0x22, 0x41, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x41, 0x22, 0x1C, 0x00, 0x00, 0x00},
	{0x00, 0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 0x00, 0x00},
	{0x00, 0x08, 0x08, 0x3E, 0x08, 0x08, 0x00, 0x00},
	{0x00, 0x00, 0x80, 0x70, 0x30, 0x00, 0x00, 0x00},
	{0x00, 0x08, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x10, 0x08, 0x04, 0x02, 0x00, 0x00},
	{0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00},
	{0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00},
	{0x00, 0x72, 0x49, 0x49, 0x49, 0x46, 0x00, 0x00},
	{0x00, 0x21, 0x41, 0x49, 0x4D, 0x33, 0x00, 0x00},
	{0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00},
	{0x00, 0x27, 0x45, 0x45, 0x45, 0x39, 0x00, 0x00},
	{0x00, 0x3C, 0x4A, 0x49, 0x49, 0x31, 0x00, 0x00},
	{0x00, 0x41, 0x21, 0x11, 0x09, 0x07, 0x00, 0x00},
	{0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00},
	{0x00, 0x46, 0x49, 0x49, 0x29, 0x1E, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x40, 0x34, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00},
	{0x00, 0x00, 0x41, 0x22, 0x14, 0x08, 0x00, 0x00},
	{0x00, 0x02, 0x01, 0x59, 0x09, 0x06, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x5D, 0x59, 0x4E, 0x00, 0x00},
	{0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C, 0x00, 0x00},
	{0x00, 0x7F, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x41, 0x41, 0x22, 0x00, 0x00},
	{0x00, 0x7F, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00},
	{0x00, 0x7F, 0x49, 0x49, 0x49, 0x41, 0x00, 0x00},
	{0x00, 0x7F, 0x09, 0x09, 0x09, 0x01, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x41, 0x51, 0x73, 0x00, 0x00},
	{0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F, 0x00, 0x00},
	{0x00, 0x00, 0x41, 0x7F, 0x41, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x40, 0x41, 0x3F, 0x01, 0x00, 0x00},
	{0x00, 0x7F, 0x08, 0x14, 0x22, 0x41, 0x00, 0x00},
	{0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00},
	{0x00, 0x7F, 0x02, 0x1C, 0x02, 0x7F, 0x00, 0x00},
	{0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E, 0x00, 0x00},
	{0x00, 0x7F, 0x09, 0x09, 0x09, 0x06, 0x00, 0x00},
	{0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E, 0x00, 0x00},
	{0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00},
	{0x00, 0x26, 0x49, 0x49, 0x49, 0x32, 0x00, 0x00},
	{0x00, 0x03, 0x01, 0x7F, 0x01, 0x03, 0x00, 0x00},
	{0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F, 0x00, 0x00},
	{0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F, 0x00, 0x00},
	{0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F, 0x00, 0x00},
	{0x00, 0x63, 0x14, 0x08, 0x14, 0x63, 0x00, 0x00},
	{0x00, 0x03, 0x04, 0x78, 0x04, 0x03, 0x00, 0x00},
	{0x00, 0x61, 0x59, 0x49, 0x4D, 0x43, 0x00, 0x00},
	{0x00, 0x00, 0x7F, 0x41, 0x41, 0x41, 0x00, 0x00},
	{0x00, 0x02, 0x04, 0x08, 0x10, 0x20, 0x00, 0x00},
	{0x00, 0x00, 0x41, 0x41, 0x41, 0x7F, 0x00, 0x00},
	{0x00, 0x04, 0x02, 0x01, 0x02, 0x04, 0x00, 0x00},
	{0x00, 0x40, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00},
	{0x00, 0x00, 0x03, 0x07, 0x08, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x54, 0x54, 0x78, 0x40, 0x00, 0x00},
	{0x00, 0x7F, 0x28, 0x44, 0x44, 0x38, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x44, 0x28, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x28, 0x7F, 0x00, 0x00},
	{0x00, 0x38, 0x54, 0x54, 0x54, 0x18, 0x00, 0x00},
	{0x00, 0x00, 0x08, 0x7E, 0x09, 0x02, 0x00, 0x00},
	{0x00, 0x18, 0xA4, 0xA4, 0x9C, 0x78, 0x00, 0x00},
	{0x00, 0x7F, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00},
	{0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x40, 0x40, 0x3D, 0x00, 0x00, 0x00},
	{0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x41, 0x7F, 0x40, 0x00, 0x00, 0x00},
	{0x00, 0x7C, 0x04, 0x78, 0x04, 0x78, 0x00, 0x00},
	{0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00},
	{0x00, 0xFC, 0x18, 0x24, 0x24, 0x18, 0x00, 0x00},
	{0x00, 0x18, 0x24, 0x24, 0x18, 0xFC, 0x00, 0x00},
	{0x00, 0x7C, 0x08, 0x04, 0x04, 0x08, 0x00, 0x00},
	{0x00, 0x48, 0x54, 0x54, 0x54, 0x24, 0x00, 0x00},
	{0x00, 0x04, 0x04, 0x3F, 0x44, 0x24, 0x00, 0x00},
	{0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C, 0x00, 0x00},
	{0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C, 0x00, 0x00},
	{0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C, 0x00, 0x00},
	{0x00, 0x44, 0x28, 0x10, 0x28, 0x44, 0x00, 0x00},
	{0x00, 0x4C, 0x90, 0x90, 0x90, 0x7C, 0x00, 0x00},
	{0x00, 0x44, 0x64, 0x54, 0x4C, 0x44, 0x00, 0x00},
	{0x00, 0x00, 0x08, 0x36, 0x41, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x41, 0x36, 0x08, 0x00, 0x00, 0x00},
	{0x00, 0x02, 0x01, 0x02, 0x04, 0x02, 0x00, 0x00},
	{0x00, 0x3C, 0x26, 0x23, 0x26, 0x3C, 0x00, 0x00},
	{0x00, 0x1E, 0xA1, 0xA1, 0x61, 0x12, 0x00, 0x00},
	{0x00, 0x3A, 0x40, 0x40, 0x20, 0x7A, 0x00, 0x00},
	{0x00, 0x38, 0x54, 0x54, 0x55, 0x59, 0x00, 0x00},
	{0x00, 0x21, 0x55, 0x55, 0x79, 0x41, 0x00, 0x00},
	{0x00, 0x22, 0x54, 0x54, 0x78, 0x42, 0x00, 0x00},
	{0x00, 0x21, 0x55, 0x54, 0x78, 0x40, 0x00, 0x00},
	{0x00, 0x20, 0x54, 0x55, 0x79, 0x40, 0x00, 0x00},
	{0x00, 0x0C, 0x1E, 0x52, 0x72, 0x12, 0x00, 0x00},
	{0x00, 0x39, 0x55, 0x55, 0x55, 0x59, 0x00, 0x00},
	{0x00, 0x39, 0x54, 0x54, 0x54, 0x59, 0x00, 0x00},
	{0x00, 0x39, 0x55, 0x54, 0x54, 0x58, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x45, 0x7C, 0x41, 0x00, 0x00},
	{0x00, 0x00, 0x02, 0x45, 0x7D, 0x42, 0x00, 0x00},
	{0x00, 0x00, 0x01, 0x45, 0x7C, 0x40, 0x00, 0x00},
	{0x00, 0x7D, 0x12, 0x11, 0x12, 0x7D, 0x00, 0x00},
	{0x00, 0xF0, 0x28, 0x25, 0x28, 0xF0, 0x00, 0x00},
	{0x00, 0x7C, 0x54, 0x55, 0x45, 0x00, 0x00, 0x00},
	{0x00, 0x20, 0x54, 0x54, 0x7C, 0x54, 0x00, 0x00},
	{0x00, 0x7C, 0x0A, 0x09, 0x7F, 0x49, 0x00, 0x00},
	{0x00, 0x32, 0x49, 0x49, 0x49, 0x32, 0x00, 0x00},
	{0x00, 0x3A, 0x44, 0x44, 0x44, 0x3A, 0x00, 0x00},
	{0x00, 0x32, 0x4A, 0x48, 0x48, 0x30, 0x00, 0x00},
	{0x00, 0x3A, 0x41, 0x41, 0x21, 0x7A, 0x00, 0x00},
	{0x00, 0x3A, 0x42, 0x40, 0x20, 0x78, 0x00, 0x00},
	{0x00, 0x00, 0x9D, 0xA0, 0xA0, 0x7D, 0x00, 0x00},
	{0x00, 0x3D, 0x42, 0x42, 0x42, 0x3D, 0x00, 0x00},
	{0x00, 0x3D, 0x40, 0x40, 0x40, 0x3D, 0x00, 0x00},
	{0x00, 0x3C, 0x24, 0xFF, 0x24, 0x24, 0x00, 0x00},
	{0x00, 0x48, 0x7E, 0x49, 0x43, 0x66, 0x00, 0x00},
	{0x00, 0x2B, 0x2F, 0xFC, 0x2F, 0x2B, 0x00, 0x00},
	{0x00, 0xFF, 0x09, 0x29, 0xF6, 0x20, 0x00, 0x00},
	{0x00, 0xC0, 0x88, 0x7E, 0x09, 0x03, 0x00, 0x00},
	{0x00, 0x20, 0x54, 0x54, 0x79, 0x41, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x44, 0x7D, 0x41, 0x00, 0x00},
	{0x00, 0x30, 0x48, 0x48, 0x4A, 0x32, 0x00, 0x00},
	{0x00, 0x38, 0x40, 0x40, 0x22, 0x7A, 0x00, 0x00},
	{0x00, 0x00, 0x7A, 0x0A, 0x0A, 0x72, 0x00, 0x00},
	{0x00, 0x7D, 0x0D, 0x19, 0x31, 0x7D, 0x00, 0x00},
	{0x00, 0x26, 0x29, 0x29, 0x2F, 0x28, 0x00, 0x00},
	{0x00, 0x26, 0x29, 0x29, 0x29, 0x26, 0x00, 0x00},
	{0x00, 0x30, 0x48, 0x4D, 0x40, 0x20, 0x00, 0x00},
	{0x00, 0x38, 0x08, 0x08, 0x08, 0x08, 0x00, 0x00},
	{0x00, 0x08, 0x08, 0x08, 0x08, 0x38, 0x00, 0x00},
	{0x00, 0x2F, 0x10, 0xC8, 0xAC, 0xBA, 0x00, 0x00},
	{0x00, 0x2F, 0x10, 0x28, 0x34, 0xFA, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x7B, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x08, 0x14, 0x2A, 0x14, 0x22, 0x00, 0x00},
	{0x00, 0x22, 0x14, 0x2A, 0x14, 0x08, 0x00, 0x00},
	{0x00, 0x55, 0x00, 0x55, 0x00, 0x55, 0x00, 0x00},
	{0x00, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x00, 0x00},
	{0x00, 0xFF, 0x55, 0xFF, 0x55, 0xFF, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x10, 0xFF, 0x00, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0xFF, 0x00, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0xFF, 0x00, 0xFF, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0xF0, 0x10, 0xF0, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0xFC, 0x00, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0xF7, 0x00, 0xFF, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0xF4, 0x04, 0xFC, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x17, 0x10, 0x1F, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x1F, 0x10, 0x1F, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0x1F, 0x00, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x10, 0xF0, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x1F, 0x10, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x10, 0x1F, 0x10, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x10, 0xF0, 0x10, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0xFF, 0x10, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x10, 0x10, 0x10, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x10, 0xFF, 0x10, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0xFF, 0x14, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x1F, 0x10, 0x17, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0xFC, 0x04, 0xF4, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x17, 0x10, 0x17, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0xF4, 0x04, 0xF4, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0xFF, 0x00, 0xF7, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0xF7, 0x00, 0xF7, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0x17, 0x14, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x1F, 0x10, 0x1F, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0xF4, 0x14, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0xF0, 0x10, 0xF0, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x1F, 0x10, 0x1F, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x1F, 0x14, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0xFC, 0x14, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0xF0, 0x10, 0xF0, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0xFF, 0x10, 0xFF, 0x00, 0x00},
	{0x00, 0x14, 0x14, 0x14, 0xFF, 0x14, 0x00, 0x00},
	{0x00, 0x10, 0x10, 0x10, 0x1F, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0xF0, 0x10, 0x00, 0x00},
	{0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00},
	{0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00},
	{0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00},
	{0x00, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x38, 0x44, 0x00, 0x00},
	{0x00, 0xFC, 0x4A, 0x4A, 0x4A, 0x34, 0x00, 0x00},
	{0x00, 0x7E, 0x02, 0x02, 0x06, 0x06, 0x00, 0x00},
	{0x00, 0x02, 0x7E, 0x02, 0x7E, 0x02, 0x00, 0x00},
	{0x00, 0x63, 0x55, 0x49, 0x41, 0x63, 0x00, 0x00},
	{0x00, 0x38, 0x44, 0x44, 0x3C, 0x04, 0x00, 0x00},
	{0x00, 0x40, 0x7E, 0x20, 0x1E, 0x20, 0x00, 0x00},
	{0x00, 0x06, 0x02, 0x7E, 0x02, 0x02, 0x00, 0x00},
	{0x00, 0x99, 0xA5, 0xE7, 0xA5, 0x99, 0x00, 0x00},
	{0x00, 0x1C, 0x2A, 0x49, 0x2A, 0x1C, 0x00, 0x00},
	{0x00, 0x4C, 0x72, 0x01, 0x72, 0x4C, 0x00, 0x00},
	{0x00, 0x30, 0x4A, 0x4D, 0x4D, 0x30, 0x00, 0x00},
	{0x00, 0x30, 0x48, 0x78, 0x48, 0x30, 0x00, 0x00},
	{0x00, 0xBC, 0x62, 0x5A, 0x46, 0x3D, 0x00, 0x00},
	{0x00, 0x3E, 0x49, 0x49, 0x49, 0x00, 0x00, 0x00},
	{0x00, 0x7E, 0x01, 0x01, 0x01, 0x7E, 0x00, 0x00},
	{0x00, 0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 0x00, 0x00},
	{0x00, 0x44, 0x44, 0x5F, 0x44, 0x44, 0x00, 0x00},
	{0x00, 0x40, 0x51, 0x4A, 0x44, 0x40, 0x00, 0x00},
	{0x00, 0x40, 0x44, 0x4A, 0x51, 0x40, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0xFF, 0x01, 0x03, 0x00, 0x00},
	{0x00, 0xE0, 0x80, 0xFF, 0x00, 0x00, 0x00, 0x00},
	{0x00, 0x08, 0x08, 0x6B, 0x6B, 0x08, 0x00, 0x00},
	{0x00, 0x36, 0x12, 0x36, 0x24, 0x36, 0x00, 0x00},
	{0x00, 0x06, 0x0F, 0x09, 0x0F, 0x06, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00},
	{0x00, 0x30, 0x40, 0xFF, 0x01, 0x01, 0x00, 0x00},
	{0x00, 0x00, 0x1F, 0x01, 0x01, 0x1E, 0x00, 0x00},
	{0x00, 0x00, 0x19, 0x1D, 0x17, 0x12, 0x00, 0x00},
	{0x00, 0x00, 0x3C, 0x3C, 0x3C, 0x3C, 0x00, 0x00},
	{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * @brief nhd_c12832a1z write command.
 * @param dev - The device structure.
 * @param cmd - Command to be written.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int nhd_c12832a1z_write_cmd(struct nhd_c12832a1z_dev *dev, uint8_t cmd)
{
	int ret;

	if(!dev->spi_desc || !dev->dc_pin)
		return -EINVAL;

	ret = no_os_gpio_set_value(dev->dc_pin, NHD_C12832A1Z_DC_CMD);
	if (ret)
		return ret;

	return no_os_spi_write_and_read(dev->spi_desc, &cmd, 1U);
}

/**
 * @brief nhd_c12832a1z write data.
 * @param dev - The device structure.
 * @param data - Data to be written.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int nhd_c12832a1z_write_data(struct nhd_c12832a1z_dev *dev, uint8_t data)
{
	int ret;

	if(!dev->spi_desc || !dev->dc_pin)
		return -EINVAL;

	ret = no_os_gpio_set_value(dev->dc_pin, NHD_C12832A1Z_DC_DATA);
	if (ret)
		return ret;

	return no_os_spi_write_and_read(dev->spi_desc, &data, 1U);
}

/**
 * @brief nhd_c12832a1z print string on LCD.
 * @param dev - The device structure.
 * @param msg - Message to be printed.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int nhd_c12832a1z_print_string(struct nhd_c12832a1z_dev *dev, char *msg)
{
	int ret;
	unsigned int i, j;
	uint8_t framebuffer_memory[NR_PAGES][NR_COLUMNS] = { 0 };
	uint8_t page = PAGE_START_ADDR;
	int32_t count = strlen(msg);
	int32_t t_cursor = 0;

	if ((t_cursor + count) > NR_COLUMNS)
		count = NR_COLUMNS - t_cursor;

	for (j = 0; j < count; ++j) {
		int cursor = (t_cursor + j) % NR_CHAR;
		int y = cursor >> 4; // page
		int x = (cursor & 0xf) << 3; // segment

		for (i = 0; i < 8; i++)
			framebuffer_memory[y][x+i] = ASC16[msg[cursor]][i];
	}

	ret = nhd_c12832a1z_write_cmd(dev, NHD_C12832A1Z_DISP_OFF);
	if (ret)
		return ret;

	ret = nhd_c12832a1z_write_cmd(dev,
				      DISPLAY_START_OFFSET); // Display start address + 0x40
	if (ret)
		return ret;

	for (i = 0; i < 4; i++) {
		// 32pixel display / 8 pixels per page = 4 pages
		ret = nhd_c12832a1z_write_cmd(dev, page); // send page address
		if (ret)
			return ret;

		// Sets the most significant 4 bits of the display RAM column address.
		ret = nhd_c12832a1z_write_cmd(dev, 0x10); // column address upper 4 bits + 0x10
		// Sets the least significant 4 bits of the display RAM column address.
		if (ret)
			return ret;

		ret = nhd_c12832a1z_write_cmd(dev, 0x00); // column address lower 4 bits + 0x00
		if (ret)
			return ret;

		for (j = 0; j < 128; j++) {
			// 128 columns wide
			ret = nhd_c12832a1z_write_data(dev,
						       framebuffer_memory[i][j]); // send picture data
			if (ret)
				return ret;
		}
		page++; // after 128 columns, go to next page
	}

	return nhd_c12832a1z_write_cmd(dev, NHD_C12832A1Z_DISP_ON);
}

/**
 * @brief nhd_c12832a1z clear LCD.
 * @param dev - The device structure.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int nhd_c12832a1z_clear_lcd(struct nhd_c12832a1z_dev *dev)
{
	int ret;
	unsigned int i;
	uint8_t page = PAGE_START_ADDR;

	ret = nhd_c12832a1z_write_cmd(dev, NHD_C12832A1Z_DISP_OFF);
	if (ret)
		return ret;

	ret = nhd_c12832a1z_write_cmd(dev,
				      DISPLAY_START_OFFSET); // Display start address + 0x40
	if (ret)
		return ret;
	for (i = 0; i < NR_PAGES; i++) {
		// 32pixel display / 8 pixels per page = 4 pages
		ret = nhd_c12832a1z_write_cmd(dev, page); // send page address
		if (ret)
			return ret;
		// Sets the most significant 4 bits of the display RAM column address.
		ret = nhd_c12832a1z_write_cmd(dev, 0x10); // column address upper 4 bits + 0x10
		// Sets the least significant 4 bits of the display RAM column address.
		if (ret)
			return ret;
		ret = nhd_c12832a1z_write_cmd(dev, 0x00); // column address lower 4 bits + 0x00
		if (ret)
			return ret;
		for (unsigned int j = 0; j < NR_COLUMNS; j++) {
			// 128 columns wide
			ret = nhd_c12832a1z_write_data(dev, 0x00); // send picture data
			if (ret)
				return ret;
		}
		page++; // after 128 columns, go to next page
	}

	return nhd_c12832a1z_write_cmd(dev, NHD_C12832A1Z_DISP_ON);
}

/**
 * @brief Initializes nhd_c12832a1z for display screening.
 * @param device - The device structure.
 * @param init_param - Initialization parameters.
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int nhd_c12832a1z_init(struct nhd_c12832a1z_dev **device,
		       struct nhd_c12832a1z_init_param init_param)
{
	struct nhd_c12832a1z_dev *dev;
	int ret;

	dev = (struct nhd_c12832a1z_dev *)calloc(1, sizeof(*dev));
	if (!dev)
		return -ENOMEM;

	ret = no_os_spi_init(&dev->spi_desc, init_param.spi_ip);
	if (ret)
		goto error_dev;

	ret = no_os_gpio_get(&dev->dc_pin, init_param.dc_pin_ip);
	if (ret)
		goto error_spi;

	ret = no_os_gpio_get_optional(&dev->reset_pin, init_param.reset_pin_ip);
	if (ret)
		goto error_dc;

	// initial pin state
	ret = no_os_gpio_direction_output(dev->dc_pin, NHD_C12832A1Z_DC_CMD);
	if (ret)
		goto error_rst;

	if (dev->reset_pin) {
		ret = no_os_gpio_direction_output(dev->reset_pin, NHD_C12832A1Z_RST_ON);
		if (ret)
			goto error_rst;

		no_os_udelay(3U);
		ret = no_os_gpio_set_value(dev->reset_pin, NHD_C12832A1Z_RST_OFF);
		if (ret)
			goto error_rst;
	}

	ret = nhd_c12832a1z_write_cmd(dev, NDH_C12832A1Z_ADC_NORMAL);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_write_cmd(dev, NHD_C12832A1Z_DISP_OFF);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_write_cmd(dev, NDH_C12832A1Z_COM_REVERSE);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_write_cmd(dev, NDH_C12832A1Z_LCD_BIAS);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_write_cmd(dev, NDH_C12832A1Z_PWR_CTRL);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_write_cmd(dev, NDH_C12832A1Z_RES_RATIO);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_write_cmd(dev, NDH_C12832A1Z_ELECTRIC_VOL);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_write_cmd(dev, NDH_C12832A1Z_ELECTRIC_VAL);
	if (ret)
		goto error_rst;

	ret = nhd_c12832a1z_clear_lcd(dev);
	if (ret)
		goto error_rst;

	*device = dev;

	return 0;

error_rst:
	no_os_gpio_remove(dev->reset_pin);
error_dc:
	no_os_gpio_remove(dev->dc_pin);
error_spi:
	no_os_spi_remove(dev->spi_desc);
error_dev:
	free(dev);

	return ret;
}

/**
 * @brief  nhd_c12832a1z remove allocated resources
 * @param dev - The device structure
 * @return Returns 0 in case of success or negative error code otherwise.
 */
int nhd_c12832a1z_remove(struct nhd_c12832a1z_dev *dev)
{
	int ret;

	ret = no_os_gpio_remove(dev->reset_pin);
	if (ret)
		return ret;

	ret = no_os_gpio_remove(dev->dc_pin);
	if (ret)
		return ret;

	ret = no_os_spi_remove(dev->spi_desc);
	if (ret)
		return ret;

	free(dev);

	return 0;
}
