/***************************************************************************//**
*   @file   iio_sps.h
*   @brief  SPS Demo
*   @author Darius Berghe (darius.berghe@analog.com)
********************************************************************************
* Copyright 2020(c) Analog Devices, Inc.
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
#ifndef IIO_SPS_H
#define IIO_SPS_H

#include "adpd188.h"
#include "iio.h"

enum gesture {
	click,
	up,
	down,
	left,
	right
};

extern char *gestures[6];

struct sps_iio_init_param {
	struct adpd188_dev *dev;
	int th_intensity;
	double th_click;
	void * pHdev;
};

struct sps_iio_desc {
	struct adpd188_dev *dev;
	int th_intensity;
	int th_click;
	uint8_t gestures;
	uint8_t d_gestures;
	int gestureStartX;
	int gestureStartY;
	bool event;
	int evc;
	char io1[16];
	char io2[16];
	char io3[16];
	void *pHdev;
};

extern struct iio_device iio_sps_device;

int32_t sps_iio_init(struct sps_iio_desc **dev,
			 struct sps_iio_init_param *init_param);
int32_t sps_iio_remove(struct sps_iio_desc *dev);

int sps_gesture_detection(struct sps_iio_desc *iiodev);
int sps_get_fifo_data(struct adpd188_dev *desc, int32_t *buff);

#endif /* IIO_SPS_H */

