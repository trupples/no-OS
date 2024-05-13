/***************************************************************************//**
 *   @file   ad4692.h
 *   @brief  Header file for ad4692 Driver.
 *   @author Radu Sabau (radu.sabau@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
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
#ifndef _AD4692_H_
#define _AD4692_H_

#include <stdbool.h>
#include "no_os_pwm.h"
#include "no_os_spi.h"
#include "no_os_gpio.h"

#define AD4692_STATUS_REG			0x14
#define AD4692_CLAMP_STATUS1_REG		0x1A
#define AD4692_CLAMP_STATUS2_REG		0x1B
#define AD4692_ADC_SETUP_REG			0x20
#define AD4692_REFRENCE_CONTROL_REG		0x21
#define AD4692_SEQUENCER_CONTROL_REG		0x22
#define AD4692_INTERNAL_OSCILLATOR_REG		0x23
#define AD4692_STD_SEQ_CONFIG			0x24
#define AD4692_SPARE_CONTROL_REG		0x2A
#define AD4692_CHANNEL_CONFIG(n)		(0x30 + (n))
#define AD4692_IN_SELECT_AS(n)			(0x100 + (n))

#define AD4692_CONV_START_REG			0x180
#define AD4692_STATE_RESET_REG			0x181
#define AD4692_ADC_OPERATION_REG		0x182
#define AD4692_ACC_MASK1_REG			0x184
#define AD4692_ACC_MASK2_REG			0x185
#define AD4692_ACC_COUNT_LIMIT(n)		(0x186 + (n))
#define AD4692_GPIO_MODE1_REG			0x196
#define AD4692_GPIO_MODE2_REG			0x197
#define AD4692_GPIO_READ_REG			0x1A0
#define AD4692_ACC_STATUS_FULL1_REG		0x1B0
#define AD4692_ACC_STATUS_FULL2_REG		0x1B1
#define AD4692_ACC_STATUS_OVERRUN1_REG		0x1B2
#define AD4692_ACC_STATUS_OVERRUN2_REG		0x1B3
#define AD4692_ACC_STATUS_SAT1_REG		0x1B4
#define AD4692_ACC_STATUS_SAT2_REG		0x1B5
#define AD4692_ACC_SAT_OVR_REG(n)		(0x1C0 + (n))
#define AD4692_ACC16B(n)			(0x200 + 2 * (n))
#define AD4692_ACC16B_STATUS(n)			(0x220 + 3 * (n))
#define AD4692_ACC24B(n)			(0x250 + 3 * (n))
#define AD4692_ACC24B_STATUS(n)			(0x280 + 4 * (n))

enum ad4692_spi_mode {
	AD4692_CNV_CLOCK,
	AD4692_CNV_BURST,
	AD4692_AUTONOMOUS,
	AD4692_SPI_BURST,
	AD4692_MANUAL_MODE,
};

enum ad4692_gpio_sel {
	AD4692_GPIO_HIGHZ,
	AD4692_GPIO_OUTPUT_LOW,
	AD4692_GPIO_OUTPUT_HIGH,
	AD4692_GPIO_INPUT,
	AD4692_GPIO_OUTPUT_ADC_BUSY,
	AD4692_GPIO_OUTPUT_SEQ_DONEb,
	AD4692_GPIO_OUTPUT_DATA_READYb,
	AD4692_GPIO_OUTPUT_ACC_OVF_ERR,
	AD4692_GPIO_OUTPUT_ACC_SAT_ERR,
};

enum ad4692_type {
	ID_AD4692,
	ID_AD4691,
	ID_AD4694,
	ID_AD4693,
};

struct ad4692_init_param {
	struct no_os_spi_init_param *comm_param;

	struct no_os_gpio_init_param *gpio0_param;
	struct no_os_gpio_init_param *gpio1_param;
	struct no_os_gpio_init_param *gpio2_param;
	struct no_os_gpio_init_param *gpio3_param;

	/* SPI Standard. */
	struct no_os_pwm_init_param *conv_param;

	enum ad4692_spi_mode mode;
};

struct ad4692_desc {
	struct no_os_spi_desc *comm_desc;

	struct no_os_gpio_desc *gpio0_desc;
	struct no_os_gpio_desc *gpio1_desc;
	struct no_os_gpio_desc *gpio2_desc;
	struct no_os_gpio_desc *gpio3_desc;

	/* SPI Standard. */
	struct no_os_pwm_desc *conv_desc;

	enum ad4692_spi_mode mode;

	uint8_t resolution;
	uint8_t max_rate;
	uint8_t num_channels;
};

int ad4692_reg_read(struct ad4692_desc *, uint32_t, uint32_t *);

int ad4692_reg_write(struct ad4692_desc *, uint32_t, uint32_t);

int ad4692_reg_update(struct ad4692_desc *, uint32_t, uint32_t, uint32_t);

int ad4692_std_seq_ch(struct ad4692_desc *, uint16_t);

int ad4692_get_ch(struct ad4692_desc *, uint8_t, uint32_t *);

int ad4692_gpio_set(struct ad4692_desc *, enum ad4692_gpio_sel);

int ad4692_gpio_get_value(struct ad4692_desc *, enum no_os_gpio_values);

int ad4692_init(struct ad4692_desc **, struct ad4692_init_param *);

int ad4692_remove(struct ad4692_desc *);

#endif /* _AD4692_H_ */
