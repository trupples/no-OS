/***************************************************************************//**
 *   @file   ad4692.c
 *   @brief  Implementation of ad4692 Driver.
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ad4692.h"
#include "no_os_delay.h"
#include "no_os_error.h"
#include "no_os_util.h"
#include "no_os_alloc.h"

static int ad4692_resolution[] = {
	[ID_AD4692] = 16,
	[ID_AD4691] = 18,
	[ID_AD4694] = 14,
	[ID_AD4693] = 20,
};

static int ad4692_max_rate[] = {
	[ID_AD4692] = 1000000,
	[ID_AD4691] = 500000,
	[ID_AD4694] = 1000000,
	[ID_AD4693] = 500000,
};

static int ad4692_num_channels[] = {
	[ID_AD4692] = 16,
	[ID_AD4691] = 16,
	[ID_AD4694] = 8,
	[ID_AD4693] = 8,
};

int ad4692_reg_read(struct ad4692_desc *desc, uint32_t reg, uint32_t *val)
{
	return 0;
}

int ad4692_reg_write(struct ad4692_desc *desc, uint32_t reg, uint32_t val)
{
	return 0;
}

int ad4692_reg_update(struct ad4692_desc *desc, uint32_t reg, uint32_t mask,
		      uint32_t val)
{
	return 0;
}

int ad4692_std_seq_ch(struct ad4692_desc *desc, uint16_t ch_mask)
{
	return 0;
}

int ad4692_get_ch(struct ad4692_desc *desc, uint8_t ch_index, uint32_t *val)
{
	return 0;
}

int ad4692_gpio_set(struct ad4692_desc *desc, enum ad4692_gpio_sel mode)
{
	return 0;
}

int ad4692_gpio_get_value(struct ad4692_desc *desc,
			  enum no_os_gpio_values value)
{
	return 0;
}

int ad4692_init(struct ad4692_desc **desc, struct ad4692_init_param *init_param)
{
	return 0;
}

int ad4692_remove(struct ad4692_desc *desc)
{
	return 0;
}
