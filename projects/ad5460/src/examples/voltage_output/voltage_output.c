/***************************************************************************//**
 *   @file   Voltage Output.c
 *   @brief  Basic example header for eval-ad5460 project
 *   @author Raquel Grau (raquel.grau@analog.com)
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

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "voltage_output.h"
#include "common_data.h"
#include "ad5460.h"
#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "no_os_print_log.h"

/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/
/***************************************************************************//**
 * @brief Basic example main execution.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously the while(1) loop and will not return.
*******************************************************************************/
int voltage_output_main()
{
	struct ad5460_desc *ad5460_desc;
	int ret;

	ret = ad5460_init(&ad5460_desc, &ad5460_ip);
	if (ret)
		goto error;

	pr_info("ad5460 successfully initialized!\r\n");

	//Configure voltage range to -12 V to 12 V
	ret = ad5460_set_channel_vout_range(ad5460_desc, 0, AD5460_VOUT_RANGE_NEG12_12V);
	if(ret)
	{
		pr_info("Error setting channel range to '-12 to 12 V\r\n");
		goto error;
	}
	ret = ad5460_set_channel_function(ad5460_desc, 0, AD5460_VOLTAGE_OUT); 

	return 0;

error_ad5460:
	ad5460_remove(ad5460_desc);
error:
	pr_info("AD5460 Error!\r\n");
	return ret;
}
