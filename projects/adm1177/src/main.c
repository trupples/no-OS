/***************************************************************************//**
 *   @file   main.c
 *   @brief  Source file of Main Example for ADM1177
 *   @author Radu Sabau (radu.sabau@analog.com)
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

#include <stdio.h>

#include "parameters.h"
#include "no_os_delay.h"
#include "maxim_i2c.h"
#include "no_os_uart.h"
#include "maxim_uart.h"
#include "maxim_uart_stdio.h"
#include "adm1177.h"

#define I2C_DEVICE_ID	0
#define I2C_OPS			&max_i2c_ops
#define I2C_EXTRA		&adm1177_i2c_extra

struct max_uart_init_param max_uart_extra_ip = {
	.flow = UART_FLOW_DIS,
//     .vssel = MXC_GPIO_VSSEL_VDDIOH,
};
struct no_os_uart_init_param uart_ip = {
	.asynchronous_rx = true,
	.irq_id = UART_IRQ_ID,
	.device_id = UART_DEVICE_ID,
	.baud_rate = UART_BAUDRATE,
	.size = NO_OS_UART_CS_8,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.platform_ops = &max_uart_ops,
	.extra = &max_uart_extra_ip,
};

struct max_i2c_init_param adm1177_i2c_extra = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

struct adm1177_init_param adm1177_ip = {
	.i2c_init = {
		.device_id = I2C_DEVICE_ID,
		.slave_address = ADM1177_ADDRESS,
		.extra = I2C_EXTRA,
		.platform_ops = I2C_OPS,
		.max_speed_hz = 100000,
	},
};

#ifdef IIO_SUPPORT
#include "iio_adm1177.h"
#include "iio_app.h"
#endif

#ifndef DATA_BUFFER_SIZE
#define DATA_BUFFER_SIZE 400
#endif

uint16_t iio_data_buffer[DATA_BUFFER_SIZE*3*sizeof(int)];

int main(void)
{
	struct no_os_uart_desc *uart_desc;

	struct adm1177_dev *device;

	int ret;

	uint8_t status_byte;
	uint16_t conv_voltage;
	uint16_t conv_current;
	uint64_t voltage;
	uint64_t current;
#ifdef IIO_SUPPORT
	struct adm1177_iio_dev *adm1177_iio_dev = NULL;
	struct adm1177_iio_init_param adm1177_iio_init_param;

	struct iio_app_desc *app;
	struct iio_data_buffer adm1177_buff = {
		.buff = (void *)iio_data_buffer,
		.size = DATA_BUFFER_SIZE*3*sizeof(int),
	};
	struct iio_app_init_param app_init_param = { 0 };
#endif

#ifndef IIO_SUPPORT
	/* Initialize the uart */
	ret = no_os_uart_init(&uart_desc, &uart_ip);
	if(ret)
		return ret;

	no_os_uart_stdio(uart_desc);

	/* Initialize the ADM1177 */
	ret = adm1177_init(&device, &adm1177_ip);
	if(ret)
		return ret;

	/* Initializing the continuous voltage and current readback */
	ret = adm1177_write(device, ADM1177_CMD_V_CONT | ADM1177_CMD_I_CONT, 0);
	if(ret)
		return ret;

	ret = adm1177_read(device, &conv_voltage, &conv_current);
	if(ret)
		return ret;

	ret = adm1177_to_microvolts(device, conv_voltage, &voltage);
	if(ret)
		return ret;
	// current = adm1177_to_microampers(conv_current);

	ret = adm1177_to_microampers(conv_current, &current);
	if(ret)
		return ret;

	printf("V: %luuV  -  I: %luuA\n", voltage, current);

	no_os_mdelay(500);

	ret = adm1177_remove(device);

	no_os_uart_remove(uart_desc);

	return ret;
#else
	adm1177_iio_init_param.adm1177_initial = &adm1177_ip;
	ret = adm1177_iio_init(&adm1177_iio_dev, &adm1177_iio_init_param);
	if(ret)
		goto error;

	no_os_mdelay(15);

	struct iio_app_device iio_devices[] = {
		{
			.name = "adm1177",
			.dev = adm1177_iio_dev,
			.dev_descriptor = adm1177_iio_dev->iio_dev,
			.read_buff = &adm1177_buff,
		}
	};

	app_init_param.devices = iio_devices;
	app_init_param.nb_devices = NO_OS_ARRAY_SIZE(iio_devices);
	app_init_param.uart_init_params = uart_ip;

	ret = iio_app_init(&app, app_init_param);
	if(ret)
		return ret;

	return iio_app_run(app);
error:
	adm1177_iio_remove(adm1177_iio_dev);
	return ret;
#endif
}
