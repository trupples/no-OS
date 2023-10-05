#include <stdio.h>

#include "parameters.h"
#include "no_os_uart.h"
#include "no_os_print_log.h"
#include "no_os_delay.h"
#include "maxim_spi.h"
#include "maxim_gpio.h"
#include "maxim_uart.h"
#include "maxim_uart_stdio.h"
#include "max14906.h"
#include <string.h>

#ifdef IIO_SUPPORT
#include "iio_max14906.h"
#include "iio_app.h"
#endif

/** Define UART and SPI macro's corresponding to the uController used. */
#define UART_IRQ_ID		UART1_IRQn
#define UART_DEVICE_ID		1
#define UART_BAUDRATE		57600
#define UART_EXTRA		&max14906_uart_extra
#define UART_OPS		&max_uart_ops

#define SPI_DEVICE_ID		1
#define SPI_OPS			&max_spi_ops
#define SPI_EXTRA		&max14906_spi_extra
#define SPI_CS			0
#define SPI_BAUDRATE		100000

/** Defining specific mask for this example. */
#define MAX14906_SLED_CH(x)	NO_OS_BIT(x + MAX14906_CHANNELS)

/** Buffer macros and variable for iio. */
#define DATA_BUFFER_SIZE	400

uint16_t iio_data_buffer[DATA_BUFFER_SIZE*4*sizeof(int)];

/** MAX specific uart and spi. */
struct max_uart_init_param max14906_uart_extra = {
	.flow = UART_FLOW_DIS,
	.vssel = MXC_GPIO_VSSEL_VDDIO,
};

struct max_spi_init_param max14906_spi_extra = {
	.num_slaves = 1,
	.polarity = SPI_SS_POL_LOW,
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

struct max_gpio_init_param gpio_extra_ip = {
	.vssel = MXC_GPIO_VSSEL_VDDIO
};

static const char *climit_name[] = {
	"600mA", "130mA", "300mA", "1.2A",
};

/** NO-OS uart and spi. */
struct no_os_uart_init_param max14906_uart_ip = {
	.device_id = UART_DEVICE_ID,
	.baud_rate = UART_BAUDRATE,
	.size = NO_OS_UART_CS_8,
	.platform_ops = UART_OPS,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.extra = UART_EXTRA,
};

struct no_os_spi_init_param max14906_spi_ip = {
	.device_id = SPI_DEVICE_ID,
	.extra = SPI_EXTRA,
	.max_speed_hz = SPI_BAUDRATE,
	.platform_ops = SPI_OPS,
	.chip_select = SPI_CS,
};

void gpio_callback_fn(void *ctx)
{
	struct max149x6_desc *desc = ctx;
	int ret;
	uint32_t val;

	ret = max149x6_reg_read(desc, MAX14906_INT_REG, &val);
	if (val)
		pr_info("Fault detected!\n");
}

int main(void)
{
	int ret;

	/** Basic example (doesn't include the iio driver.) */
#ifndef IIO_SUPPORT
	int i;
	uint32_t val;
	struct no_os_uart_desc *uart_desc;

	struct max149x6_desc *max14906_desc;
	/** max149x6 init param to be used for both 14906 and 14916. */
	struct max149x6_init_param max14906_ip = {
		.chip_address = 0,
		.comm_param = &max14906_spi_ip,
		.crc_en = true,
	};

	enum max14906_climit climit = MAX14906_CL_300;
	enum max14906_climit climit2 = MAX14906_CL_600;

	ret = no_os_uart_init(&uart_desc, &max14906_uart_ip);
	if (ret)
		goto error;

	no_os_uart_stdio(uart_desc);

	/** Confiuring pin 5 of port 0 as a input GPIO. */
	struct no_os_gpio_desc *gpio_desc;
	struct no_os_gpio_init_param gpio_param = {
		.port = 0,
		.pull = NO_OS_PULL_NONE,
		.number = 5,
		.platform_ops = &max_gpio_ops,
		.extra = &gpio_extra_ip,
	};


	ret = no_os_gpio_get(&gpio_desc, &gpio_param);
	if (ret)
		return ret;

	ret = no_os_gpio_direction_input(gpio_desc);
	if (ret)
		return ret;

	/** MAX14906 Initialization */
	ret = max14906_init(&max14906_desc, &max14906_ip);
	if (ret)
		goto error;

	/** Testing the UART. */
	pr_info("Hello World!\n");

	struct no_os_irq_ctrl_desc *global_desc;
	struct no_os_irq_init_param global_desc_param = {
		.irq_ctrl_id = GPIO_IRQ_ID,
		.platform_ops = &max_irq_ops,
		.extra = NULL
	};

	ret = no_os_irq_ctrl_init(&global_desc, &global_desc_param);
	if (ret)
		return ret;

	/** GPIO Interrupt Controller */
	struct no_os_irq_ctrl_desc *gpio_irq_desc;
	struct no_os_irq_init_param gpio_irq_desc_param = {
		.irq_ctrl_id = GPIO_IRQ_ID,
		.platform_ops = GPIO_IRQ_OPS,
		.extra = NULL
	};

	ret = no_os_irq_ctrl_init(&gpio_irq_desc, &gpio_irq_desc_param);
	if (ret)
		return ret;

	struct no_os_callback_desc gpio_cb = {
		.callback = gpio_callback_fn,
		/** Parameter to be passed when the callback is called. */
		.ctx = max14906_desc,
		.event = NO_OS_EVT_GPIO,
		.peripheral = NO_OS_GPIO_IRQ,
		.handle = NULL
	};

	/**
	 * The callback will be registered on pin 21 of the GPIO port 2 in this case
	 * (the port is specified by the id field of the gpio_irq_desc).
	*/
	ret = no_os_irq_register_callback(gpio_irq_desc, 5, &gpio_cb);
	if (ret)
		return ret;

	/**
	 * Set the trigger condition of the interrupt on pin 21 to rising edge.
	*/
	ret = no_os_irq_trigger_level_set(gpio_irq_desc, 5, NO_OS_IRQ_EDGE_BOTH);
	if (ret)
		return ret;

	/**
	 * This calls the interrupt priority set function of the parent controller.
	*/
	ret = no_os_irq_set_priority(gpio_irq_desc, 5, 1);
	if (ret)
		return ret;

	/**
	 * Enbale interrupts on pin 21
	*/
	ret = no_os_irq_enable(gpio_irq_desc, 5);
	if (ret)
		return ret;

	ret = no_os_irq_enable(global_desc, GPIO0_IRQn);
	if (ret)
		return ret;

	/** Setting SLED set bit 1 in the config register. */
	ret = max149x6_reg_update(max14906_desc, MAX14906_CONFIG1_REG,
				  MAX14906_SLED_MASK, no_os_field_prep(MAX14906_SLED_MASK, 1));
	if (ret)
		goto error;

	/** Turning the Status LEDs on, then off. */
	for(i = 0; i <  MAX14906_CHANNELS; i++) {
		ret = max149x6_reg_update(max14906_desc, MAX14906_SETLED_REG,
					  MAX14906_SLED_CH(i), no_os_field_prep(MAX14906_SLED_CH(i), 1));
		if (ret)
			goto error;

		no_os_mdelay(500);

		ret = max149x6_reg_update(max14906_desc, MAX14906_SETLED_REG,
					  MAX14906_SLED_CH(i), no_os_field_prep(MAX14906_SLED_CH(i), 0));
		if (ret)
			goto error;

		no_os_mdelay(500);
	}

	/** Setting a current limit for channel 0. */
	ret = max14906_climit_set(max14906_desc, 0, climit);
	if(ret)
		goto error;

	/** Read current limit for all channels. */
	for(i = 0; i < MAX14906_CHANNELS; i++) {
		ret = max14906_climit_get(max14906_desc, i, &climit2);
		if(ret)
			goto cl_error;
		pr_info("Current limit for channel %d is %s\n", i,
			climit_name[climit2]);
	}

	/** Setting the on state for channel 0, and then verifying it. */
	if(max14906_desc->en_gpio) {
		ret = max14906_ch_set(max14906_desc, 0, 1);
		if(ret)
			goto ch_error;

		ret = max14906_ch_get(max14906_desc, 0, &val);
		if(ret)
			goto ch_error;
	}

	return 0;
#else /** IIO part (to build this type make TINYIIOD=y in terminal). */

	/** IIO descriptor and initialization parameter. */
	struct max14906_iio_desc *max14906_iio_desc;

	struct max14906_iio_desc_init_param max14906_iio_ip = {
		.max14906_init_param = &(struct max149x6_init_param)
		{
			.chip_address = 0,
			.comm_param = &max14906_spi_ip,
			.crc_en = true,
		},
		/** Channel configuration to be used in the example. */
		.channel_configs = {
			{
				.enabled = true,
				.function = MAX14906_OUT,
			},
			{
				.enabled = true,
				.function = MAX14906_IN,
			},
			{
				.enabled = true,
				.function = MAX14906_OUT,
			},
			{
				.enabled = true,
				.function = MAX14906_IN,
			},
		},
	};

	/** IIO app. */
	struct iio_app_desc *app;
	struct iio_data_buffer max14906_buff = {
		.buff = (void *)iio_data_buffer,
		.size = DATA_BUFFER_SIZE*4*sizeof(int),
	};
	struct iio_app_init_param app_init_param = { 0 };

	ret = max14906_iio_init(&max14906_iio_desc, &max14906_iio_ip);
	if(ret)
		goto error;

	/** Declaring iio_devices structure */
	struct iio_app_device iio_devices[] = {
		{
			.name = "max14906",
			.dev = max14906_iio_desc,
			.dev_descriptor = max14906_iio_desc->iio_dev,
			.read_buff = &max14906_buff
		}
	};

	/** Initializing IIO app init param. */
	app_init_param.devices = iio_devices;
	app_init_param.nb_devices = NO_OS_ARRAY_SIZE(iio_devices);
	app_init_param.uart_init_params = max14906_uart_ip;

	/** Initializing IIO app. */
	ret = iio_app_init(&app, app_init_param);
	if(ret)
		return ret;

	/** Running the IIO app (use iio_info or osc in terminal). */
	return iio_app_run(app);
#endif
error:
	pr_info("Error!\n");
	return ret;

cl_error:
	pr_info("CLimit Error!\n");
	return ret;

ch_error:
	pr_info("CH Error!\n");
	return ret;
}
