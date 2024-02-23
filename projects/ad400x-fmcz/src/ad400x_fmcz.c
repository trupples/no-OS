#include <stdio.h>
#include <sleep.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <xil_cache.h>
#include <xparameters.h>
#include <inttypes.h>
#include "xil_printf.h"
#include "spi_engine.h"
#include "ad400x.h"
#include "no_os_error.h"
#include "no_os_delay.h"

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
#define AD400x_EVB_SAMPLE_NO			10000
#define AD400X_DMA_BASEADDR             XPAR_AXI_PULSAR_ADC_DMA_BASEADDR
#define AD400X_SPI_ENGINE_BASEADDR      XPAR_SPI_PULSAR_ADC_SPI_PULSAR_ADC_AXI_REGMAP_BASEADDR
#define AD400x_SPI_CS                   0
#define AD400x_SPI_ENG_REF_CLK_FREQ_HZ	XPAR_PS7_SPI_0_SPI_CLK_FREQ_HZ

#ifndef SPI_ENGINE_OFFLOAD_EXAMPLE
#define SPI_ENGINE_OFFLOAD_EXAMPLE	1
#endif

/**
 * Print data on the right format.
 * @param adc_data - the data to print.
 * @param res the resolution of the device.
 * @param sign - signed or unsigned.
 * @param is_big_endian - is the data big endian.
 * @return - None.
 */
static void print_output_data(uint32_t adc_data, uint16_t res, char sign,
			      bool is_big_endian)
{
	uint32_t data = adc_data;

	if (is_big_endian) {
		data = no_os_get_unaligned_be32((uint8_t *)&data);
		data >>= 32 - res;
	}

	data = data & NO_OS_GENMASK(res, 0);
	if (sign == 's')
		xil_printf("ADC: %d\n\r", no_os_sign_extend32(data, res - 1));
	else
		xil_printf("ADC: %d\n\r", data);
}

int main()
{
	struct ad400x_dev *dev;
	uint32_t *offload_data;
	uint32_t adc_data;
	struct spi_engine_offload_init_param spi_engine_offload_init_param = {
		.offload_config = OFFLOAD_RX_EN,
		.rx_dma_baseaddr = AD400X_DMA_BASEADDR,
	};
	struct spi_engine_offload_message msg;
	uint32_t commands_data[2] = {0xFF, 0xFF};
	int32_t ret, i;
	enum ad400x_supported_dev_ids dev_id = ID_AD4020;

	uint16_t res = ad400x_device_resol[dev_id];

	struct spi_engine_init_param spi_eng_init_param  = {
		.ref_clk_hz = AD400x_SPI_ENG_REF_CLK_FREQ_HZ,
		.type = SPI_ENGINE,
		.spi_engine_baseaddr = AD400X_SPI_ENGINE_BASEADDR,
		.cs_delay = 2,
		.data_width = res,
	};

	struct ad400x_init_param ad400x_init_param = {
		.spi_init = {
			.chip_select = AD400x_SPI_CS,
			.max_speed_hz = 83333333,
			.mode = NO_OS_SPI_MODE_0,
			.platform_ops = &spi_eng_platform_ops,
			.extra = (void*)&spi_eng_init_param,
		},
		.reg_access_speed = 1000000,
		dev_id, /* dev_id */
		1,0,0,0,
	};

	print("Test\n\r");

	uint32_t spi_eng_msg_cmds[3] = {
		CS_LOW,
		READ(2),
		CS_HIGH
	};

	Xil_ICacheEnable();
	Xil_DCacheEnable();

	/* data must be byte alligned when offload is disabled */
	if (SPI_ENGINE_OFFLOAD_EXAMPLE == 0) {
		spi_eng_init_param.data_width =
			NO_OS_DIV_ROUND_UP(spi_eng_init_param.data_width, 8) * 8;
	}

	ret = ad400x_init(&dev, &ad400x_init_param);
	if (ret < 0)
		return ret;

	if (SPI_ENGINE_OFFLOAD_EXAMPLE == 0) {
		while(1) {
			ad400x_spi_single_conversion(dev, (uint8_t *)&adc_data);
			print_output_data(adc_data, res, ad400x_device_sign[dev_id], true);
		}
	}
	/* Offload example */
	else {
		ret = spi_engine_offload_init(dev->spi_desc, &spi_engine_offload_init_param);
		if (ret != 0)
			return -1;

		msg.commands = spi_eng_msg_cmds;
		msg.no_commands = NO_OS_ARRAY_SIZE(spi_eng_msg_cmds);
		msg.rx_addr = 0x800000;
		msg.tx_addr = 0xA000000;
		msg.commands_data = commands_data;

		ret = spi_engine_offload_transfer(dev->spi_desc, msg, AD400x_EVB_SAMPLE_NO);
		if (ret != 0)
			return ret;

		no_os_mdelay(2000);
		Xil_DCacheInvalidateRange(0x800000, AD400x_EVB_SAMPLE_NO * 4);
		offload_data = (uint32_t *)msg.rx_addr;

		for(i = 0; i < AD400x_EVB_SAMPLE_NO; i++) {
			print_output_data(*offload_data, res, ad400x_device_sign[dev_id], false);
			offload_data += 1;
		}
	}

	print("Success\n\r");

	Xil_DCacheDisable();
	Xil_ICacheDisable();
}
