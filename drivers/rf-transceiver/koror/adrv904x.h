/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ADRV9025
 *
 * Copyright 2020-2023 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef IIO_TRX_ADRV9025_H_
#define IIO_TRX_ADRV9025_H_

#include "adi_adrv904x_types.h"
#include "adi_adrv904x_core.h"
// #include "adi_adrv9025_agc.h"
// #include "adi_adrv9025_arm.h"
// #include "adi_adrv9025_cals.h"
// #include "adi_adrv9025_dfe.h"
// #include "adi_adrv9025_error.h"
// #include "adi_adrv9025_gpio.h"
// #include "adi_adrv9025_hal.h"
// #include "adi_adrv9025_radioctrl.h"
// #include "adi_adrv9025_rx.h"
// #include "adi_adrv9025_tx.h"
// #include "adi_adrv9025_user.h"
// #include "adi_adrv9025_utilities.h"
// #include "adi_adrv9025_version.h"
// #include "adi_adrv9025_data_interface.h"
// #include "adi_platform.h"

#include "no_os_platform.h"
#include "no_os_mutex.h"
#include "no_os_clk.h"
#include <stdbool.h>
#include <stdint.h>

// #define MIN_GAIN_mdB		0
// #define MAX_RX_GAIN_mdB		30000
// #define MAX_OBS_RX_GAIN_mdB	30000
// #define RX_GAIN_STEP_mdB	500

/*
 * JESD204-FSM defines
 */

#define DEFRAMER0_LINK_TX	0
#define DEFRAMER1_LINK_TX	1
#define FRAMER0_LINK_RX		2
#define FRAMER1_LINK_RX		3
#define FRAMER2_LINK_RX		4

// enum adrv9025_rx_ext_info {
// 	ADRV9025_RSSI,
// 	ADRV9025_RX_QEC,
// 	ADRV9025_RX_HD2,
// 	ADRV9025_RX_DIG_DC,
// 	ADRV9025_RX_RF_BANDWIDTH,
// };

// enum adrv9025_tx_ext_info {
// 	ADRV9025_TX_QEC,
// 	ADRV9025_TX_LOL,
// 	ADRV9025_TX_RF_BANDWIDTH,
// };

// enum adrv9025_device_id {
// 	ADRV9025_ID_ADRV9025,
// 	ADRV9025_ID_ADRV9026,
// 	ADRV9025_ID_ADRV9029,
// };

enum adrv904x_clocks {
	ADRV904X_RX_SAMPL_CLK,
	ADRV904X_TX_SAMPL_CLK,
	ADRV904X_ORX_SAMPL_CLK,
	NUM_ADRV904X_CLKS,
};

// #define ADRV9025_MAX_NUM_GAIN_TABLES 10

struct adrv904x_rf_phy {
	struct no_os_spi_desc			*spi_desc;
	adi_adrv904x_Device_t			adi_adrv904x_device;
	adi_adrv904x_Device_t			*kororDevice;
	adi_adrv904x_SpiConfigSettings_t	spiSettings;
	adi_adrv904x_SpiOptions_t 		spiOptions;
	adi_adrv904x_Init_t			deviceInitStruct;
	adi_adrv904x_TrxFileInfo_t 		trxBinaryInfoPtr;
	adi_adrv904x_PostMcsInit_t		adrv904xPostMcsInitInst;
	struct adrv904x_hal_cfg			hal;

	struct jesd204_dev			*jdev;
	/* protect against device accesses */
	void					*lock;
	uint32_t				tx_iqRate_kHz;
	uint32_t				rx_iqRate_kHz;

	struct no_os_clk_desc			*dev_clk;

// 	struct clk				*clk_ext_lo_rx;
// 	struct clk				*clk_ext_lo_tx;
	struct no_os_clk_desc			*clks[NUM_ADRV904X_CLKS];

	uint8_t					device_id;
	bool					is_initialized;
	int					spi_device_id;

	struct axi_adc				*rx_adc;
	struct axi_dac				*tx_dac;
};

struct adrv904x_init_param {
	struct no_os_spi_init_param	*spi_init;
	struct no_os_clk_desc		*dev_clk;
	struct adi_adrv904x_Device	*adrv904x_device;
};

// /* Initialize the device. */
int32_t adrv904x_init(struct adrv904x_rf_phy **device,
		      const struct adrv904x_init_param *init_param);
// int adrv9025_hdl_loopback(struct adrv9025_rf_phy *phy, bool enable);
// int adrv9025_register_axi_converter(struct adrv9025_rf_phy *phy);
// int adrv9025_spi_read(struct no_os_spi_desc *spi, uint32_t reg);
// int adrv9025_spi_write(struct no_os_spi_desc *spi, uint32_t reg, uint32_t val);

int adrv904x_setup(struct adrv904x_rf_phy *phy);
int adrv904x_remove(struct adrv904x_rf_phy *phy);

adi_adrv904x_SpiConfigSettings_t *adrv904x_spi_settings_get(void);
adi_adrv904x_SpiOptions_t *adrv904x_spi_options_get(void);

int adrv904x_post_setup(struct adrv904x_rf_phy *phy);

#endif
