/***************************************************************************//**
 *   @file   ad9545.c
 *   @brief  Implementation of ad9545 Clock Driver.
 *   @author Jonathan (Jonathan.Santos@analog.com)
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
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "no_os_print_log.h"
#include "no_os_error.h"
#include "no_os_util.h"
#include "no_os_clk.h"
#include "ad9545.h"
#include "no_os_alloc.h"
#include "sys/types.h"

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/

/**
 * Wrapper used to read device registers.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9545_read_reg(struct ad9545_dev *dev,
			 uint8_t reg_addr,
			 uint8_t *reg_data)
{
	return dev->reg_read(dev, reg_addr, reg_data);
}

/**
 * Wrapper used to write to device registers.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9545_write_reg(struct ad9545_dev *dev,
			  uint8_t reg_addr,
			  uint8_t reg_data)
{
	return dev->reg_write(dev, reg_addr, reg_data);
}

/**
 * Wrapper used of multibyte reads.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @param count - Number of bytes to read.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9545_read_reg_multiple(struct ad9545_dev *dev,
				  uint8_t reg_addr,
				  uint8_t *reg_data,
				  uint16_t count)
{
	return dev->reg_read_multiple(dev, reg_addr, reg_data, count);
}

/**
 * Wrapper used of multibyte write.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @param count - Number of bytes to write.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9545_write_reg_multiple(struct ad9545_dev *dev,
				  uint8_t reg_addr,
				  uint8_t *reg_data,
				  uint16_t count)
{
	return dev->reg_write_multiple(dev, reg_addr, reg_data, count);
}

/**
 * Write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9545_write_mask(struct ad9545_dev *dev,
			   uint8_t reg_addr,
			   uint32_t mask,
			   uint8_t data)
{
	uint8_t reg_data;
	int32_t ret;

	ret = ad9545_read_reg(dev, reg_addr, &reg_data);
	if (ret < 0)
		return ret;

	reg_data &= ~mask;
	reg_data |= data;

	return ad9545_write_reg(dev, reg_addr, reg_data);
}

static int ad9545_get_r_div(struct ad9545_dev *dev, int addr, uint32_t *r_div)
{
	uint32_t regval;
	uint32_t val, div;
	int ret;

	ret = ad9545_read_reg_multiple(dev, AD9545_REF_X_RDIV(addr), &regval, 4);
	if (ret < 0)
		return ret;
	val = regval;
	div = no_os_field_get(AD9545_R_DIV_MSK, val);

	/* r-div ratios are mapped from 0 onward */
	*r_div = div + 1;

	return 0;
}

static int32_t ad9545_in_clk_recalc_rate(struct no_os_clk_desc *hw, uint64_t *rate)
{
	struct ad9545_dev *dev = hw->dev_desc;
	uint32_t div;
	int ret;

	ret = ad9545_get_r_div(dev, hw->hw_ch_num, &div);
	if (ret < 0) {
		pr_err("Could not read r div value.");
		return 1;
	}

	rate = NO_OS_DIV_ROUND_CLOSEST(dev->ref_in_clks->, div);
	// return NO_OSDIV_ROUND_CLOSEST(parent_rate, div);
	return 0;
}

static const struct no_os_clk_platform_ops ad9545_in_clk_ops = {
	.clk_recalc_rate = ad9545_in_clk_recalc_rate,
	.init = ad9545_in_clk_debug_init,
};
/* ------------------------------------------------------------------------------------------------ */
static int ad9545_io_update(struct ad9545_dev *dev)
{
	return ad9545_write_reg(dev, AD9545_IO_UPDATE, AD9545_UPDATE_REGS);
}

static int32_t ad9545_check_id(struct ad9545_dev *dev)
{
	
	uint32_t chip_id;
	uint32_t val;
	int ret;

	ret = ad9545_read_reg(dev, AD9545_PRODUCT_ID_LOW, &val);
	if (ret < 0)
		return ret;

	chip_id = val;
	ret = ad9545_read_reg(dev, AD9545_PRODUCT_ID_HIGH, &val);
	if (ret < 0)
		return ret;

	chip_id += val << 8;
	if (chip_id != AD9545_CHIP_ID) {
		pr_err("Unrecognized CHIP_ID 0x%X\n", chip_id);
		return -ENODEV;
	}
}

static int ad9545_set_r_div(struct ad9545_dev *dev, uint32_t div, int addr)
{
	uint32_t regval;
	uint32_t val;
	int ret;

	if (div > AD9545_R_DIV_MAX || div == 0) {
		pr_err("Invalid R-divider value (addr: %d): %u",
			addr, div);
		return -EINVAL;
	}

	/* r-div ratios are mapped from 0 onward */
	div -= 1;

	val = no_os_field_prep(AD9545_R_DIV_MSK, div);
	regval = val;
	ret = ad9545_write_reg_multiple(dev, AD9545_REF_X_RDIV(addr), &regval, 4);
	if (ret < 0)
		return ret;

	return ad9545_io_update(dev);
}
/* PARSE functions */
static int32_t ad9545_parse_inputs(struct ad9545_dev *dev, struct ad9545_init_param *init_param)
{
	struct clk *clk;
	bool prop_found;
	// int ref_ind;
	uint32_t val;
	int ret = 0;
	int i;

	for (i = 0; i < NO_OS_ARRAY_SIZE(init_param->ref_in_clks); i += 2) {
		// if (!fwnode_property_present(child, "adi,r-divider-ratio"))
		// 	continue;

		// ret = fwnode_property_read_u32(child, "reg", &ref_ind);
		// if (ret < 0) {
		// 	dev_err(st->dev, "reg not specified in ref node.");
		// 	goto out_fail;
		// }
		
		if (!init_param->ref_in_clks[i].ref_used) {
			continue;
		}

		dev->ref_in_clks[i].dev = dev;
		dev->ref_in_clks[i].ref_used = true;
		dev->ref_in_clks[i].address = init_param->ref_in_clks[i].address;

		dev->ref_in_clks[i].phase_lock_fill_rate = init_param->ref_in_clks[i].phase_lock_fill_rate;
		dev->ref_in_clks[i].phase_lock_drain_rate = init_param->ref_in_clks[i].phase_lock_drain_rate;
		dev->ref_in_clks[i].freq_lock_fill_rate = init_param->ref_in_clks[i].freq_lock_fill_rate;
		dev->ref_in_clks[i].freq_lock_drain_rate = init_param->ref_in_clks[i].freq_lock_drain_rate;

		dev->ref_in_clks[i].mode = init_param->ref_in_clks[i].mode;
		if (dev->ref_in_clks[i].mode == AD9545_SINGLE_ENDED){
			dev->ref_in_clks[i].s_conf = init_param->ref_in_clks[i].s_conf;
		} else if (init_param->ref_in_clks[i].mode == AD9545_DIFFERENTIAL){
			dev->ref_in_clks[i].d_conf = init_param->ref_in_clks[i].d_conf;
		} else {
			ret = -EINVAL;
			goto out_fail;
		}
		dev->ref_in_clks[i].r_div_ratio = init_param->ref_in_clks[i].r_div_ratio;
		dev->ref_in_clks[i].d_tol_ppb = init_param->ref_in_clks[i].d_tol_ppb;

		for (i = 0; i < ARRAY_SIZE(ad9545_hyst_scales_bp); i++) { // FIXME: Correct this
			if (ad9545_hyst_scales_bp[i] == init_param->ref_in_clks[i].monitor_hyst_scale) {
				dev->ref_in_clks[i].monitor_hyst_scale = i;
				break;
			}
		}

		if (i == ARRAY_SIZE(ad9545_hyst_scales_bp)) {
			ret = -EINVAL;
			goto out_fail;
		}

		dev->ref_in_clks[i].valid_t_ms = init_param->ref_in_clks[i].valid_t_ms;
		dev->ref_in_clks[i].freq_thresh_ps = init_param->ref_in_clks[i].freq_thresh_ps;
		dev->ref_in_clks[i].phase_thresh_ps = init_param->ref_in_clks[i].phase_thresh_ps;

		// clk = devm_clk_get(st->dev, ad9545_ref_clk_names[ref_ind]);// TODO: FIND SIMILAR
		// if (IS_ERR(clk)) {
		// 	ret = PTR_ERR(clk);
		// 	goto out_fail;
		// }

		// st->ref_in_clks[ref_ind].parent_clk = clk;
	}

out_fail:
	return ret;
}

static int ad9545_parse_pll_profiles(struct ad9545_dev *dev, struct ad9545_init_param *init_param, uint32_t addr)
{
	int ret = 0;

	/* parse DPLL profiles */
	uint32_t profile_addr;
	for (profile_addr = 0; profile_addr < AD9545_MAX_DPLL_PROFILES; profile_addr++) {
		if (!init_param->pll_clks[addr].profiles[profile_addr].en)
			continue;

		dev->pll_clks[addr].profiles[profile_addr].en = true;
		dev->pll_clks[addr].profiles[profile_addr].address = init_param->pll_clks[addr].profiles[profile_addr].address;
		dev->pll_clks[addr].profiles[profile_addr].loop_bw_uhz = init_param->pll_clks[addr].profiles[profile_addr].loop_bw_uhz;
		dev->pll_clks[addr].fast_acq_trigger_mode = init_param->pll_clks[addr].fast_acq_trigger_mode;
		dev->pll_clks[addr].profiles[profile_addr].fast_acq_excess_bw = init_param->pll_clks[addr].profiles[profile_addr].fast_acq_excess_bw;
		dev->pll_clks[addr].profiles[profile_addr].fast_acq_timeout_ms = init_param->pll_clks[addr].profiles[profile_addr].fast_acq_timeout_ms;
		dev->pll_clks[addr].profiles[profile_addr].fast_acq_settle_ms = init_param->pll_clks[addr].profiles[profile_addr].fast_acq_settle_ms;
		dev->pll_clks[addr].profiles[profile_addr].priority = init_param->pll_clks[addr].profiles[profile_addr].priority;

		if (init_param->pll_clks[addr].profiles[profile_addr].tdc_source > 5) {
			ret = -EINVAL;
			goto out_fail;
		}
		dev->pll_clks[addr].profiles[profile_addr].tdc_source = init_param->pll_clks[addr].profiles[profile_addr].tdc_source;
	}

out_fail:
	return ret;
}

static int32_t ad9545_parse_plls(struct ad9545_dev *dev, struct ad9545_init_param *init_param)
{
	uint32_t addr;
	int ret = 0;
	for (addr = 0; addr < NO_OS_ARRAY_SIZE(init_param->pll_clks); addr++) {
		if (!init_param->pll_clks[addr].pll_used)
			continue;
		dev->pll_clks[addr].pll_used = true;
		dev->pll_clks[addr].address = addr;
		dev->pll_clks[addr].slew_rate_limit_ps = init_param->pll_clks[addr].slew_rate_limit_ps;
		dev->pll_clks[addr].internal_zero_delay = init_param->pll_clks[addr].internal_zero_delay;

		if (init_param->pll_clks[addr].internal_zero_delay_source >= ARRAY_SIZE(ad9545_out_clk_names)) {
			pr_err("Invalid zero-delay fb path: %u.\n", init_param->pll_clks[addr].internal_zero_delay_source);
			ret = -EINVAL;
			goto out_fail;
		}
		dev->pll_clks[addr].internal_zero_delay_source = init_param->pll_clks[addr].internal_zero_delay_source;
		
		if (init_param->pll_clks[addr].internal_zero_delay_source_rate_hz >= AD9545_MAX_ZERO_DELAY_RATE) {
			pr_err("Invalid zero-delay output rate: %u.\n", init_param->pll_clks[addr].internal_zero_delay_source_rate_hz);
			ret = -EINVAL;
			goto out_fail;
		}
		dev->pll_clks[addr].internal_zero_delay_source_rate_hz = init_param->pll_clks[addr].internal_zero_delay_source_rate_hz;

		ret = ad9545_parse_dt_pll_profiles(dev, init_param, addr);
		if (ret)
			goto out_fail;
	}
out_fail:
	return ret;
}

static int32_t ad9545_parse_outputs(struct ad9545_dev *dev, struct ad9545_init_param *init_param)
{
	int out_ind;
	
	for (out_ind = 0; out_ind < NO_OS_ARRAY_SIZE(init_param->out_clks); out_ind++) {
		if (!init_param->out_clks[out_ind].output_used)
			continue;

		dev->out_clks[out_ind].output_used = true;
		dev->out_clks[out_ind].address = out_ind;
		dev->out_clks[out_ind].source_current = init_param->out_clks[out_ind].source_current;
		dev->out_clks[out_ind].source_ua = init_param->out_clks[out_ind].source_ua;
		dev->out_clks[out_ind].output_mode = init_param->out_clks[out_ind].output_mode;
	}
	return 0;
}

static int32_t ad9545_parse_ncos(struct ad9545_dev *dev, struct ad9545_init_param *init_param)
{
	int addr;
	
	for (addr = 0; addr < NO_OS_ARRAY_SIZE(init_param->aux_nco_clks); addr++) {
		if (!init_param->aux_nco_clks[addr].nco_used)
			continue;

		dev->aux_nco_clks[addr].nco_used = true;
		dev->aux_nco_clks[addr].address = addr;
		dev->aux_nco_clks[addr].dev = dev;
		dev->aux_nco_clks[addr].freq_thresh_ps = init_param->aux_nco_clks[addr].freq_thresh_ps;
		dev->aux_nco_clks[addr].phase_thresh_ps = init_param->aux_nco_clks[addr].phase_thresh_ps;
	}

	return 0;
}

static int32_t ad9545_parse_tdcs(struct ad9545_dev *dev, struct ad9545_init_param *init_param)
{
	int addr, ret = 0;
	
	for (addr = 0; addr < NO_OS_ARRAY_SIZE(init_param->aux_tdc_clks); addr++) {
		if (!init_param->aux_tdc_clks[addr].tdc_used)
			continue;

		dev->aux_tdc_clks[addr].tdc_used = true;
		dev->aux_tdc_clks[addr].address = addr;
		dev->aux_tdc_clks[addr].dev = dev;

		if (init_param->aux_tdc_clks[addr].pin_nr >= NO_OS_ARRAY_SIZE(ad9545_ref_m_clk_names)) {
			pr_err("Invalid Mx pin-nr: %d", init_param->aux_tdc_clks[addr].pin_nr);
			ret = -EINVAL;
			goto out_fail;
		}

		dev->aux_tdc_clks[addr].pin_nr = init_param->aux_tdc_clks[addr].pin_nr;
	}

out_fail:
	return ret;
}

static int32_t ad9545_parse_aux_dpll(struct ad9545_dev *dev, struct ad9545_init_param *init_param)
{
	if (!init_param->aux_dpll_clk.dpll_used)
		return 0;

	dev->aux_dpll_clk.dpll_used = true;
	dev->aux_dpll_clk.dev = dev;
	dev->aux_dpll_clk.rate_change_limit = init_param->aux_dpll_clk.rate_change_limit;
	dev->aux_dpll_clk.source = init_param->aux_dpll_clk.source;
	dev->aux_dpll_clk.loop_bw_mhz = init_param->aux_dpll_clk.loop_bw_mhz;
	
	return 0;
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 *		       parameters.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9545_init(struct ad9545_dev **device,
		     struct ad9545_init_param init_param)
{
	struct ad9545_dev	*dev;
	struct no_os_clk_desc **clocks = NULL;
	struct no_os_clk_init_param clk_init;
	int32_t ret;

	dev = (struct ad9545_dev *)no_os_malloc(sizeof(*dev));
	if (!dev)
		goto error;

	dev->comm_type = init_param.comm_type;
	if (dev->comm_type == SPI) {
		/* SPI */
		ret = no_os_spi_init(&dev->spi_desc, &init_param.spi_init);
		if (ret < 0)
			goto error;

		dev->reg_read = ad9545_spi_reg_read;
		dev->reg_write = ad9545_spi_reg_write;
		dev->reg_read_multiple = ad9545_spi_reg_read_multiple;
		dev->reg_write_multiple = ad9545_spi_reg_write_multiple;
	} else { /* I2C */
		ret = no_os_i2c_init(&dev->i2c_desc, &init_param.i2c_init);
		if (ret < 0)
			goto error;

		dev->reg_read = ad9545_i2c_reg_read;
		dev->reg_write = ad9545_i2c_reg_write;
		dev->reg_read_multiple = ad9545_i2c_reg_read_multiple;
		dev->reg_write_multiple = ad9545_i2c_reg_write_multiple;
	}

	/* Check device */
	ret = ad9545_check_id(dev);
	if (ret < 0)
		goto error;

	/* Device settings */
	/* Parse Init Inputs */
	ret = ad9545_parse_inputs(dev, &init_param);
	/* Parse PLLS */
	ret = ad9545_parse_plls(dev, &init_param);
	/* Parse Outputs */
	ret = ad9545_parse_outputs(dev, &init_param);
	/* Parse NCOS*/
	ret = ad9545_parse_ncos(dev, &init_param);
	/* Parse TDCS*/
	ret = ad9545_parse_tdcs(dev, &init_param);
	/* Parse DPLL */
	ret = ad9545_parse_aux_dpll(dev, &init_param);
	if (!ret) {
		*device = dev;
		printf("ad9545 successfully initialized\n");
		no_os_mdelay(1000);
		return ret;
	}
error:
	printf("ad9545 initialization error (%d)\n", ret);
	no_os_free(dev);
	no_os_mdelay(1000);
	return ret;
}

static int ad9545_sys_clk_setup(struct ad9545_dev *dev)
{
	uint64_t ref_freq_milihz;
	uint32_t stability_timer;
	uint64_t regval64;
	uint32_t regval;
	uint8_t div_ratio;
	uint32_t fosc;
	int ret;
	uint8_t val;
	uint32_t fs;
	int i;

	/*
	 * System frequency must be between 2250 MHz and 2415 MHz.
	 * fs = fosc * K / j
	 * K - feedback divider ratio [4, 255]
	 * j = 1/2 if frequency doubler is enabled
	 */
	fosc = NO_OS_DIV_ROUND_UP(dev->sys_clk.ref_freq_hz, 1000000);

	if (dev->sys_clk.sys_clk_freq_doubler)
		fosc *= 2;

	div_ratio = 0;
	for (i = 4; i < 256; i++) {
		fs = i * fosc;

		if (fs > 2250 && fs < 2415) {
			div_ratio = i;
			break;
		}
	}

	if (!div_ratio) {
		pr_err("No feedback divider ratio for sys clk PLL found.\n");
		return -EINVAL;
	}

	dev->sys_clk.sys_freq_hz = dev->sys_clk.ref_freq_hz * div_ratio;
	if (dev->sys_clk.sys_clk_freq_doubler)
		dev->sys_clk.sys_freq_hz *= 2;

	ret = ad9545_write_reg(dev, AD9545_SYS_CLK_FB_DIV, div_ratio);
	if (ret < 0)
		return ret;

	/* enable crystal maintaining amplifier */
	val = 0;
	if (dev->sys_clk.sys_clk_crystal)
		val |= BIT(3);

	if (dev->sys_clk.sys_clk_freq_doubler)
		val |= BIT(0);

	ret = ad9545_write_reg(dev, AD9545_SYS_CLK_INPUT, val);
	if (ret < 0)
		return ret;

	/* write reference frequency provided at XOA, XOB in milliherz */
	ref_freq_milihz = no_os_mul_u32_u32(dev->sys_clk.ref_freq_hz, 1000);
	regval64 = ref_freq_milihz;

	ret = ad9545_write_reg_multiple(dev, AD9545_SYS_CLK_REF_FREQ, &regval64, 5); //TODO: Check this
	if (ret < 0)
		return ret;

	stability_timer = no_os_field_prep(AD9545_SYS_CLK_STABILITY_PERIOD_MASK,
				     AD9545_SYS_CLK_STABILITY_MS);
	regval = stability_timer;
	return ad9545_write_reg_multiple(dev, AD9545_STABILITY_TIMER,
				 &regval, 3);
}

static int ad9545_input_refs_setup(struct ad9545_dev *dev)
{
	// struct clk_init_data init[4] = {0}; // TODO: See this
	struct no_os_clk_init_param init[4] = {0}; // TODO: See this
	uint32_t regval;
	uint64_t regval64;
	uint64_t period_es;
	int ret;
	uint32_t val;
	uint8_t reg;
	int i;

	/* configure input references */
	for (i = 0; i < NO_OS_ARRAY_SIZE(dev->ref_in_clks); i += 2) {
		if (dev->ref_in_clks[i].mode == AD9545_DIFFERENTIAL) {
			reg = NO_OS_BIT(0);
			reg |= no_os_field_prep(AD9545_REF_CTRL_DIF_MSK, dev->ref_in_clks[i].d_conf);
		} else {
			reg = 0;
			reg |= no_os_field_prep(AD9545_REF_CTRL_REFA_MSK, dev->ref_in_clks[i].s_conf);
			reg |= no_os_field_prep(AD9545_REF_CTRL_REFAA_MSK, dev->ref_in_clks[i + 1].s_conf);
		}

		ret = ad9545_write_reg(dev, AD9545_REF_A_CTRL + i * 2, reg);
		if (ret < 0)
			return ret;
	}

	for (i = 0; i < NO_OS_ARRAY_SIZE(dev->ref_in_clks); i++) {
		if (!dev->ref_in_clks[i].ref_used)
			continue;

		/* configure refs r dividers */
		ret = ad9545_set_r_div(dev, dev->ref_in_clks[i].r_div_ratio, i);
		if (ret < 0)
			return ret;

		/* write nominal period in attoseconds */
		period_es = 1000000000000000000ULL;
		val = no_os_clk_recalc_rate(dev->ref_in_clks[i].parent_clk, &val);
		if (!val)
			return -EINVAL;

		period_es = NO_OS_DIV_U64(period_es, val);

		regval = dev->ref_in_clks[i].d_tol_ppb;
		ret = ad9545_write_reg_multiple(dev, AD9545_REF_X_OFFSET_LIMIT(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval64 = period_es;
		ret = ad9545_write_reg_multiple(dev, AD9545_REF_X_PERIOD(i), &regval64, 8);
		if (ret < 0)
			return ret;

		ret = ad9545_write_reg(dev, AD9545_REF_X_MONITOR_HYST(i),
				   dev->ref_in_clks[i].monitor_hyst_scale);
		if (ret < 0)
			return ret;

		regval = dev->ref_in_clks[i].freq_thresh_ps;
		ret = ad9545_write_reg_multiple(dev, AD9545_SOURCEX_FREQ_THRESH(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = dev->ref_in_clks[i].valid_t_ms;
		ret = ad9545_write_reg_multiple(dev, AD9545_REF_X_VALID_TIMER(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = dev->ref_in_clks[i].phase_thresh_ps;
		ret = ad9545_write_reg_multiple(dev, AD9545_SOURCEX_PHASE_THRESH(i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = dev->ref_in_clks[i].freq_lock_fill_rate;
		if (regval) {
			ret = ad9545_write_reg(dev, AD9545_REF_X_FREQ_LOCK_FILL(i), regval);
			if (ret < 0)
				return ret;
		}

		regval = dev->ref_in_clks[i].freq_lock_drain_rate;
		if (regval) {
			ret = ad9545_write_reg(dev, AD9545_REF_X_FREQ_LOCK_DRAIN(i), regval);
			if (ret < 0)
				return ret;
		}

		regval = dev->ref_in_clks[i].phase_lock_fill_rate;
		if (regval) {
			ret = ad9545_write_reg(dev, AD9545_REF_X_PHASE_LOCK_FILL(i), regval);
			if (ret < 0)
				return ret;
		}

		regval = dev->ref_in_clks[i].phase_lock_drain_rate;
		if (regval) {
			ret = ad9545_write_reg(dev, AD9545_REF_X_PHASE_LOCK_DRAIN(i), regval);
			if (ret < 0)
				return ret;
		}

		init[i].name = ad9545_in_clk_names[i];
		init[i].platform_ops = &ad9545_in_clk_ops;
		// init[i].parent_names = &ad9545_ref_clk_names[i];
		init[i].hw_ch_num = i;

		// dev->ref_in_clks[i].hw.init = &init[i];
		// ret = devm_clk_hw_register(st->dev, &st->ref_in_clks[i].hw);
		ret = no_os_clk_init(&dev->ref_in_clks[i].hw, &init);
		if (ret < 0)
			return ret;
	}

	/* disable unused references */
	reg = 0;
	for (i = 0; i < NO_OS_ARRAY_SIZE(dev->ref_in_clks); i++) {
		if (!dev->ref_in_clks[i].ref_used)
			reg |= (1 << i);
	}

	return ad9545_write_reg(dev, AD9545_POWER_DOWN_REF, reg);
}


static int ad9545_setup(struct ad9545_dev *dev)
{
	int ret;
	uint32_t val;
	int i;

	// TODO: Check this below
	ret = ad9545_write_mask(dev, AD9545_CONFIG_0, AD9545_RESET_REGS, AD9545_RESET_REGS);
	if (ret < 0)
		return ret;

	ret = ad9545_sys_clk_setup(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_input_refs_setup(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_aux_ncos_setup(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_calib_system_clock(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_aux_tdcs_setup(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_aux_dpll_setup(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_calib_aplls(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_io_update(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_plls_setup(dev);
	if (ret < 0)
		return ret;

	ret = ad9545_outputs_setup(dev);
	if (ret < 0)
		return ret;

	// TODO: Check this too
	// ret = devm_of_clk_add_hw_provider(st->dev, ad9545_clk_hw_twocell_get,
	// 				  &st->clks[AD9545_CLK_OUT]);
	// if (ret < 0)
	// 	return ret;

	ret = ad9545_calib_aplls(dev);
	if (ret < 0)
		return ret;

	/* check locks */
	ret = ad9545_read_reg(dev, AD9545_PLL_STATUS, &val);
	for (i = 0; i < ARRAY_SIZE(dev->pll_clks); i++)
		if (dev->pll_clks[i].pll_used && !AD9545_PLLX_LOCK(i, val))
			pr_warning("PLL%d unlocked.\n", i);

	if (dev->aux_dpll_clk.dpll_used) {
		ret = ad9545_read_reg(dev, AD9545_MISC, &val);
		if (ret < 0)
			return ret;

		if (!(val & AD9545_AUX_DPLL_LOCK_MSK))
			pr_warning("Aux DPLL unlocked.\n");

		if (val & AD9545_AUX_DPLL_REF_FAULT)
			pr_warning("Aux DPLL reference fault.\n");
	}

	return 0;
}
