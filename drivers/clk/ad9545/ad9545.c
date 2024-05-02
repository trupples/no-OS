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

// FIXME: See how to calculate this
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

// FIXME: See how to calculate this
static int32_t ad9545_in_clk_recalc_rate(struct no_os_clk_desc *hw, uint64_t *rate)
{
	struct ad9545_dev *dev = hw->dev_desc;
	uint32_t div;
	uint64_t parent_rate;
	int ret;

	ret = ad9545_get_r_div(dev, hw->hw_ch_num, &div);
	if (ret < 0) {
		pr_err("Could not read r div value.");
		return -EINVAL;
	}
	ret = no_os_clk_recalc_rate(dev->ref_in_clks[hw->hw_ch_num].parent_clk,
			      &parent_rate);
	if (ret < 0)
		return -EINVAL;
	*rate = NO_OS_DIV_ROUND_CLOSEST(parent_rate, div);
	return 0;
}

static int ad9545_tdc_source_valid(struct ad9545_pll_clk *clk, unsigned int tdc_source)
{
	unsigned int regval;
	int ret;

	if (tdc_source >= AD9545_MAX_REFS) {
		ret = ad9545_read_reg(clk->dev, AD9545_MISC, &regval);
		if (ret < 0)
			return ret;

		if (tdc_source == AD9545_MAX_REFS)
			return !(regval & AD9545_MISC_AUX_NC0_ERR_MSK);
		else
			return !(regval & AD9545_MISC_AUX_NC1_ERR_MSK);
	} else {
		ret = ad9545_read_reg(clk->dev, AD9545_REFX_STATUS(tdc_source), &regval);
		if (ret < 0)
			return ret;

		return !!(regval & AD9545_REFX_VALID_MSK);
	}
}

static uint8_t ad9545_pll_get_parent(struct no_os_clk_desc *hw)
{
	struct ad9545_dev *dev = hw->dev_desc;
	struct ad9545_pll_clk *clk = &dev->pll_clks[hw->hw_ch_num];
	struct ad9545_dpll_profile *profile;
	uint8_t best_prio = 0xFF;
	uint8_t best_parent;
	int ret;
	int i;

	ret = ad9545_io_update(dev);
	if (ret < 0)
		return ret;

	/*
	 * A DPLL will pick a parent clock depending
	 * on the priorities and if it is a valid timestamp source.
	 */
	for (i = 0; i < AD9545_MAX_DPLL_PROFILES; i++) {
		profile = &clk->profiles[i];
		if (!profile->en)
			continue;

		ret = ad9545_tdc_source_valid(clk, profile->tdc_source);
		if (ret < 0)
			return clk->num_parents;

		if (ret > 0 && profile->priority < best_prio) {
			best_prio = profile->priority;
			best_parent = profile->parent_index;
		}
	}

	if (best_prio != 0xFF)
		return best_parent;

	return clk->num_parents;
}

static int ad9545_get_q_div(struct ad9545_dev *dev, int addr, uint32_t *q_div)
{
	uint32_t regval;
	int ret;

	ret = ad9545_read_reg_multiple(dev, AD9545_QX_DIV(addr), &regval, 4);
	if (ret < 0)
		return ret;

	*q_div = regval;

	return 0;
}

static int32_t ad9545_pll_clk_recalc_rate(struct no_os_clk_desc *hw, uint64_t *rate)
{
	// struct ad9545_pll_clk *clk = to_pll_clk(hw);
	struct ad9545_dev *dev = hw->dev_desc;
	struct ad9545_pll_clk *clk = &dev->pll_clks[hw->hw_ch_num];
	unsigned long output_rate;
	uint64_t parent_rate;
	uint32_t regval;
	uint32_t frac;
	uint32_t mod;
	int ret;
	uint32_t m;
	uint32_t n;
	int i;

	m = 0;
	ret = ad9545_read_reg(hw->dev_desc, AD9545_APLLX_M_DIV(hw->hw_ch_num), &m);
	if (ret < 0)
		return ret;

	/*
	 * If no ref is valid, pll will run in free run mode.
	 * At this point the NCO of the DPLL will output a free run frequency
	 * thus the output frequency of the PLL block will be:
	 * f NCO * M-div / 2
	 */
	i = ad9545_pll_get_parent(hw);
	if (i == clk->num_parents)
		return DIV_ROUND_UP(clk->free_run_freq, 2) * m;

	ret = no_os_clk_recalc_rate(clk->parents[i], &parent_rate);
	if (ret < 0)
		return ret;

	ret = ad9545_read_reg_multiple(clk->dev, AD9545_DPLLX_N_DIV(clk->address, i), &regval, 4);
	if (ret < 0)
		return ret;

	n = regval + 1;

	regval = 0;
	ret = ad9545_read_reg_multiple(clk->dev, AD9545_DPLLX_FRAC_DIV(clk->address, i), &regval, 3);
	if (ret < 0)
		return ret;

	frac = regval;

	regval = 0;
	ret = ad9545_read_reg_multiple(clk->dev, AD9545_DPLLX_MOD_DIV(clk->address, i), &regval, 3);
	if (ret < 0)
		return ret;

	mod = regval;

	if (clk->internal_zero_delay) {
		int output_nr = clk->internal_zero_delay_source;
		uint32_t q_div;

		ret = ad9545_get_q_div(clk->dev, output_nr, &q_div);
		if (ret < 0)
			return ret;

		if (!m)
			return 0;

		if (clk->profiles[i].fb_tagging) {
			ret = ad9545_read_reg_multiple(clk->dev,
					       ad9545_out_regs[output_nr].modulation_counter_reg,
					       &regval, 4);
			if (ret < 0)
				return ret;

			n = regval;
		} else {
			ret = ad9545_read_reg_multiple(clk->dev,
					       AD9545_DPLLX_HITLESS_N(clk->address, i),
					       &regval, 4);
			if (ret < 0)
				return ret;

			n = regval + 1;
		}

		output_rate = 2 * q_div * parent_rate * n;
	} else {
		if (!mod)
			return 0;

		/* Output rate of APLL = parent_rate * (N + (Frac / Mod)) * M */
		// output_rate = mul_u64_u32_div(frac * parent_rate, m, mod);
		output_rate = no_os_mul_u64_u32_shr(frac * parent_rate, m, 0);
		output_rate = no_os_div_u64(output_rate, mod);
		output_rate += parent_rate * n * m;

	}
	*rate = output_rate / 2;
	return 0;
}

static uint32_t ad9545_calc_m_div(unsigned long rate)
{
	uint32_t m_div;

	/*
	 * PFD of APLL has input frequency limits in 162 - 350 Mghz range.
	 * Use APLL to upconvert this freq to Ghz range.
	 */
	m_div = no_os_div_u64(rate, ad9545_apll_pfd_rate_ranges_hz[0] / 2 +
			ad9545_apll_pfd_rate_ranges_hz[1] / 2);
	m_div = no_os_clamp_t(uint8_t, m_div, AD9545_APLL_M_DIV_MIN, AD9545_APLL_M_DIV_MAX);

	return m_div;
}

static int ad9545_calc_ftw(struct ad9545_pll_clk *clk, uint32_t freq, uint64_t *tuning_word)
{
	uint64_t ftw = 1;
	uint32_t ftw_frac;
	uint32_t ftw_int;

	/*
	 * In case of unlock event the DPLL will go in open-loop mode and output
	 * the freq given by the freerun tuning word.
	 * DPLLx Freerun TW = (2 ^ 48) × (f NCO /f System )
	 */
	// ftw = mul_u64_u32_div(ftw << 48, freq, clk->dev->sys_clk.sys_freq_hz); // FIXME:
	ftw = no_os_mul_u64_u32_shr(ftw << 48, freq, 0); // FIXME:
	ftw = no_os_div64_u64_rem(ftw, clk->dev->sys_clk.sys_freq_hz, NULL);
	/*
	 * Check if FTW is valid:
	 * (2 ^ 48) / FTW = INT.FRAC where:
	 * 7 ≤ INT ≤ 13 and 0.05 ≤ FRAC ≤ 0.95
	 */
	ftw_int = no_os_div64_u64_rem(1ULL << 48, ftw, NULL);
	if (ftw_int < 7 || ftw_int > 13)
		return -EINVAL;

	no_os_div64_u64_rem(no_os_div64_u64_rem(100 * (1ULL << 48), ftw, NULL), 100, &ftw_frac);
	if (ftw_frac < 5 || ftw_frac > 95)
		return -EINVAL;

	*tuning_word = ftw;

	return 0;
}

static uint64_t ad9545_calc_pll_params(struct ad9545_pll_clk *clk, unsigned long rate,
				  unsigned long parent_rate, bool zero_delay, uint32_t *m, uint32_t *n,
				  unsigned long *frac, unsigned long *mod, int profile_nr)
{
	uint32_t min_dpll_n_div;
	uint64_t output_rate;
	uint32_t dpll_n_div;
	uint32_t m_div;
	uint64_t den;
	uint64_t num;

	if (zero_delay) {
		*frac = 0;
		*mod = 1;

		return ad9545_calc_pll_zero_delay_params(clk, rate, parent_rate, m, n, profile_nr);
	}

	/* half divider at output requires APLL to generate twice the frequency demanded */
	rate *= 2;

	m_div = ad9545_calc_m_div(rate);

	/*
	 * If N + FRAC / MOD = rate / (m_div * parent_rate)
	 * and N = [rate / (m_div * past_rate)]:
	 * We get: FRAC/MOD = (rate / (m_div * parent_rate)) - N
	 */
	dpll_n_div = no_os_div64_u64_rem(rate, parent_rate * m_div, NULL);
	// dpll_n_div = div64_u64(rate, parent_rate * m_div);

	/*
	 * APLL has to be able to satisfy output freq bounds
	 * thus output of DPLL has a lower bound
	 */
	min_dpll_n_div = no_os_div64_u64_rem(ad9545_apll_rate_ranges_hz[clk->address][0],
				 AD9545_APLL_M_DIV_MAX * parent_rate, NULL);
	dpll_n_div = no_os_clamp_t(uint32_t, dpll_n_div, min_dpll_n_div, AD9545_DPLL_MAX_N);

	num = rate - (dpll_n_div * m_div * parent_rate);
	den = m_div * parent_rate;

	no_os_rational_best_approximation(num, den, AD9545_DPLL_MAX_FRAC, AD9545_DPLL_MAX_MOD, frac, mod);
	*m = m_div;
	*n = dpll_n_div;

	// output_rate = mul_u64_u32_div(*frac * parent_rate, m_div, *mod);
	output_rate = no_os_mul_u64_u32_shr(*frac * parent_rate, m_div, 0); // FIXME:
	output_rate = no_os_div64_u64_rem(output_rate, *mod, NULL);
	output_rate += parent_rate * dpll_n_div * m_div;

	return (uint32_t)NO_OS_DIV_ROUND_CLOSEST(output_rate, 2);
}

// static long ad9545_pll_clk_round_rate(struct clk_hw *hw, unsigned long rate,
// 				      unsigned long *parent_rate)
static int ad9545_pll_clk_round_rate(struct no_os_clk_desc *hw,
				       uint64_t rate, uint64_t *rounded_rate)
{
	// struct ad9545_pll_clk *clk = to_pll_clk(hw);
	struct ad9545_dev *dev = hw->dev_desc;
	struct ad9545_pll_clk *clk = &dev->pll_clks[hw->hw_ch_num];
	uint64_t parent_rate;
	unsigned long frac;
	unsigned long mod;
	bool zero_delay;
	int ret;
	uint64_t ftw;
	int i;
	uint32_t m;
	uint32_t n;

	clk->rate_requested_hz = rate;

	/* if no ref is valid, check if requested rate can be set in free run mode */
	i = ad9545_pll_get_parent(hw);
	if (i == clk->num_parents) {
		/* in free run mode output freq is given by f NCO * m / 2 */
		m = ad9545_calc_m_div(rate * 2);
		ret = ad9545_calc_ftw(clk,  div_u64(rate * 2, m), &ftw);
		if (ret < 0)
			return ret;

		return rate;
	}
	ret = no_os_clk_recalc_rate(clk->parents[i], &parent_rate);
	zero_delay = clk->internal_zero_delay;

	return ad9545_calc_pll_params(clk, rate, parent_rate, zero_delay, &m, &n, &frac, &mod, i);
}

static int ad9545_pll_set_feedback_tagging(struct ad9545_pll_clk *clk, unsigned long rate,
					   unsigned long parent_rate, unsigned int profile_nr,
					   uint32_t *n)
{
	unsigned long out_rate_hz;
	int fb_output_nr;
	uint32_t regval;
	uint32_t q_div;
	int ret;
	uint8_t val;

	clk->profiles[profile_nr].fb_tagging = true;

	val = no_os_field_prep(AD9545_TAG_MODE_MSK, AD9545_FB_PATH_TAG) | AD9545_BASE_FILTER_MSK;
	ret = ad9545_write_mask(clk->dev, AD9545_DPLLX_FB_MODE(clk->address, profile_nr),
				 AD9545_BASE_FILTER_MSK | AD9545_TAG_MODE_MSK, val);
	if (ret < 0)
		return ret;

	fb_output_nr = clk->internal_zero_delay_source;
	out_rate_hz = clk->internal_zero_delay_source_rate_hz;

	if (!out_rate_hz)
		return 0;

	q_div = NO_OS_DIV_ROUND_UP(rate, out_rate_hz);

	/* Limit tagging frequency via N-Div */
	*n = NO_OS_DIV_ROUND_UP(out_rate_hz, AD9545_IN_MAX_TDC_FREQ_HZ);
	regval = *n - 1;
	ret = ad9545_write_reg_multiple(clk->dev, AD9545_DPLLX_HITLESS_N(clk->address, profile_nr),
				&regval, 4);
	if (ret < 0)
		return ret;

	/* determine modulation period */
	regval = NO_OS_DIV_ROUND_CLOSEST_ULL(rate, q_div * parent_rate);
	ret = ad9545_write_reg_multiple(clk->dev,
				ad9545_out_regs[fb_output_nr].modulation_counter_reg,
				&regval, 4);
	if (ret < 0)
		return ret;

	return ad9545_write_mask(clk->dev, ad9545_out_regs[fb_output_nr].modulator_reg,
				  AD9545_MODULATOR_EN, AD9545_MODULATOR_EN);
}

static int ad9545_set_freerun_freq(struct ad9545_pll_clk *clk, uint32_t freq)
{
	uint64_t regval;
	uint64_t ftw;
	int ret;

	if (!freq)
		return 0;

	ret = ad9545_calc_ftw(clk, freq, &ftw);
	if (ret < 0)
		return ret;

	regval = ftw;
	ret = ad9545_write_reg_multiple(clk->dev, AD9545_DPLLX_FTW(clk->address), &regval, 6);
	if (ret < 0)
		return ret;

	clk->free_run_freq = freq;
	return ad9545_io_update(clk->dev);
}


// static int ad9545_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
static int ad9545_pll_set_rate(struct no_os_clk_desc *hw, uint64_t rate)
{
	struct ad9545_dev *dev = hw->dev_desc;
	struct ad9545_pll_clk *clk = &dev->pll_clks[hw->hw_ch_num];
	// struct ad9545_pll_clk *clk = to_pll_clk(hw);
	uint64_t parent_rate;
	unsigned long out_rate;
	unsigned long frac;
	unsigned long mod;
	int fb_output_nr;
	bool zero_delay;
	uint32_t regval;
	uint32_t q_div;
	int ret;
	uint32_t m;
	uint32_t n;
	int i;

	/*
	 * When setting a PLL rate, precalculate params for all enabled profiles.
	 * At this point there may or may not be a valid reference.
	 */

	if (!rate)
		return -EINVAL;

	for (i = 0; i < clk->num_parents; i++) {
		ret = no_os_clk_recalc_rate(clk->parents[i], &parent_rate);
		if (ret < 0)
			return -EINVAL;

		zero_delay = clk->internal_zero_delay;
		out_rate = ad9545_calc_pll_params(clk, rate, parent_rate, zero_delay, &m, &n, &frac,
						  &mod, i);

		/*
		 * Feedback under 2 kHz needs to be from tagged
		 * sources when hitless mode is active.
		 */
		clk->profiles[i].fb_tagging = false;
		if (zero_delay && parent_rate < 2000) {
			ret = ad9545_pll_set_feedback_tagging(clk, rate, parent_rate, i, &n);
		} else if (zero_delay) {
			ret = ad9545_write_reg(clk->dev,
					   AD9545_DPLLX_FB_MODE(clk->address, i),
					   no_os_field_prep(AD9545_EN_HITLESS_MSK, 1));
		} else {
			ret = ad9545_write_reg(clk->dev,
					   AD9545_DPLLX_FB_MODE(clk->address, i), 0);
		}

		if (ret < 0)
			return ret;

		if (zero_delay) {
			/*
			 * In zero delay mode: Phase Buildout N needs to be programmed to:
			 * N BUILDOUT = N HITLESS × 2 × Q/M
			 */

			fb_output_nr = clk->internal_zero_delay_source;
			if (!clk->internal_zero_delay_source_rate_hz)
				return 0;

			q_div = NO_OS_DIV_ROUND_UP(rate, clk->internal_zero_delay_source_rate_hz);
			regval = NO_OS_DIV_ROUND_UP(n * 2 * q_div, m) - 1;
		} else {
			regval = n - 1;
		}

		ret = ad9545_write_reg_multiple(clk->dev,
					AD9545_DPLLX_N_DIV(clk->address, i),
					&regval, 4);
		if (ret < 0)
			return ret;

		regval = n - 1;
		ret = ad9545_write_reg_multiple(clk->dev,
					AD9545_DPLLX_HITLESS_N(clk->address, i),
					&regval, 4);
		if (ret < 0)
			return ret;

		ret = ad9545_write_reg(clk->dev, AD9545_APLLX_M_DIV(clk->address), m);
		if (ret < 0)
			return ret;

		regval = frac;
		ret = ad9545_write_reg_multiple(clk->dev, AD9545_DPLLX_FRAC_DIV(clk->address, i),
					&regval, 3);
		if (ret < 0)
			return ret;

		regval = mod;
		ret = ad9545_write_reg_multiple(clk->dev, AD9545_DPLLX_MOD_DIV(clk->address, i),
					&regval, 3);
		if (ret < 0)
			return ret;
	}

	return ad9545_set_freerun_freq(clk, NO_OS_DIV_U64(rate * 2, m));
}

static const struct no_os_clk_platform_ops ad9545_in_clk_ops = {
	.clk_recalc_rate = ad9545_in_clk_recalc_rate,
	// .init = ad9545_in_clk_debug_init,
};
static const struct no_os_clk_platform_ops ad9545_pll_clk_ops = {
	.clk_recalc_rate = ad9545_pll_clk_recalc_rate,
	.clk_round_rate = ad9545_pll_clk_round_rate,
	.clk_set_rate = ad9545_pll_set_rate,
	// .get_parent = ad9545_pll_get_parent,
	// .debug_init = ad9545_pll_debug_init,
};

static int ad9545_get_nco_center_freq(struct ad9545_dev *dev, int addr, uint64_t *freq)
{
	uint64_t regval64;
	int ret;

	regval64 = 0;
	ret = ad9545_write_reg_multiple(dev, AD9545_NCOX_CENTER_FREQ(addr), &regval64, 7);
	if (ret < 0)
		return ret;

	*freq = regval64;

	return 0;
}

static int ad9545_get_nco_offset_freq(struct ad9545_dev *dev, int addr, uint32_t *freq)
{
	uint32_t regval;
	int ret;

	ret = ad9545_write_reg_multiple(dev, AD9545_NCOX_OFFSET_FREQ(addr), &regval, 4);
	if (ret < 0)
		return ret;

	*freq = regval;

	return 0;
}

static int ad9545_get_nco_freq(struct ad9545_dev *dev, int addr, uint64_t *freq)
{
	uint64_t center_freq;
	uint32_t offset_freq;
	int shift;
	int ret;

	ret = ad9545_get_nco_center_freq(dev, addr, &center_freq);
	if (ret < 0)
		return ret;

	ret = ad9545_get_nco_offset_freq(dev, addr, &offset_freq);
	if (ret < 0)
		return ret;

	/* align center frequency and offset frequency, and then add */
	shift = AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH;
	*freq = center_freq + ((uint64_t)offset_freq << shift);

	return 0;
}

#define NO_OS_BIT_ULL(x) ((uint64_t) 1 << (x))
static int ad9545_get_nco_freq_hz(struct ad9545_dev *dev, int addr, uint32_t *freq)
{
	uint64_t val64;
	uint32_t freq_int;
	int ret;

	ret = ad9545_get_nco_freq(dev, addr, &val64);
	if (ret < 0)
		return ret;

	freq_int = val64 >> AD9545_NCO_CENTER_FREQ_FRAC_WIDTH;

	if (val64 & NO_OS_BIT_ULL(AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - 1))
		freq_int++;

	*freq = freq_int;

	return 0;
}

static int32_t ad9545_nco_clk_recalc_rate(struct no_os_clk_desc *hw, uint64_t *rate)
{
	struct ad9545_dev *dev = hw->dev_desc;
	struct ad9545_aux_nco_clk *clk = &dev->aux_nco_clks[hw->hw_ch_num];
	uint32_t rate;
	int ret;

	ret = ad9545_get_nco_freq_hz(clk->dev, clk->address, &rate);
	if (ret < 0) {
		pr_err("Could not read NCO freq.");
		return 0;
	}

	return rate;
}

static int ad9545_nco_clk_round_rate(struct no_os_clk_desc *desc,
				       uint64_t rate, uint64_t *rounded_rate)
{
	return no_os_min(rate, (unsigned long)AD9545_NCO_FREQ_INT_MAX + 1);
}

static int ad9545_set_nco_center_freq(struct ad9545_dev *dev, int addr, uint64_t freq)
{
	uint64_t regval64;

	if (freq > AD9545_NCO_CENTER_FREQ_MAX)
		return -EINVAL;

	regval64 = freq;

	return ad9545_write_reg_multiple(dev, AD9545_NCOX_CENTER_FREQ(addr), &regval64, 7);
}

static int ad9545_set_nco_offset_freq(struct ad9545_dev *dev, int addr, uint32_t freq)
{
	uint32_t regval;

	regval = freq;

	return ad9545_write_reg_multiple(dev, AD9545_NCOX_OFFSET_FREQ(addr), &regval, 4);
}

static int ad9545_set_nco_freq_hz(struct ad9545_dev *dev, int addr, uint32_t freq)
{
	uint64_t center_freq;
	uint32_t center_freq_int, offset_freq, offset_freq_int;
	bool use_fractional;
	int ret;

	if (freq > AD9545_NCO_FREQ_INT_MAX + 1)
		return -EINVAL;

	use_fractional = (freq == AD9545_NCO_FREQ_INT_MAX + 1);
	if (use_fractional)
		freq--;

	if (freq <= AD9545_NCO_CENTER_FREQ_INT_MAX) {
		center_freq_int = freq;
		offset_freq_int = 0;
	} else {
		center_freq_int = AD9545_NCO_CENTER_FREQ_INT_MAX;
		offset_freq_int = freq - center_freq_int;
	}

	center_freq = no_os_field_prep(AD9545_NCO_CENTER_FREQ_INT_MSK, center_freq_int);
	if (use_fractional)
		center_freq |= NO_OS_BIT_ULL(AD9545_NCO_CENTER_FREQ_FRAC_WIDTH - 1);

	ret = ad9545_set_nco_center_freq(dev, addr, center_freq);
	if (ret < 0)
		return ret;

	offset_freq = no_os_field_prep(AD9545_NCO_OFFSET_FREQ_INT_MSK, offset_freq_int);
	if (use_fractional)
		offset_freq |= NO_OS_BIT(AD9545_NCO_OFFSET_FREQ_FRAC_WIDTH - 1);

	ret = ad9545_set_nco_offset_freq(dev, addr, offset_freq);
	if (ret < 0)
		return ret;

	return ad9545_io_update(dev);
}

static int ad9545_nco_clk_set_rate(struct no_os_clk_desc *hw, uint64_t rate)
{
	struct ad9545_dev *dev = hw->dev_desc;
	struct ad9545_aux_nco_clk *clk = &dev->aux_nco_clks[hw->hw_ch_num];

	return ad9545_set_nco_freq_hz(clk->dev, clk->address, rate);
}

static const struct no_os_clk_platform_ops ad9545_nco_clk_ops = {
	.clk_recalc_rate = ad9545_nco_clk_recalc_rate,
	.clk_round_rate = ad9545_nco_clk_round_rate,
	.clk_set_rate = ad9545_nco_clk_set_rate,
};

static unsigned long ad9545_tdc_clk_recalc_rate(struct no_os_clk_desc *hw, uint64_t *rate)
{
	struct ad9545_dev *dev = hw->dev_desc;
	struct ad9545_aux_tdc_clk *clk = &dev->aux_tdc_clks[hw->hw_ch_num];
	int ret;
	uint64_t parent_rate;
	uint32_t div;

	// FIXME: Find a way to do this!
	no_os_clk_recalc_rate(clk.);

	ret = ad9545_read_reg(clk->dev, AD9545_TDCX_DIV(clk->address), &div);
	if (ret < 0) {
		pr_err("Could not read TDC freq.");
		return ret;
	}

	div++;

	*rate = ((uint64_t)parent_rate, div);

	return 0;
}

static const struct no_os_clk_platform_ops ad9545_aux_tdc_clk_ops = {
	.clk_recalc_rate = ad9545_tdc_clk_recalc_rate,
	.clk_round_rate = ad9545_tdc_clk_round_rate,
	.clk_set_rate = ad9545_tdc_clk_set_rate,
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

		for (i = 0; i < NO_OS_ARRAY_SIZE(ad9545_hyst_scales_bp); i++) { // FIXME: Correct this
			if (ad9545_hyst_scales_bp[i] == init_param->ref_in_clks[i].monitor_hyst_scale) {
				dev->ref_in_clks[i].monitor_hyst_scale = i;
				break;
			}
		}

		if (i == NO_OS_ARRAY_SIZE(ad9545_hyst_scales_bp)) {
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

		dev->ref_in_clks[i].parent_clk = init_param->ref_in_clks[i].parent_clk;
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
