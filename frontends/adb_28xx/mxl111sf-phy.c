/*
 *  mxl111sf-phy.c - driver for the MaxLinear MXL111SF
 *
 *  Copyright (C) 2010 Michael Krufky <mkrufky@kernellabs.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "mxl111sf-phy.h"
#include "mxl111sf-reg.h"

int mxl111sf_init_tuner_demod(struct mxl111sf_state *state)
{
	struct mxl111sf_reg_ctrl_info mxl_111_overwrite_default_[] = {
		{0x07, 0xff, 0x0c},
		{0x58, 0xff, 0x9d},
		{0x09, 0xff, 0x00},
		{0x06, 0xff, 0x06},
		{0xc8, 0xff, 0x40}, /* ED_LE_WIN_OLD = 0 */
		{0x8d, 0x01, 0x01}, /* NEGATE_Q */
		{0x32, 0xff, 0xac}, /* DIG_RFREFSELECT = 12 */
		{0x42, 0xff, 0x43}, /* DIG_REG_AMP = 4 */
		{0x74, 0xff, 0xc4}, /* SSPUR_FS_PRIO = 4 */
		{0x71, 0xff, 0xe6}, /* SPUR_ROT_PRIO_VAL = 1 */
		{0x83, 0xff, 0x64}, /* INF_FILT1_THD_SC = 100 */
		{0x85, 0xff, 0x64}, /* INF_FILT2_THD_SC = 100 */
		{0x88, 0xff, 0xf0}, /* INF_THD = 240 */
		{0x6f, 0xf0, 0xb0}, /* DFE_DLY = 11 */
		{0x00, 0xff, 0x01}, /* Change to page 1 */
		{0x81, 0xff, 0x11}, /* DSM_FERR_BYPASS = 1 */
		{0xf4, 0xff, 0x07}, /* DIG_FREQ_CORR = 1 */
		{0xd4, 0x1f, 0x0f}, /* SPUR_TEST_NOISE_TH = 15 */
		{0xd6, 0xff, 0x0c}, /* SPUR_TEST_NOISE_PAPR = 12 */
		{0x00, 0xff, 0x00}, /* Change to page 0 */
		{0,    0,    0}
	};

	//mxl101sf
struct mxl111sf_reg_ctrl_info mxl_111_overwrite_default[] = {
  {0xC8, 0xFF, 0x40},  
  {0x8D, 0x01, 0x01},  
  {0x42, 0xFF, 0x33},  
  {0x71, 0xFF, 0x66},  
  {0x72, 0xFF, 0x01},  
  {0x73, 0xFF, 0x00}, 
  {0x74, 0xFF, 0xC2}, 
  {0x71, 0xFF, 0xE6},  
  {0x83, 0xFF, 0x64},  
  {0x85, 0xFF, 0x64},  
  {0x88, 0xFF, 0xF0},  
  {0x6F, 0xFF, 0xB7}, 
  {0x98, 0xFF, 0x40},  
  {0x99, 0xFF, 0x18}, 
  {0xE0, 0xFF, 0x00},  
  {0xE1, 0xFF, 0x10}, 
  {0xE2, 0xFF, 0x91}, 
  {0xE4, 0xFF, 0x3F}, 
  {0xB1, 0xFF, 0x00}, 
  {0xB2, 0xFF, 0x09}, 
  {0xB3, 0xFF, 0x1F}, 
  {0xB4, 0xFF, 0x1F}, 
  {0x00, 0xFF, 0x01},  
  {0xE2, 0xFF, 0x02},  
  {0x81, 0xFF, 0x05},   
  {0xF4, 0xFF, 0x07},  
  {0xD4, 0x1F, 0x2F},  
  {0xD6, 0xFF, 0x0C}, 
  {0xB8, 0xFF, 0x42},
  {0xBF, 0xFF, 0x1C},
//  {0x82, 0xFF, 0xC6},
//  {0x84, 0xFF, 0x8C}, 
//  {0x85, 0xFF, 0x02}, 
//  {0x86, 0xFF, 0x6C}, 
//  {0x87, 0xFF, 0x01}, 
  {0x00, 0xFF, 0x00},  

  {0x09, 0xff, 0x00},
  {0x07, 0xff, 0x04},
  {0x32, 0xff, 0xa4},
  {0x31, 0xff, 0x0f},
  {0x06, 0xff, 0x06},
  {0x66, 0xff, 0x17},
  {0x08, 0xff, 0x0c},
  {0x03, 0xff, 0x01},
  {0x7d, 0xff, 0x0a},
  {0x17, 0xff, 0xe0},
  {0x19, 0xff, 0x80},
  {0x18, 0xff, 0x02},
  {0x01, 0xff, 0x01},
  {0, 0, 0}
};

	mxl_debug("()");

	return mxl111sf_ctrl_program_regs(state, mxl_111_overwrite_default);
}

int mxl1x1sf_soft_reset(struct mxl111sf_state *state)
{
	int ret;
	mxl_debug("()");

	ret = mxl111sf_write_reg(state, 0xff, 0x00); /* AIC */
	if (mxl_fail(ret))
		goto fail;
	ret = mxl111sf_write_reg(state, 0x02, 0x01); /* get out of reset */
	mxl_fail(ret);
fail:
	return ret;
}

int mxl1x1sf_set_device_mode(struct mxl111sf_state *state, int mode)
{
	int ret;

	mxl_debug("(%s)", MXL_SOC_MODE == mode ?
		"MXL_SOC_MODE" : "MXL_TUNER_MODE");

	/* set device mode */
	ret = mxl111sf_write_reg(state, 0x03,
				 MXL_SOC_MODE == mode ? 0x01 : 0x00);
	if (mxl_fail(ret))
		goto fail;

	ret = mxl111sf_write_reg_mask(state,
				      0x7d, 0x40, MXL_SOC_MODE == mode ?
				      0x00 : /* enable impulse noise filter,
						INF_BYP = 0 */
				      0x40); /* disable impulse noise filter,
						INF_BYP = 1 */
	if (mxl_fail(ret))
		goto fail;

	state->device_mode = mode;
fail:
	return ret;
}

/* power up tuner */
int mxl1x1sf_top_master_ctrl(struct mxl111sf_state *state, int onoff)
{
	mxl_debug("(%d)", onoff);

	return mxl111sf_write_reg(state, 0x01, onoff ? 0x01 : 0x00);
}

int mxl111sf_disable_656_port(struct mxl111sf_state *state)
{
	mxl_debug("()");

	return mxl111sf_write_reg_mask(state, 0x12, 0x04, 0x00);
}

int mxl111sf_enable_usb_output(struct mxl111sf_state *state)
{
	mxl_debug("()");

	return mxl111sf_write_reg_mask(state, 0x17, 0x40, 0x00);
}

/* initialize TSIF as input port of MxL1X1SF for MPEG2 data transfer */
int mxl111sf_config_mpeg_in(struct mxl111sf_state *state,
			    unsigned int parallel_serial,
			    unsigned int msb_lsb_1st,
			    unsigned int clock_phase,
			    unsigned int mpeg_valid_pol,
			    unsigned int mpeg_sync_pol)
{
	int ret;
	u8 mode, tmp;

	mxl_debug("(%u,%u,%u,%u,%u)", parallel_serial, msb_lsb_1st,
		  clock_phase, mpeg_valid_pol, mpeg_sync_pol);

	/* Enable PIN MUX */
	ret = mxl111sf_write_reg(state, V6_PIN_MUX_MODE_REG, V6_ENABLE_PIN_MUX);
	mxl_fail(ret);

	/* Configure MPEG Clock phase */
	mxl111sf_read_reg(state, V6_MPEG_IN_CLK_INV_REG, &mode);

	if (clock_phase == TSIF_NORMAL)
		mode &= ~V6_INVERTED_CLK_PHASE;
	else
		mode |= V6_INVERTED_CLK_PHASE;

	ret = mxl111sf_write_reg(state, V6_MPEG_IN_CLK_INV_REG, mode);
	mxl_fail(ret);

	/* Configure data input mode, MPEG Valid polarity, MPEG Sync polarity
	 * Get current configuration */
	ret = mxl111sf_read_reg(state, V6_MPEG_IN_CTRL_REG, &mode);
	mxl_fail(ret);

	/* Data Input mode */
	if (parallel_serial == TSIF_INPUT_PARALLEL) {
		/* Disable serial mode */
		mode &= ~V6_MPEG_IN_DATA_SERIAL;

		/* Enable Parallel mode */
		mode |= V6_MPEG_IN_DATA_PARALLEL;
	} else {
		/* Disable Parallel mode */
		mode &= ~V6_MPEG_IN_DATA_PARALLEL;

		/* Enable Serial Mode */
		mode |= V6_MPEG_IN_DATA_SERIAL;

		/* If serial interface is chosen, configure
		   MSB or LSB order in transmission */
		ret = mxl111sf_read_reg(state,
					V6_MPEG_INOUT_BIT_ORDER_CTRL_REG,
					&tmp);
		mxl_fail(ret);

		if (msb_lsb_1st == MPEG_SER_MSB_FIRST_ENABLED)
			tmp |= V6_MPEG_SER_MSB_FIRST;
		else
			tmp &= ~V6_MPEG_SER_MSB_FIRST;

		ret = mxl111sf_write_reg(state,
					 V6_MPEG_INOUT_BIT_ORDER_CTRL_REG,
					 tmp);
		mxl_fail(ret);
	}

	/* MPEG Sync polarity */
	if (mpeg_sync_pol == TSIF_NORMAL)
		mode &= ~V6_INVERTED_MPEG_SYNC;
	else
		mode |= V6_INVERTED_MPEG_SYNC;

	/* MPEG Valid polarity */
	if (mpeg_valid_pol == 0)
		mode &= ~V6_INVERTED_MPEG_VALID;
	else
		mode |= V6_INVERTED_MPEG_VALID;

	ret = mxl111sf_write_reg(state, V6_MPEG_IN_CTRL_REG, mode);
	mxl_fail(ret);

	return ret;
}

int mxl111sf_init_i2s_port(struct mxl111sf_state *state, u8 sample_size)
{
	static struct mxl111sf_reg_ctrl_info init_i2s[] = {
		{0x1b, 0xff, 0x1e}, /* pin mux mode, Choose 656/I2S input */
		{0x15, 0x60, 0x60}, /* Enable I2S */
		{0x17, 0xe0, 0x20}, /* Input, MPEG MODE USB,
				       Inverted 656 Clock, I2S_SOFT_RESET,
				       0 : Normal operation, 1 : Reset State */
#if 0
		{0x12, 0x01, 0x00}, /* AUDIO_IRQ_CLR (Overflow Indicator) */
#endif
		{0x00, 0xff, 0x02}, /* Change to Control Page */
		{0x26, 0x0d, 0x0d}, /* I2S_MODE & BT656_SRC_SEL for FPGA only */
		{0x00, 0xff, 0x00},
		{0,    0,    0}
	};
	int ret;

	mxl_debug("(0x%02x)", sample_size);

	ret = mxl111sf_ctrl_program_regs(state, init_i2s);
	if (mxl_fail(ret))
		goto fail;

	ret = mxl111sf_write_reg(state, V6_I2S_NUM_SAMPLES_REG, sample_size);
	mxl_fail(ret);
fail:
	return ret;
}

int mxl111sf_disable_i2s_port(struct mxl111sf_state *state)
{
	static struct mxl111sf_reg_ctrl_info disable_i2s[] = {
		{0x15, 0x40, 0x00},
		{0,    0,    0}
	};

	mxl_debug("()");

	return mxl111sf_ctrl_program_regs(state, disable_i2s);
}

int mxl111sf_config_i2s(struct mxl111sf_state *state,
			u8 msb_start_pos, u8 data_width)
{
	int ret;
	u8 tmp;

	mxl_debug("(0x%02x, 0x%02x)", msb_start_pos, data_width);

	ret = mxl111sf_read_reg(state, V6_I2S_STREAM_START_BIT_REG, &tmp);
	if (mxl_fail(ret))
		goto fail;

	tmp &= 0xe0;
	tmp |= msb_start_pos;
	ret = mxl111sf_write_reg(state, V6_I2S_STREAM_START_BIT_REG, tmp);
	if (mxl_fail(ret))
		goto fail;

	ret = mxl111sf_read_reg(state, V6_I2S_STREAM_END_BIT_REG, &tmp);
	if (mxl_fail(ret))
		goto fail;

	tmp &= 0xe0;
	tmp |= data_width;
	ret = mxl111sf_write_reg(state, V6_I2S_STREAM_END_BIT_REG, tmp);
	mxl_fail(ret);
fail:
	return ret;
}

int mxl111sf_config_spi(struct mxl111sf_state *state, int onoff)
{
	u8 val;
	int ret;

	mxl_debug("(%d)", onoff);

	ret = mxl111sf_write_reg(state, 0x00, 0x02);
	if (mxl_fail(ret))
		goto fail;

	ret = mxl111sf_read_reg(state, V8_SPI_MODE_REG, &val);
	if (mxl_fail(ret))
		goto fail;

	if (onoff)
		val |= 0x04;
	else
		val &= ~0x04;

	ret = mxl111sf_write_reg(state, V8_SPI_MODE_REG, val);
	if (mxl_fail(ret))
		goto fail;

	ret = mxl111sf_write_reg(state, 0x00, 0x00);
	mxl_fail(ret);
fail:
	return ret;
}

int mxl111sf_idac_config(struct mxl111sf_state *state,
			 u8 control_mode, u8 current_setting,
			 u8 current_value, u8 hysteresis_value)
{
	int ret;
	u8 val;
	/* current value will be set for both automatic & manual IDAC control */
	val = current_value;

	if (control_mode == IDAC_MANUAL_CONTROL) {
		/* enable manual control of IDAC */
		val |= IDAC_MANUAL_CONTROL_BIT_MASK;

		if (current_setting == IDAC_CURRENT_SINKING_ENABLE)
			/* enable current sinking in manual mode */
			val |= IDAC_CURRENT_SINKING_BIT_MASK;
		else
			/* disable current sinking in manual mode */
			val &= ~IDAC_CURRENT_SINKING_BIT_MASK;
	} else {
		/* disable manual control of IDAC */
		val &= ~IDAC_MANUAL_CONTROL_BIT_MASK;

		/* set hysteresis value  reg: 0x0B<5:0> */
		ret = mxl111sf_write_reg(state, V6_IDAC_HYSTERESIS_REG,
					 (hysteresis_value & 0x3F));
		mxl_fail(ret);
	}

	ret = mxl111sf_write_reg(state, V6_IDAC_SETTINGS_REG, val);
	mxl_fail(ret);

	return ret;
}

/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
