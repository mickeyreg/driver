/*
 * Copyright (C) 2010 Michael Krufky (mkrufky@kernellabs.com)
 *
 *   This program is free software; you can redistribute it and/or modify it
 *   under the terms of the GNU General Public License as published by the Free
 *   Software Foundation, version 2.
 *
 * see Documentation/dvb/README.dvb-usb for more information
 */

#include <linux/i2c.h>

#include "mxl111sf.h"
#include "mxl111sf-reg.h"
#include "mxl111sf-phy.h"
#include "mxl111sf-demod.h"
#include "mxl111sf-tuner.h"

int dvb_usb_mxl111sf_debug=1;

int mxl111sf_read_reg(struct mxl111sf_state *state, u8 addr, u8 *data)
{
	u8 b0[] = { 0xfb, addr };
	u8 b;
	struct i2c_msg msg[2] = {
		{.addr = state->i2c_addr,.flags = 0,.buf = b0,.len = 2},
		{.addr = state->i2c_addr,.flags = I2C_M_RD,.buf = &b,.len = 1},
	};

	if (i2c_transfer(state->i2c_adap, msg, 2) != 2) {
		printk("MXL101SF I2C read failed\n");
		return -EINVAL;
	}
	*data = b;
	//printk("MXL101SF> i2c read addr:%02x val:%02x\n",addr,b);
	return 0;
}

int mxl111sf_write_reg(struct mxl111sf_state *state, u8 addr, u8 data)
{
	u8 b[3] = { addr, data };
	struct i2c_msg msg = {.addr = state->i2c_addr,.flags = 0,.buf = b,.len = 2 };

	if (i2c_transfer(state->i2c_adap, &msg, 1) != 1) {
		printk(KERN_WARNING "MXL101SF I2C write failed\n");
		return -EREMOTEIO;
	}
	//printk("MXL101SF> i2c write addr:%02x val:%02x\n",addr,data);
	return 0;}
/* ------------------------------------------------------------------------ */

int mxl111sf_write_reg_mask(struct mxl111sf_state *state,
				   u8 addr, u8 mask, u8 data)
{
	int ret;
	u8 val;

	if (mask != 0xff) {
		ret = mxl111sf_read_reg(state, addr, &val);
#if 1
		/* dont know why this usually errors out on the first try */
		if (mxl_fail(ret))
			err("error writing addr: 0x%02x, mask: 0x%02x, "
			    "data: 0x%02x, retrying...", addr, mask, data);

		ret = mxl111sf_read_reg(state, addr, &val);
#endif
		if (mxl_fail(ret))
			goto fail;
	}
	val &= ~mask;
	val |= data;

	ret = mxl111sf_write_reg(state, addr, val);
	mxl_fail(ret);
fail:
	return ret;
}

/* ------------------------------------------------------------------------ */

int mxl111sf_ctrl_program_regs(struct mxl111sf_state *state,
			       struct mxl111sf_reg_ctrl_info *ctrl_reg_info)
{
	int i, ret = 0;

	for (i = 0;  ctrl_reg_info[i].addr |
		     ctrl_reg_info[i].mask |
		     ctrl_reg_info[i].data;  i++) {

		ret = mxl111sf_write_reg_mask(state,
					      ctrl_reg_info[i].addr,
					      ctrl_reg_info[i].mask,
					      ctrl_reg_info[i].data);
		if (mxl_fail(ret)) {
			err("failed on reg #%d (0x%02x)", i,
			    ctrl_reg_info[i].addr);
			break;
		}
	}
	return ret;
}

/* ------------------------------------------------------------------------ */


/*
 * Local variables:
 * c-basic-offset: 8
 * End:
 */
