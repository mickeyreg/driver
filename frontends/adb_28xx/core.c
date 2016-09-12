#include "core.h"
#include "stv6110x.h"
#include "stv090x.h"
#include "dib7000p.h"
#include "dib0070.h"

#include "mxl111sf.h"
#include "mxl111sf-demod.h"
#include "mxl111sf-tuner.h"
#include "mxl111sf-phy.h"
#include "mxl111sf-reg.h"
//#include "mxl111sf-gpio.h"

#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/io.h>
#include <linux/dvb/dmx.h>
#include <linux/proc_fs.h>
#include <pvr_config.h>

#include <linux/gpio.h>
#include <linux/stm/gpio.h>
#include <linux/delay.h>

#define ADB2850 0
#define ADB2849 1

#define DVBT

int box_type = ADB2850;

short paramDebug = 0;
int bbgain = -1;

static struct core *core[MAX_DVB_ADAPTERS];

static struct stv090x_config tt1600_stv090x_config = {
	.device			= STX7111,
	.demod_mode		= STV090x_DUAL,
	.clk_mode		= STV090x_CLK_EXT,
	.xtal			= 30000000,
	.address		= 0x68,
	.ref_clk		= 16000000,
	.ts1_mode		= STV090x_TSMODE_DVBCI,
	.ts2_mode		= STV090x_TSMODE_SERIAL_CONTINUOUS,
	.ts1_clk		= 0, /* diese regs werden in orig nicht gesetzt */
	.ts2_clk		= 0, /* diese regs werden in orig nicht gesetzt */
	.repeater_level		= STV090x_RPTLEVEL_64,
	.tuner_bbgain = 10,
	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_2Vpp,
	.diseqc_envelope_mode = false,
	.tuner_init				= NULL,
	.tuner_set_mode			= NULL,
	.tuner_set_frequency	= NULL,
	.tuner_get_frequency	= NULL,
	.tuner_set_bandwidth	= NULL,
	.tuner_get_bandwidth	= NULL,
	.tuner_set_bbgain		= NULL,
	.tuner_get_bbgain		= NULL,
	.tuner_set_refclk		= NULL,
	.tuner_get_status		= NULL,
};

static struct stv6110x_config stv6110x_config = {
	.addr			= 0x63,//0x60,
//	.refclk			= 16000000,
	.refclk			= 30000000,
};


#ifdef DVBT

static struct dibx000_agc_config dib7070_agc_config = {
	.band_caps = BAND_UHF | BAND_VHF | BAND_LBAND | BAND_SBAND,

	
	// * P_agc_use_sd_mod1=0, P_agc_use_sd_mod2=0, P_agc_freq_pwm_div=5,
	// * P_agc_inv_pwm1=0, P_agc_inv_pwm2=0, P_agc_inh_dc_rv_est=0,
	// * P_agc_time_est=3, P_agc_freeze=0, P_agc_nb_est=5, P_agc_write=0
	 
	.setup = (0 << 15) | (0 << 14) | (5 << 11) | (0 << 10) | (0 << 9) |
		 (0 << 8) | (3 << 5) | (0 << 4) | (5 << 1) | (0 << 0),
	.inv_gain = 600,
	.time_stabiliz = 10,
	.alpha_level = 0,
	.thlock = 118,
	.wbd_inv = 0,
	.wbd_ref = 3530,
	.wbd_sel = 1,
	.wbd_alpha = 5,
	.agc1_max = 65535,
	.agc1_min = 0,
	.agc2_max = 65535,
	.agc2_min = 0,
	.agc1_pt1 = 0,
	.agc1_pt2 = 40,
	.agc1_pt3 = 183,
	.agc1_slope1 = 206,
	.agc1_slope2 = 255,
	.agc2_pt1 = 72,
	.agc2_pt2 = 152,
	.agc2_slope1 = 88,
	.agc2_slope2 = 90,
	.alpha_mant = 17,
	.alpha_exp = 27,
	.beta_mant = 23,
	.beta_exp = 51,
	.perform_agc_softsplit = 0,
};

static struct dibx000_agc_config dib7070_agc_config_ = {
	BAND_UHF | BAND_VHF | BAND_LBAND | BAND_SBAND,
	/* P_agc_use_sd_mod1=0, P_agc_use_sd_mod2=0, P_agc_freq_pwm_div=5, P_agc_inv_pwm1=0, P_agc_inv_pwm2=0,
	 * P_agc_inh_dc_rv_est=0, P_agc_time_est=3, P_agc_freeze=0, P_agc_nb_est=5, P_agc_write=0 */
	(0 << 15) | (0 << 14) | (5 << 11) | (0 << 10) | (0 << 9) | (0 << 8) | (3 << 5) | (0 << 4) | (5 << 1) | (0 << 0), // setup

	600, // inv_gain
	10,  // time_stabiliz

	0,  // alpha_level
	118,  // thlock

	0,     // wbd_inv
	3530,  // wbd_ref
	1,     // wbd_sel
	5,     // wbd_alpha

	65535,  // agc1_max
		0,  // agc1_min

	65535,  // agc2_max
	0,      // agc2_min

	0,      // agc1_pt1
	40,     // agc1_pt2
	183,    // agc1_pt3
	206,    // agc1_slope1
	255,    // agc1_slope2
	72,     // agc2_pt1
	152,    // agc2_pt2
	88,     // agc2_slope1
	90,     // agc2_slope2

	17,  // alpha_mant
	27,  // alpha_exp
	23,  // beta_mant
	51,  // beta_exp

	0,  // perform_agc_softsplit
};

static struct dibx000_bandwidth_config dib7070_bw_config_12_mhz = {
	.internal = 60000,
	.sampling = 15000,
	.pll_prediv = 1,
	.pll_ratio = 20,
	.pll_range = 3,
	.pll_reset = 1,
	.pll_bypass = 0,
	.enable_refdiv = 0,
	.bypclk_div = 0,
	.IO_CLK_en_core = 1,
	.ADClkSrc = 1,
	.modulo = 2,
	// refsel, sel, freq_15k 
	.sad_cfg = (3 << 14) | (1 << 12) | (524 << 0),
	.ifreq = (0 << 25) | 0,
	.timf = 20452225,
	.xtal_hz = 12000000,
};

static struct dibx000_bandwidth_config dib7070_bw_config_12_mhz_ = {
	60000, 15000, // internal, sampling
	1, 20, 3, 1, 0, // pll_cfg: prediv, ratio, range, reset, bypass
	0, 0, 1, 1, 2, // misc: refdiv, bypclk_div, IO_CLK_en_core, ADClkSrc, modulo
	(3 << 14) | (1 << 12) | (524 << 0), // sad_cfg: refsel, sel, freq_15k
	(0 << 25) | 0, // ifreq = 0.000000 MHz
	20452225, // timf
	12000000, // xtal_hz
};

static struct dib7000p_config cxusb_dualdig4_rev2_config = {
	.output_mode = OUTMODE_MPEG2_SERIAL,//OUTMODE_MPEG2_PAR_GATED_CLK,
	.output_mpeg2_in_188_bytes = 1,

	.agc_config_count = 1,
	.agc = &dib7070_agc_config_,
	.bw  = &dib7070_bw_config_12_mhz_,
	.tuner_is_baseband = 1,
	.spur_protect = 1,

	.gpio_dir = 0xfcef,
	.gpio_val = 0x0110,
//	.gpio_dir = DIB7000P_GPIO_DEFAULT_DIRECTIONS,
//	.gpio_val = DIB7000P_GPIO_DEFAULT_VALUES,
	.gpio_pwm_pos = DIB7000P_GPIO_DEFAULT_PWM_POS,

	.hostbus_diversity = 1,
};

static int dib7070_tuner_reset(struct dvb_frontend *fe, int onoff)
{
	printk("dib7070_tuner_reset:%d\n",onoff);
	return dib7000p_set_gpio(fe, 8, 0, !onoff);
	return 0;
}

static int dib7070_tuner_sleep(struct dvb_frontend *fe, int onoff)
{
	printk("dib7070_tuner_sleep:%d\n",onoff);
	return dib7000p_set_gpio(fe, 9, 0, onoff);
	return 0;
}

static struct dib0070_config dib7070p_dib0070_config = {
	.i2c_address = 0x60,//DEFAULT_DIB0070_I2C_ADDRESS,
	.reset = dib7070_tuner_reset,
	.sleep = dib7070_tuner_sleep,
	.clock_khz = 12000,
	.clock_pad_drive = 4,
	.charge_pump = 2,
};

struct dib0700_adapter_state {
	int (*set_param_save) (struct dvb_frontend *,
			       struct dvb_frontend_parameters *);
};

static int dib7070_set_param_override(struct dvb_frontend *fe,
				      struct dvb_frontend_parameters *fep)
{
	struct dib0700_adapter_state *state = fe->demodulator_priv;
	
	u16 offset;
	u8 band = BAND_OF_FREQUENCY(fep->frequency/1000);
	switch (band) {
	case BAND_VHF: offset = 950; break;
	default:
	case BAND_UHF: offset = 550; break;
	}
	printk("WBD for DiB7000P: %d\n", offset + dib0070_wbd_offset(fe));
	dib7000p_set_wbd_ref(fe, offset + dib0070_wbd_offset(fe));
	return state->set_param_save(fe, fep);
}

static struct mxl111sf_demod_config mxl_demod_config = {
	.read_reg        = mxl111sf_read_reg,
	.write_reg       = mxl111sf_write_reg,
	.program_regs    = mxl111sf_ctrl_program_regs,
};


static int mxl111sf_ant_hunt(struct dvb_frontend *fe)
{
	return 0;
}

static struct mxl111sf_tuner_config mxl_tuner_config = {
	.if_freq         = MXL_IF_6_0, /* applies to external IF output, only */
	.invert_spectrum = 0,
	.read_reg        = mxl111sf_read_reg,
	.write_reg       = mxl111sf_write_reg,
	.program_regs    = mxl111sf_ctrl_program_regs,
	.top_master_ctrl = mxl1x1sf_top_master_ctrl,
	.ant_hunt        = mxl111sf_ant_hunt,
};

static int mxl1x1sf_get_chip_info(struct mxl111sf_state *state)
{
	int ret;
	unsigned char id, ver;
	char *mxl_chip, *mxl_rev;

//	if ((state->chip_id) && (state->chip_ver))
//		return 0;

	if(mxl111sf_read_reg(state, CHIP_ID_REG, &id)) goto fail;
	state->chip_id = id;

	if(mxl111sf_read_reg(state, TOP_CHIP_REV_ID_REG, &ver)) goto fail;
	state->chip_ver = ver;

	switch (id) {
	case 0x61:
		mxl_chip = "MxL101SF";
		break;
	case 0x63:
		mxl_chip = "MxL111SF";
		break;
	default:
		mxl_chip = "UNKNOWN MxL1X1";
		break;
	}
	switch (ver) {
	case 0x36:
		state->chip_rev = MXL111SF_V6;
		mxl_rev = "v6";
		break;
	case 0x08:
		state->chip_rev = MXL111SF_V8_100;
		mxl_rev = "v8_100";
		break;
	case 0x18:
		state->chip_rev = MXL111SF_V8_200;
		mxl_rev = "v8_200";
		break;
	default:
		state->chip_rev = 0;
		mxl_rev = "UNKNOWN REVISION";
		break;
	}
	info("%s detected, %s (0x%x)\n", mxl_chip, mxl_rev, ver);
fail:
	return ret;
}

struct mxl111sf_state_ {
	u8 i2c_addr;
	struct i2c_adapter	*i2c_adap;
};

#endif




static struct dvb_frontend * frontend_init(struct core_config *cfg, int i)
{
	struct tuner_devctl *ctl = NULL;
	struct dvb_frontend *frontend = NULL;
	struct mxl111sf_state *state;// = NULL;

	printk("%s > nr:%d\n", __FUNCTION__,i);

	if(i==0)
	{
	frontend = stv090x_attach(&tt1600_stv090x_config, cfg->i2c_adap, STV090x_DEMODULATOR_0, STV090x_TUNER1);
	if (frontend) {
		printk("%s: attached stv090x\n", __FUNCTION__);
		ctl = dvb_attach(stv6110x_attach, frontend, &stv6110x_config, cfg->i2c_adap);
		if(ctl){
			printk("%s: attached stv6110x\n", __FUNCTION__);
			tt1600_stv090x_config.tuner_init	  	  = ctl->tuner_init;
			tt1600_stv090x_config.tuner_set_mode	  = ctl->tuner_set_mode;
			tt1600_stv090x_config.tuner_set_frequency = ctl->tuner_set_frequency;
			tt1600_stv090x_config.tuner_get_frequency = ctl->tuner_get_frequency;
			tt1600_stv090x_config.tuner_set_bandwidth = ctl->tuner_set_bandwidth;
			tt1600_stv090x_config.tuner_get_bandwidth = ctl->tuner_get_bandwidth;
			tt1600_stv090x_config.tuner_set_bbgain	  = ctl->tuner_set_bbgain;
			tt1600_stv090x_config.tuner_get_bbgain	  = ctl->tuner_get_bbgain;
			tt1600_stv090x_config.tuner_set_refclk	  = ctl->tuner_set_refclk;
			tt1600_stv090x_config.tuner_get_status	  = ctl->tuner_get_status;
		} 
		else {
			printk ("%s: error attaching stv6110x\n", __FUNCTION__);
			goto error_out;
			}
	} 
	else {
		printk ("%s: error attaching stv090x\n", __FUNCTION__);
		goto error_out;
		}
	}

#ifdef DVBT
	else
	if((i==1)&&(box_type==ADB2850))
	{
			printk("DVBT DIB7070\n");
			frontend = dvb_attach(dib7000p_attach, cfg->i2c_adap, 18,&cxusb_dualdig4_rev2_config);
			if (frontend) {
				printk("%s: demod attached dib7000p\n", __FUNCTION__);
				if(dvb_attach(dib0070_attach, frontend, cfg->i2c_adap,&dib7070p_dib0070_config)== NULL)
				{
					printk ("%s: error attaching dib0070\n", __FUNCTION__);
					goto error_out;
				} else {
					printk("%s: dib0070 attached\n", __FUNCTION__);
					struct dib0700_adapter_state *st = frontend->demodulator_priv;
					st->set_param_save = frontend->ops.tuner_ops.set_params;
					frontend->ops.tuner_ops.set_params = dib7070_set_param_override;
				}
			}
			else {
				printk ("%s: error attaching dib7000p\n", __FUNCTION__);
				goto error_out;
				}
	}
	else
	if((i==1)&&(box_type==ADB2849))
	{
		printk("DVBT MXL101SF\n");
			state = kmalloc (sizeof (struct mxl111sf_state), GFP_KERNEL);
			if (state == NULL)
			{
			printk ("DVBT MXL101SF: kmalloc failed\n");
			return NULL;
			}
		state->i2c_addr = 0x60;
		state->i2c_adap = cfg->i2c_adap;
		if(mxl1x1sf_soft_reset(state)) goto error_out;
		mxl1x1sf_get_chip_info(state);
		if(mxl111sf_init_tuner_demod(state)) goto error_out;
		frontend = dvb_attach(mxl111sf_demod_attach,state,&mxl_demod_config);
		if (frontend) {
			printk("%s: demod attached mxl101sf\n", __FUNCTION__);
				if(dvb_attach(mxl111sf_tuner_attach, frontend, state,&mxl_tuner_config)== NULL)
				{
					printk ("%s: error attaching mxl101sf\n", __FUNCTION__);
					goto error_out;
				} else {
					printk("%s: mxl101sf attached\n", __FUNCTION__);
				}
			}
			else {
				printk ("%s: error attaching mxl101sf\n", __FUNCTION__);
				goto error_out;
				}
	}
	else return NULL;
#endif

	printk ("%s < nr:%d\n", __FUNCTION__,i);
	return frontend;

error_out:
	printk("core: Frontend registration failed!\n");
	if (frontend) 
		dvb_frontend_detach(frontend);
	return NULL;
}

static struct dvb_frontend *
init_stv090x_device (struct dvb_adapter *adapter,
                     struct plat_tuner_config *tuner_cfg, int i)
{
  struct dvb_frontend *frontend;
  struct core_config *cfg;

  printk ("> (bus = %d) %s\n", tuner_cfg->i2c_bus,__FUNCTION__);

  cfg = kmalloc (sizeof (struct core_config), GFP_KERNEL);
  if (cfg == NULL)
  {
    printk ("stv090x: kmalloc failed\n");
    return NULL;
  }

  /* initialize the config data */
  cfg->tuner_enable_pin = NULL;
  cfg->i2c_adap = i2c_get_adapter (tuner_cfg->i2c_bus);

  printk("i2c adapter = 0x%0x\n", cfg->i2c_adap);

  cfg->i2c_addr = tuner_cfg->i2c_addr;

  printk("i2c addr = %02x\n", cfg->i2c_addr);

  if (cfg->i2c_adap == NULL)
  {
      printk ("[stv090x] failed to allocate resources (i2c adapter)\n");
      goto error;
  }

  frontend = frontend_init(cfg, i);

  if (frontend == NULL)
  {
      printk ("[stv090x] frontend init failed!\n");
      goto error;
  }

  printk (KERN_INFO "%s: Call dvb_register_frontend (adapter = 0x%x)\n",
           __FUNCTION__, (unsigned int) adapter);

  if (dvb_register_frontend (adapter, frontend))
  {
    printk ("[stv090x] frontend registration failed !\n");
    if (frontend->ops.release)
      frontend->ops.release (frontend);
    goto error;
  }

  return frontend;

error:
	if(cfg->tuner_enable_pin != NULL)
	{
		printk("[stv090x] freeing tuner enable pin\n");
		stpio_free_pin (cfg->tuner_enable_pin);
	}
	kfree(cfg);
  	return NULL;
}

struct plat_tuner_config tuner_resources[] = {
        [0] = {
                .adapter = 0,
                .i2c_bus = 1,
                .i2c_addr = 0x68,
        },
#ifdef DVBT
        [1] = {
                .adapter = 0,
                .i2c_bus = 1,
                .i2c_addr = 0x00,	//fake addres
        },
#endif
};

void stv090x_register_frontend(struct dvb_adapter *dvb_adap)
{
	int i = 0;
	int vLoop = 0;

	printk (KERN_INFO "%s: stv090x DVB: 0.11 \n", __FUNCTION__);

	core[i] = (struct core*) kmalloc(sizeof(struct core),GFP_KERNEL);
	if (!core[i])
		return;

	memset(core[i], 0, sizeof(struct core));

	core[i]->dvb_adapter = dvb_adap;
	dvb_adap->priv = core[i];

	printk("tuner = %d\n", ARRAY_SIZE(tuner_resources));
	
	for (vLoop = 0; vLoop < ARRAY_SIZE(tuner_resources); vLoop++)
	{
	  if (core[i]->frontend[vLoop] == NULL)
	  {
      	 printk("%s: init tuner %d\n", __FUNCTION__, vLoop);
	     core[i]->frontend[vLoop] = 
				   init_stv090x_device (core[i]->dvb_adapter, &tuner_resources[vLoop], vLoop);
	  }
	}

	printk (KERN_INFO "%s: <\n", __FUNCTION__);

	return;
}

EXPORT_SYMBOL(stv090x_register_frontend);

#define DIB7070_RST stm_gpio(7, 2)

int __init stv090x_init(void)
{
	gpio_request(DIB7070_RST, "DIB7070_RST");
	if (DIB7070_RST==NULL)
	{
	printk("DIB7070_RST Fail request pio !!!\n");
	}
	else
	{
	printk("DIB7070_RST OK !!!\n");
	gpio_direction_output(DIB7070_RST, STM_GPIO_DIRECTION_OUT);
	gpio_set_value(DIB7070_RST, 0);
	mdelay(10);
	gpio_set_value(DIB7070_RST, 1);
	mdelay(200);
	}
	return 0;
}

static void __exit stv090x_exit(void) 
{  
   printk("stv090x unloaded\n");
}

module_init             (stv090x_init);
module_exit             (stv090x_exit);

module_param(paramDebug, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(paramDebug, "Debug Output 0=disabled >0=enabled(debuglevel)");

module_param(bbgain, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP);
MODULE_PARM_DESC(bbgain, "default=-1 (use default config = 10");

module_param(box_type, int, 0444);
MODULE_PARM_DESC(box_type, "boxtype 0=adb2850(default) 1=adb2849");

MODULE_DESCRIPTION      ("Tunerdriver");
MODULE_AUTHOR           ("TDT (mod plfreebox@gmail.com)");
MODULE_LICENSE          ("GPL");
