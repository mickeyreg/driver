/*
 * boxtype.c
 * 
 *  captaintrip 12.07.2008
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

/*
 *  Description:
 *
 *  in /proc/boxtype
 *
 *  14w returns 1 or 3
 *  01w returns 0
 *
 */

#define debug =1

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>

#if defined(ADB_BOX) || defined(ADB5800) || defined(ADB2850)
#include <linux/i2c.h>
#include <linux/delay.h>
#endif

#include "boxtype.h"
#include "pio.h"

#define procfs_name "boxtype"
#define DEVICENAME "boxtype"

int boxtype = 0;
struct proc_dir_entry *BT_Proc_File;

#if defined(ADB_BOX) || defined(ADB5800)
#define I2C_ADDR_STB0899_1	(0xd0 >> 1)	//d0=0x68 d2=69
#define I2C_ADDR_STB0899_2	(0xd2 >> 1)	//d0=0x68 d2=69
#define I2C_ADDR_STV090x	(0xd0 >> 1)	//d0=0x68 d2=69
#define I2C_ADDR_STV0297	(0x38 >> 1)
#define I2C_BUS 0

#define STB0899_NCOARSE		0xf1b3
#define STB0899_DEV_ID		0xf000
#define STV090x_MID		0xf100

static int _read_reg_boxtype(unsigned int reg, unsigned char nr)
{
	int ret;

	dprintk("899 read reg = %x\n", reg);

	u8 b0[] = { reg >> 8, reg & 0xff };
	u8 buf;

	struct i2c_msg msg[] = {
		{
			.addr	= nr,
			.flags	= 0,
			.buf	= b0,
			.len	= 2
		},{
			.addr	= nr,
			.flags	= I2C_M_RD,
			.buf	= &buf,
			.len	= 1
		}
	};


	struct i2c_adapter *i2c_adap = i2c_get_adapter (I2C_BUS);

	ret = i2c_transfer(i2c_adap, msg, 2);
	if (ret != 2) {
		return -1;
	}

	return (unsigned int) buf;
}

int stb0899_read_reg_boxtype(unsigned int reg,unsigned char nr)
{
	int result;

	result = _read_reg_boxtype(reg, nr);
	/*
	 * Bug ID 9:
	 * access to 0xf2xx/0xf6xx
	 * must be followed by read from 0xf2ff/0xf6ff.
	 */
	if ((reg != 0xf2ff) && (reg != 0xf6ff) &&
	    (((reg & 0xff00) == 0xf200) || ((reg & 0xff00) == 0xf600)))
		_read_reg_boxtype((reg | 0x00ff), nr);

	return result;
}

int stv6412_boxtype(void)
{
    int ret;
    unsigned char buf;

    struct i2c_msg msg = {.addr = 0x4b, I2C_M_RD, .buf = &buf, .len = 1 };

    struct i2c_adapter *i2c_adap = i2c_get_adapter (I2C_BUS);
    ret=i2c_transfer(i2c_adap, &msg, 1);
    return ret;
}


int stv0297_boxtype(void) 
{ 
	int ret;

	dprintk("stv0297 read reg = %x\n", 0);

	u8 b0[] = { 0 >> 8, 0 & 0xff };
	u8 buf;

	struct i2c_msg msg[] = {
		{
			.addr	= I2C_ADDR_STV0297,
			.flags	= 0,
			.buf	= b0,
			.len	= 1
		},{
			.addr	= I2C_ADDR_STV0297,
			.flags	= I2C_M_RD,
			.buf	= &buf,
			.len	= 1
		}
	};


	struct i2c_adapter *i2c_adap = i2c_get_adapter (I2C_BUS);

	ret = i2c_transfer(i2c_adap, msg, 2);
	if (ret != 2) {
		return -1;
	}

	return (unsigned int) buf;
}

int isl6405(void)	//dla bsla
{
    int ret;
    u8 b;
    struct i2c_msg msg_w = {.addr = 0x08, .flags = 0,.buf = &b,.len = 1 };
    struct i2c_msg msg_r = {.addr = 0x08, .flags = I2C_M_RD,.buf = &b,.len = 1 };
    struct i2c_adapter *i2c_adap = i2c_get_adapter (I2C_BUS);

    b=64; //DCL na 1
    ret=i2c_transfer(i2c_adap, &msg_w, 1);
    if (ret=!1) {printk("Error W:ISL6405 SR1\n"); return 2;}
    ret=i2c_transfer(i2c_adap, &msg_r, 1);
    if (ret=!1) {printk("Error R:ISL6405 SR1\n"); return 2;}

    b=b&128;
    if (b==128)
    {
        b=0; //DCL na 0
        ret=i2c_transfer(i2c_adap, &msg_w, 1);
        if (ret=!1) {printk("Error W:ISL6405 SR1\n"); return 2;}
        ret=i2c_transfer(i2c_adap, &msg_r, 1);
        if (ret=!1) {printk("Error R:ISL6405 SR1\n"); return 2;}
        b=b&128;
        if (b==0) return 0; else return 1;
    }
    else
    return 1;
}
#elif defined(ADB2850)
#define ADB2850 1
#define ADB2849 2
#define ADB2850b 3
#define I2C_BUS 1

int lnb_boxtype(void)
{
    int ret;
    unsigned char buf;

    struct i2c_adapter *i2c_adap = i2c_get_adapter (I2C_BUS);
    struct i2c_msg msg1 = {.addr = 0x0a, I2C_M_RD, .buf = &buf, .len = 1 };
    struct i2c_msg msg2 = {.addr = 0x60, I2C_M_RD, .buf = &buf, .len = 1 };

    ret=i2c_transfer(i2c_adap, &msg1, 1);
    if (ret!=1) // ADB2849
       return ret;

    ret=i2c_transfer(i2c_adap, &msg2, 1);
    if (ret==1) // ADB2850
       return ret;
    return 2;  // ADB2850b
}
#endif



/************************************************************************
*
* Unfortunately there is no generic mechanism to unambiguously determine
* STBs from different manufacturers. Since each hardware platform needs
* special I/O pin handling it also requires a different kernel image.
* Setting platform device configuration in the kernel helps to roughly
* determine the STB type. Further identification can be based on reading
* an EPLD or I/O pins.
* To provide a platform identification data add a platform device
* to the board setup.c file as follows:
*
*   static struct platform_device boxtype_device = {
*        .name = "boxtype",
*        .dev.platform_data = (void*)NEW_ID
*   };
*
*   static struct platform_device *plat_devices[] __initdata = {
*           &boxtype_device,
*           &st40_ohci_devices,
*           &st40_ehci_devices,
*           &ssc_device,
*   ...
*
* Where NEW_ID is a unique integer identifying the platform and
* plat_devices is provided to the platform_add_devices() function
* during initialization of hardware resources.
*
************************************************************************/
#if 0
static int boxtype_probe (struct device *dev)
{
  struct platform_device *pdev = to_platform_device (dev);

  if(pdev != NULL)
    boxtype = (int)pdev->dev.platform_data;

  return 0;
}

static int boxtype_remove (struct device *dev)
{
  return 0;
}
#endif

#if 0
static struct device_driver boxtype_driver = {
  .name = DEVICENAME,
  .bus = &platform_bus_type,
  .owner = THIS_MODULE,
  .probe = boxtype_probe,
  .remove = boxtype_remove,
};
#endif

int procfile_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data)
{
	int ret;

	if (offset > 0)
	{
		ret  = 0;
	}
	else
	{
#if !defined(ADB_BOX) && !defined(ADB5800) && !defined(ADB2850)
		ret = sprintf(buffer, "%d\n", boxtype);
#endif
#if defined(ADB_BOX)
		if (boxtype==1) ret = sprintf(buffer, "bska\n");
		else
		if (boxtype==2) ret = sprintf(buffer, "bsla\n");
		else
		if (boxtype==3) ret = sprintf(buffer, "bxzb\n");
		else
		if (boxtype==4) ret = sprintf(buffer, "bzzb\n");
		else
		if (boxtype==5) ret = sprintf(buffer, "bvxa\n");
		else
		ret = sprintf(buffer, "UNKOWN\n");
#elif defined(ADB5800)
		if (boxtype==1) ret = sprintf(buffer, "BSKA\n");
		else
		if (boxtype==2) ret = sprintf(buffer, "BSLA\n");
		else
		if (boxtype==3) ret = sprintf(buffer, "BXZB\n");
		else
		if (boxtype==4) ret = sprintf(buffer, "BZZB\n");
		else
		if (boxtype==5) ret = sprintf(buffer, "BVXA\n");
		else
		ret = sprintf(buffer, "UNKOWN\n");
#elif defined(ADB2850)
		if (boxtype==ADB2850) ret = sprintf(buffer, "ADB2850\n");
		else
		if (boxtype==ADB2849) ret = sprintf(buffer, "ADB2849\n");
		else
		if (boxtype==ADB2850b) ret = sprintf(buffer, "ADB2850b\n");
		else
		ret = sprintf(buffer, "UNKOWN\n");
	}
	return ret;
}

int __init boxtype_init(void)
{
#if defined(ADB_BOX) || defined(ADB5800) || defined(ADB2850)
	int ret;
#endif
	dprintk("[BOXTYPE] initializing ...\n");

// TODO: FIX THIS
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
	if (driver_register (&boxtype_driver) < 0)
	{
		printk ("%s(): error registering device driver\n", __func__);
	}
#endif

	if (boxtype == 0)
	{
#if !defined(ADB_BOX) && !defined(ADB5800) && !defined(ADB2850)
	/* no platform data found, assume ufs910 */
	boxtype = (STPIO_GET_PIN(PIO_PORT(4),5) << 1) | STPIO_GET_PIN(PIO_PORT(4), 4);
#endif
#if defined(ADB_BOX) || defined(ADB5800)
		ret=stv6412_boxtype();
		dprintk("ret stv6412 = %d\n", ret);//ret=1 -> ok
		if (ret!=1) boxtype=3;	//BXZB
		else
		{
		    ret = stv0297_boxtype();
		    dprintk("ret stv0297 = %d\n", ret);//ret=9 -> ok
		    if (ret==0x9) boxtype=5; 	//BVXA
		    else
		    {
			//jezeli uda sie przeczytac rejestar z drugiego demodulatora to jest to BSLA
			ret = stb0899_read_reg_boxtype(STB0899_NCOARSE,I2C_ADDR_STB0899_2);	//read DIV from demodulator
			dprintk("ret stb0899_2 = %d\n", ret);
			if (ret!=-1)
			    boxtype=2;	//BSLA
			else 
			{
			    //sprawdzamy czy to jest stb0899 czy stv090x
			    //czytamy chipid i rev
			    ret = stb0899_read_reg_boxtype(STB0899_DEV_ID,I2C_ADDR_STB0899_1);
			    dprintk("ret stb0899_1 = %d\n", ret);	//stb0899 = 130
			    if(ret>0x30)
				boxtype=1;	//BSKA
			    else
				boxtype=4;	//BZZB
			}
		    }
		}
#elif defined(ADB2850)
		ret=lnb_boxtype();
		dprintk("ret1 = %d\n", ret); //ret 0=adb2849 1=adb2850 2=adb2850b
		if (ret==1) boxtype = ADB2850; 
		else if (ret==2) boxtype = ADB2850b; 
		else boxtype = ADB2849;

		if (boxtype==ADB2850) dprintk("ADB2850\n");
		else if (boxtype==ADB2849) dprintk("ADB2849\n");
		else if (boxtype==ADB2850b) dprintk("ADB2850b\n");
		else dprintk("UNKOWN\n");
#endif
#if defined(ADB_BOX)
		if (boxtype==1) dprintk("bska\n");
		else
		if (boxtype==2) dprintk("bsla\n");
		else
		if (boxtype==3) dprintk("bxzb\n");
		else
		if (boxtype==4) dprintk("bzzb\n");
		else
		if (boxtype==5) dprintk("bvxa\n");
		else
		dprintk("UNKOWN\n");
#elif defined(ADB5800)
		if (boxtype==1) dprintk("BSKA\n");
		else
		if (boxtype==2) dprintk("BSLA\n");
		else
		if (boxtype==3) dprintk("BXZB\n");
		else
		if (boxtype==4) dprintk("BZZB\n");
		else
		if (boxtype==5) dprintk("BVXA\n");
		else
		dprintk("UNKOWN\n");
#endif
	}

	dprintk("[BOXTYPE] boxtype = %d\n", boxtype);

	BT_Proc_File = create_proc_read_entry(procfs_name, 0644, NULL, procfile_read, NULL);

	return 0;
}

void __exit boxtype_exit(void)
{
	dprintk("[BOXTYPE] unloading ...\n");
// TODO: FIX THIS
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,30)
	driver_unregister (&boxtype_driver);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	remove_proc_entry(procfs_name, &proc_root);
#else
	remove_proc_entry(procfs_name, NULL);
#endif
}

module_init(boxtype_init);
module_exit(boxtype_exit);

MODULE_DESCRIPTION("Provides the type of an STB based on STb71xx");
MODULE_AUTHOR("Team Ducktales mod B4Team & freebox & j00zek");
MODULE_LICENSE("GPL");
