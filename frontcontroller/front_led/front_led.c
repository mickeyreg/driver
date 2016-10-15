#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/version.h>

#include <linux/gpio.h>
#include <linux/stm/gpio.h>

#define VFD_MAJOR	147

#define LED1 stm_gpio(2, 1)
#define LED2 stm_gpio(2, 2)
#define LED3 stm_gpio(2, 3)

struct vfd_ioctl_data {
    unsigned char address;
    unsigned char data[64];
    unsigned char length;
};

struct vfd_driver {
  struct semaphore      sem;
  int opencount;
};

spinlock_t mr_lock = SPIN_LOCK_UNLOCKED;

static struct vfd_driver vfd;
static int debug  = 0;

#define DBG(fmt, args...) if ( debug ) printk(KERN_INFO "[vfd] :: " fmt "\n", ## args )
#define ERR(fmt, args...) printk(KERN_ERR "[vfd] :: " fmt "\n", ## args )

typedef enum
{
/* common icons */
VFD_ICON_USB = 0x00,
VFD_ICON_HD,
VFD_ICON_HDD,
VFD_ICON_LOCK,
VFD_ICON_BT,
VFD_ICON_MP3,
VFD_ICON_MUSIC,
VFD_ICON_DD,
VFD_ICON_MAIL,
VFD_ICON_MUTE,
VFD_ICON_PLAY,
VFD_ICON_PAUSE,
VFD_ICON_FF,
VFD_ICON_FR,
VFD_ICON_REC,
VFD_ICON_CLOCK,

//adb2850
VFD_ICON_LED1 = 0x20,
VFD_ICON_LED2,
} VFD_ICON;


static void set_icon(unsigned char *kbuf, unsigned char len)
{
	spin_lock(&mr_lock);

	switch(kbuf[0])
	{
	case 1:
	    if(kbuf[4]==1) gpio_set_value(LED1, 1); else gpio_set_value(LED1, 0);
	    break;
	case 2:
	    if(kbuf[4]==1) gpio_set_value(LED2, 1); else gpio_set_value(LED2, 0);
	    break;
	case 3:
	    if(kbuf[4]==1) gpio_set_value(LED3, 1); else gpio_set_value(LED3, 0);
	    break;
	 //inverted blinking
	case 11:
	    if(kbuf[4]==1) gpio_set_value(LED1, 0); else gpio_set_value(LED1, 1);
	    break;
	case 12:
	    if(kbuf[4]==1) gpio_set_value(LED2, 0); else gpio_set_value(LED2, 1);
	    break;
	default:
	    ERR("icon unknown %d", kbuf[0]);
	    break;
	}

//led1 = green
//led2 = red
//led1 + led2 = orange

	spin_unlock(&mr_lock);
}

//-------------------------------------------------------------------------------------------
// VFD
//-------------------------------------------------------------------------------------------

#define VFDIOC_DCRAMWRITE			0xc0425a00
#define VFDIOC_BRIGHTNESS			0xc0425a03
#define VFDIOC_DISPLAYWRITEONOFF		0xc0425a05
#define VFDIOC_DRIVERINIT			0xc0425a08
#define VFDIOC_ICONDISPLAYONOFF			0xc0425a0a

static int vfd_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg ) {
  struct vfd_ioctl_data vfddata;

  switch( cmd ) {
  case VFDIOC_DCRAMWRITE:
    break;
  case VFDIOC_BRIGHTNESS:
    break;
  case VFDIOC_DISPLAYWRITEONOFF:
    break;
  case VFDIOC_ICONDISPLAYONOFF:
    copy_from_user(&vfddata, (void*)arg, sizeof( struct vfd_ioctl_data ) );
    set_icon(vfddata.data,vfddata.length);
    break;
  case VFDIOC_DRIVERINIT:
    break;
  default:
    ERR("[vfd] unknown ioctl %08x", cmd );
    break;
  }
  return 0;

}

static ssize_t vfd_write( struct file *filp, const char *buf, size_t len, loff_t *off ) {
  return len;
}

static ssize_t vfd_read(struct file *filp, char *buf, size_t len, loff_t *off ) {
  return len;
}

static int vfd_open( struct inode *inode, struct file *file ) {

  DBG("vfd_open");

  if ( down_interruptible( &(vfd.sem) ) ) {
        DBG("interrupted while waiting for sema.");
    return -ERESTARTSYS;
  }

  if ( vfd.opencount > 0 ) {
        DBG("device already opened.");
    up( &(vfd.sem) );
    return -EUSERS;
  }

  vfd.opencount++;
  up( &(vfd.sem) );

  return 0;
}

static int vfd_close( struct inode *inode, struct file *file ) {
  DBG("vfd_close");
  vfd.opencount = 0;
  return 0;
}

static struct file_operations vfd_fops = {
  .owner   = THIS_MODULE,
  .ioctl   = vfd_ioctl,
  .write   = vfd_write,
  .read    = vfd_read,
  .open    = vfd_open,
  .release = vfd_close
};

static void __exit led_module_exit(void) 
{
  unregister_chrdev( VFD_MAJOR, "vfd" );
}

static int __init led_module_init( void ) {

	DBG("LED ADB2850 init.");
	DBG("register character device %d.", VFD_MAJOR );
	if(register_chrdev( VFD_MAJOR, "vfd", &vfd_fops ) ) 
	{
	ERR("register major %d failed", VFD_MAJOR );
	goto led_init_fail;
	}

	sema_init( &(vfd.sem), 1 );
	vfd.opencount = 0;

gpio_request(LED1, "LED1");
if (LED1==NULL){ERR("Request LED1 failed. abort.");goto led_init_fail;}
gpio_direction_output(LED1, STM_GPIO_DIRECTION_OUT);
gpio_set_value(LED1, 1);	//green on

gpio_request(LED2, "LED2");
if (LED2==NULL){ERR("Request LED2 failed. abort.");goto led_init_fail;}
gpio_direction_output(LED2, STM_GPIO_DIRECTION_OUT);
gpio_set_value(LED2, 0);

return 0;

  led_init_fail:
  led_module_exit();
  return -EIO;
}

module_init(led_module_init);
module_exit(led_module_exit);

MODULE_DESCRIPTION("ADB28xx front_led driver");
MODULE_AUTHOR("plfreebox@gmail.com");
MODULE_LICENSE("GPL");
