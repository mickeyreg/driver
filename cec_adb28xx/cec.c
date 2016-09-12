/*
 */

#include <linux/proc_fs.h>  	/* proc fs */ 
#include <asm/uaccess.h>    	/* copy_from_user */

#include <linux/string.h>
#include <linux/module.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>

#include <linux/interrupt.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,17)
#include <linux/stm/pio.h>
#else
#include <linux/stpio.h>
#endif

#include <linux/gpio.h>
#include <linux/stm/gpio.h>

int remove_e2_procs(char *path, read_proc_t *read_func, write_proc_t *write_func);
int install_e2_procs(char *path, read_proc_t *read_func, write_proc_t *write_func, void *data);

unsigned long asc_registers;
struct stpio_pin* cec_tx;
struct stpio_pin* cec_rx;

static volatile unsigned char buf[3000];
static volatile unsigned int buf_len=0;
static volatile unsigned int buf_pos=0;

#define CEC_HARDWARE

#define DEBUG_CEC

#ifdef DEBUG_CEC
#define debug_cec(args...) printk(args)
#else
#define debug_cec(args...)
#endif

unsigned long asc_registers;

#define ASC_BAUDRATE			(asc_registers + 0x00)
#define ASC_TXBUF			(asc_registers + 0x04)
#define ASC_RXBUF			(asc_registers + 0x08)
#define ASC_CTL				(asc_registers + 0x0C)
#define ASC_INTEN			(asc_registers + 0x10)
#define ASC_STA				(asc_registers + 0x14)
#define ASC_GUARDTIME			(asc_registers + 0x18)
#define ASC_TIMEOUT			(asc_registers + 0x1C)
#define ASC_TXRESET			(asc_registers + 0x20)
#define ASC_RXRESET			(asc_registers + 0x24)
#define ASC_RETRIES			(asc_registers + 0x28)

#define ADJ 1
#define BAUDRATE_VAL_M1(bps, clk) \
	(((bps * (1 << 14)) / ((clk) / (1 << 6))) + ADJ)

#define PIO1BaseAddress       0xFD021000
#define CECBaseAddress        0xFE030C00
#define SysConfigBaseAddress  0xFE001000

#define PIO_PC0                0x24
#define PIO_PC1                0x34
#define PIO_PC2                0x44

#define SYS_CFG0               0x100
#define SYS_CFG2               0x108
#define SYS_CFG5               0x114

#define CEC_PRESCALER_LEFT     0x00
#define CEC_PRESCALER_RIGHT    0x04
#define CEC_STATUS_ERROR       0x08
#define CEC_CONTROL            0x0C
#define CEC_TX                 0x10
#define CEC_RX                 0x14
#define CEC_CFG                0x18
#define CEC_ADDRESS            0x1C

#define CEC_TRANSM_SOM            1 /* start of message */ 
#define CEC_TRANSM_EOM            2 /* end of message */ 
#define CEC_TRANSMIT_ERROR        4
#define CEC_TRANSMITTING          8
#define CEC_RECEIVE_SOM          16
#define CEC_RECEIVE_EOM          32
#define CEC_RECEIVE_ERROR        64
#define CEC_RECEIVING           128

#define CEC_STATUS_RECV_BTF 128
#define CEC_STATUS_RECV_ERR 64
#define CEC_STATUS_RECV_EOMSG 32
#define CEC_STATUS_RECV_SOMSG 16
#define CEC_STATUS_SEND_BTF 8
#define CEC_STATUS_SEND_ERR 4
#define CEC_STATUS_SEND_EOMSG 2
#define CEC_STATUS_SEND_SOMSG 1

#define CEC_ERROR_SEND_BTF          64
#define CEC_ERROR_ON_LINE          32
#define CEC_ERROR_ACK     16
#define CEC_ERROR_START         8
#define CEC_ERROR_RECV_BTF          4
#define CEC_ERROR_PERIOD    2
#define CEC_ERROR_TIMING    1

#define CEC_MAX_DATA_LEN 15
#define RECV_BUF_SIZE (CEC_MAX_DATA_LEN + 1)
#define SEND_BUF_SIZE (CEC_MAX_DATA_LEN + 1)

#define CEC_PIO stm_gpio(1, 7)

void cec_write_register_u32(unsigned long address, u32 value)
{
    unsigned long mapped_register = (unsigned long) ioremap(address, 4);

    writel(value, mapped_register);
     
    iounmap((void*) mapped_register);
}

u32 cec_read_register_u32(unsigned long address)
{
    u32 result;

    unsigned long mapped_register = (unsigned long) ioremap(address, 4);

    result = readl(mapped_register);
     
    iounmap((void*) mapped_register);
    
    return result;
}

void cec_write_data(u32 value)
{
    cec_write_register_u32(CECBaseAddress + CEC_TX, value);
}

u32 cec_read_data(void)
{
    return cec_read_register_u32(CECBaseAddress + CEC_RX);
}

void cec_start_sending(unsigned char isPing)
{
   //printk("[CEC] start_sending %d\n", isPing);
   if (isPing == 1)
   {
       cec_write_register_u32(CECBaseAddress + CEC_CONTROL, CEC_TRANSM_EOM | CEC_TRANSM_SOM);
   } else
   {
       cec_write_register_u32(CECBaseAddress + CEC_CONTROL, CEC_TRANSM_SOM);
   }
}

void cec_end_sending(void)
{
   //printk("[CEC] end_sending\n");
   cec_write_register_u32(CECBaseAddress + CEC_CONTROL, CEC_TRANSM_EOM);
}

u8 cec_get_status(void)
{
    u32 res = cec_read_register_u32(CECBaseAddress + CEC_CONTROL);
    return res & 0xFF;
}

u8 cec_get_error(void)
{
    u32 res = cec_read_register_u32(CECBaseAddress + CEC_STATUS_ERROR);
    return res & 0xFF;
}

void cec_acknowledge(void)
{
   //printk("[CEC] ack\n");
   cec_write_register_u32(CECBaseAddress + CEC_CONTROL, 0x00);
}

void cec_acknowledge_eom(void)
{
   //printk("[CEC] ack eom\n");
   cec_write_register_u32(CECBaseAddress + CEC_CONTROL, 0x02);
}

void cec_set_own_address(u32 own_address)
{
   cec_write_register_u32(CECBaseAddress + CEC_ADDRESS, own_address);
}

//------------------------------



void str_status(unsigned char status)
{
printk("[CEC] Control Status:\n");
if(status & CEC_STATUS_RECV_BTF)
printk("[CEC] \tRECV_BTF\n");
if(status & CEC_STATUS_RECV_ERR)
printk("[CEC] \tRECV_ERR\n");
if(status & CEC_STATUS_RECV_EOMSG)
printk("[CEC] \tRECV_EOMSG\n");
if(status & CEC_STATUS_RECV_SOMSG)
printk("[CEC] \tRECV_SOMSG\n");
if(status & CEC_STATUS_SEND_BTF)
printk("[CEC] \tSEND_BTF\n");
if(status & CEC_STATUS_SEND_ERR)
printk("[CEC] \tSEND_ERR\n");
if(status & CEC_STATUS_SEND_EOMSG)
printk("[CEC] \tSEND_EOMSG\n");
if(status & CEC_STATUS_SEND_SOMSG)
printk("[CEC] \tSEND_SOMSG\n");
}


#define CEC_ERROR_SEND_BTF 64
#define CEC_ERROR_ON_LINE  32
#define CEC_ERROR_ACK      16
#define CEC_ERROR_START     8
#define CEC_ERROR_RECV_BTF  4
#define CEC_ERROR_PERIOD    2
#define CEC_ERROR_TIMING    1

void str_error(unsigned char error)
{
printk("[CEC] Error Status:\n");
if(error & CEC_ERROR_SEND_BTF)
printk("[CEC] \tSEND_BTF\n");
if(error & CEC_ERROR_ON_LINE)
printk("[CEC] \tON_LINE - Collision\n");
if(error & CEC_ERROR_ACK)
printk("[CEC] \tACK - No one answered\n");
if(error & CEC_ERROR_START)
printk("[CEC] \tSTART\n");
if(error & CEC_ERROR_RECV_BTF)
printk("[CEC] \tRECV_BTF\n");
if(error & CEC_ERROR_PERIOD)
printk("[CEC] \tPERIOD\n");
if(error & CEC_ERROR_TIMING)
printk("[CEC] \tTIMING\n");
}

static int wait_send_btf ( int timeout )
{
    unsigned long start = jiffies;
    int status;

	while(1)
	{
		if(cec_get_status()&CEC_STATUS_SEND_BTF) break;
        if (jiffies - start > timeout) {
            debug_cec ("%s: timeout send_btf!!\n", __FUNCTION__);
            return -ETIMEDOUT;
        }
        msleep(1);
    };
    return 0;
}


//-------------------------------------------------------------------------
#ifndef CEC_HARDWARE

static int wait_idle ( int timeout )
{
    unsigned long start = jiffies;
    int status;

	while(1)
	{
		if (buf_len==0) break;
        if (jiffies - start > timeout) {
            debug_cec ("%s: timeout!!\n", __FUNCTION__);
            return -ETIMEDOUT;
        }
        msleep(10);
    };
    return 0;
}

int prepare_to_buf_300us(char	*myString,unsigned long count)
{

	unsigned char len;
	int i,j,k;
	unsigned char byte;
	unsigned char buf_[10];
	unsigned int value;

	len = count / 2;
	
	i=0;k=0;
	for (j=0;j<len;j++)
        {
		buf[0]='0';
		buf[1]='x';
		buf[2]=myString[k];k=k+1;
		buf[3]=myString[k];k=k+1;
		buf[4]= '\0';
		sscanf((char*)buf, "%x", &value);
		buf_[j]=value;
	}

	buf_pos=1;
	buf_len=0;

	//start bit
	for (i=0;i<12;i++){buf_len=buf_len+1;buf[buf_len]=0;} //3.6
	for (i=0;i<3;i++){buf_len=buf_len+1;buf[buf_len]=1;}  //0.9

	for (k=0;k<len;k++)
        {
	byte=buf_[k];
	debug_cec("[%d]=%x\n",k,byte);

	for (i=0;i<8;i++)
        {
		if((byte&128)==128) 
		{	//1
			for (j=0;j<2;j++){buf_len=buf_len+1;buf[buf_len]=0;}
			for (j=0;j<6;j++){buf_len=buf_len+1;buf[buf_len]=1;}
		}
		else
		{	//0
			for (j=0;j<5;j++){buf_len=buf_len+1;buf[buf_len]=0;}
			for (j=0;j<3;j++){buf_len=buf_len+1;buf[buf_len]=1;}
		}
		byte=byte<<1;
	}

	if ((k+1)==len)
	{
	debug_cec("End_of_message\n");
	//end of message (1)
	for (j=0;j<2;j++){buf_len=buf_len+1;buf[buf_len]=0;}
	for (j=0;j<6;j++){buf_len=buf_len+1;buf[buf_len]=1;}
	}
	else
	{
	//end of message (0)
	for (j=0;j<5;j++){buf_len=buf_len+1;buf[buf_len]=0;}
	for (j=0;j<3;j++){buf_len=buf_len+1;buf[buf_len]=1;}
	}

	//ack (0)
	for (j=0;j<5;j++){buf_len=buf_len+1;buf[buf_len]=0;}
	for (j=0;j<3;j++){buf_len=buf_len+1;buf[buf_len]=1;}

	}

	//3036	standby
	//3004	imageviewon
	//30820000 activesource

}


int prepare_to_buf_100us(char	*myString,unsigned long count)
{

	unsigned char len;
	int i,j,k;
	unsigned char byte;
	unsigned char buf_[10];
	unsigned int value;

	len = count / 2;
	
	i=0;k=0;
	for (j=0;j<len;j++)
        {
		buf[0]='0';
		buf[1]='x';
		buf[2]=myString[k];k=k+1;
		buf[3]=myString[k];k=k+1;
		buf[4]= '\0';
		sscanf((char*)buf, "%x", &value);
		buf_[j]=value;
	}

	buf_pos=1;
	buf_len=0;

	//start bit
	for (i=0;i<37;i++){buf_len=buf_len+1;buf[buf_len]=0;}
	for (i=0;i<8;i++){buf_len=buf_len+1;buf[buf_len]=1;}

	for (k=0;k<len;k++)
        {
	byte=buf_[k];
	debug_cec("[%d]=%x\n",k,byte);

	for (i=0;i<8;i++)
        {
		if((byte&128)==128) 
		{	//1
			for (j=0;j<6;j++){buf_len=buf_len+1;buf[buf_len]=0;}
			for (j=0;j<18;j++){buf_len=buf_len+1;buf[buf_len]=1;}
		}
		else
		{	//0
			for (j=0;j<15;j++){buf_len=buf_len+1;buf[buf_len]=0;}
			for (j=0;j<9;j++){buf_len=buf_len+1;buf[buf_len]=1;}
		}
		byte=byte<<1;
	}

	if ((k+1)==len)
	{
	debug_cec("End_of_message\n");
	//end of message (1)
	for (j=0;j<6;j++){buf_len=buf_len+1;buf[buf_len]=0;}
	for (j=0;j<18;j++){buf_len=buf_len+1;buf[buf_len]=1;}
	}
	else
	{
	//end of message (0)
	for (j=0;j<15;j++){buf_len=buf_len+1;buf[buf_len]=0;}
	for (j=0;j<9;j++){buf_len=buf_len+1;buf[buf_len]=1;}
	}

	//ack (0)
	for (j=0;j<15;j++){buf_len=buf_len+1;buf[buf_len]=0;}
	for (j=0;j<9;j++){buf_len=buf_len+1;buf[buf_len]=1;}

	}

	//3036	standby
	//3004	imageviewon
	//30820000 activesource

}

static irqreturn_t asc_irq(int irq, void *dev_id)
{
	return IRQ_HANDLED;

	if (buf_len>0)
	{
		if (buf[buf_pos]==1) 
			gpio_set_value(CEC_PIO, 1);
		else
			gpio_set_value(CEC_PIO, 0);
		buf_pos=buf_pos+1;buf_len=buf_len-1;
	}
	else {gpio_set_value(CEC_PIO, 1);writel(0,ASC_INTEN);return IRQ_HANDLED;}
	writel(0xff,ASC_TXBUF);//tx
	return IRQ_HANDLED;
}

#define CLOCKGEN_PLL1_CFG	(0xfe213000 + 0x04)
#define CONFIG_SH_EXTERNAL_CLOCK 30000000

static int get_pll_freq(unsigned long addr)
{
	unsigned long freq, data, ndiv, pdiv, mdiv;
	data = readl(addr);
	mdiv = (data >> 0) & 0x07;
	ndiv = (data >> 8) & 0xff;
	freq = ((2 * (CONFIG_SH_EXTERNAL_CLOCK / 1000) * ndiv) / mdiv) * 1000;

	return freq;
}


#endif

//power off
//echo "3036" > /proc/stb/hdmi/cec

//power on
//echo "3004" > /proc/stb/hdmi/cec

//echo "3f821100" > /proc/stb/hdmi/cec
//echo "3f821000" > /proc/stb/hdmi/cec

//3f821100 - przelaczenie na hdmi1 dziala z panasonic
//3f822100 - przelaczenie na hdmi2

//3f821000 - przelaczenie na hdmi1
//3f822000 - przelaczenie na hdmi2


int prepare_to_txcec(char	*myString,unsigned long count)
{
	unsigned char len;
	int i,j,k;
	unsigned char byte;
	unsigned char buf_[10];
	unsigned int value;
	u32 res;

	len = count / 2;
	
	i=0;k=0;
	for (j=0;j<len;j++)
        {
		buf[0]='0';
		buf[1]='x';
		buf[2]=myString[k];k=k+1;
		buf[3]=myString[k];k=k+1;
		buf[4]= '\0';
		sscanf((char*)buf, "%x", &value);
		buf_[j]=value;
		debug_cec("[%d]=%x\n",j,value);
	}

	debug_cec("Start send cec !!!\n");
	for (k=0;k<len;k++)
	{
	byte=buf_[k];
	cec_write_data((unsigned int)byte);
	if((k+1)>=len) {cec_acknowledge_eom();cec_end_sending();}else cec_acknowledge();
	if (k==0) cec_start_sending(0);
	if (wait_send_btf (50) < 0) {str_status(cec_get_status());break;}
	}
	debug_cec("Stop send cec !!!\n");

}
int proc_cec_write(struct file *file, const char __user *buf,
                           unsigned long count, void *data)
{
	char 		*page;
	char		*myString;
	ssize_t 	ret = -ENOMEM;
	
	debug_cec("%s %d - ", __FUNCTION__, (int) count);

	page = (char *)__get_free_page(GFP_KERNEL);
	if (page) 
	{
		
		
		ret = -EFAULT;
		if (copy_from_user(page, buf, count))
			goto out;

		myString = (char *) kmalloc(count + 1, GFP_KERNEL);
		strncpy(myString, page, count);
		myString[count] = '\0';

		debug_cec("%s\n", myString);
#ifdef CEC_HARDWARE
		prepare_to_txcec(myString,count);
#else
		if (wait_idle (100) < 0) goto out;
//		prepare_to_buf_100us(myString,count);
		prepare_to_buf_300us(myString,count);
		writel(2,ASC_INTEN);//int tx empty
		writel(0xff,ASC_TXBUF);//tx
#endif
		kfree(myString);
	}
	ret = count;
out:
	
	free_page((unsigned long)page);
	return ret;
}


int proc_cec_read(char *page, char **start, off_t off, int count,
			  int *eof, void *data_unused)
{
	printk("%s\n", __FUNCTION__);
        return 0;
}



struct e2_procs
{
  char *name;
  read_proc_t *read_proc;
  write_proc_t *write_proc;
} e2_procs[] =
{
  {"stb/hdmi/cec", proc_cec_read, proc_cec_write}
};

static int __init init_cec_module(void)
{
	int f,t;
	u32 res = 0;

	debug_cec("CEC Init>>\n");

	install_e2_procs(e2_procs[0].name, e2_procs[0].read_proc, e2_procs[0].write_proc, NULL);

#ifdef CEC_HARDWARE
#warning !!!!!!!!!!! CEC_HARDWARE !!!!!!!!!!!!
    res = cec_read_register_u32( SysConfigBaseAddress + SYS_CFG5 );
    // pio 1.7 pad 
    res |= ( 1 << (8) );
    cec_write_register_u32( SysConfigBaseAddress + SYS_CFG5, res );
    res = cec_read_register_u32( SysConfigBaseAddress + SYS_CFG5 );
    // pio 1.7 alt 
    res |= ( 1 << (24) );
    cec_write_register_u32( SysConfigBaseAddress + SYS_CFG5, res );

	gpio_request(CEC_PIO, "CEC_PIO");
	if (CEC_PIO==NULL){debug_cec("Request CEC_PIO failed. abort.\n");goto err;}
	gpio_direction_output(CEC_PIO, STM_GPIO_DIRECTION_ALT_BIDIR);//STM_GPIO_DIRECTION_BIDIR);//STM_GPIO_DIRECTION_OUT);

    res = cec_read_register_u32(CECBaseAddress + CEC_CFG );
    /* enable */
    res |= 0x3;
    cec_write_register_u32(CECBaseAddress + CEC_CFG, res );

    /* prescaler */ 
    cec_write_register_u32(CECBaseAddress + CEC_PRESCALER_LEFT, 0x88 );
    cec_write_register_u32(CECBaseAddress + CEC_PRESCALER_RIGHT, 0x13 );
#else
    res = cec_read_register_u32( SysConfigBaseAddress + SYS_CFG5 );
    // pio 1.7 pad 
    res &= ~( 1 << (8) );
    cec_write_register_u32( SysConfigBaseAddress + SYS_CFG5, res );
    res = cec_read_register_u32( SysConfigBaseAddress + SYS_CFG5 );
    // pio 1.7 alt 
    res &= ~( 1 << (24) );
    cec_write_register_u32( SysConfigBaseAddress + SYS_CFG5, res );

	gpio_request(CEC_PIO, "CEC_PIO");
	if (CEC_PIO==NULL){debug_cec("Request CEC_PIO failed. abort.\n");goto err;}
	gpio_direction_output(CEC_PIO, STM_GPIO_DIRECTION_BIDIR);//STM_GPIO_DIRECTION_OUT);
	gpio_set_value(CEC_PIO, 1);

	asc_registers = (unsigned long) ioremap(0xfd033000, 0x2c);
	if (request_irq(evt2irq(0x1100), (void*)asc_irq, IRQF_DISABLED, "cec", NULL)){debug_cec("FAIL : request irq asc\n");goto err;}

	f=get_pll_freq(CLOCKGEN_PLL1_CFG);
	f = (f / 4);
	debug_cec("f_system=%d\n",f);

//	t = BAUDRATE_VAL_M1(100000, f);	//10bit x 10us = 100us
	t = BAUDRATE_VAL_M1(33333, f);	//10bit x 30us = 300us

	writel(0,ASC_INTEN);
	debug_cec("baud=%d\n",t);
	writel(t,ASC_BAUDRATE);
	writel(0x1089,ASC_CTL);//8n1 off_fifo >> //0 1 0 0 0 0 1 0 0 01 001 =0x1089
	writel(0,ASC_TXRESET);
	writel(0,ASC_RXRESET);
#endif

	debug_cec("CEC Init<<\n");

err:
  return 0;
}

static void __exit cleanup_cec_module(void)
{
    remove_e2_procs(e2_procs[0].name, e2_procs[0].read_proc, e2_procs[0].write_proc);
}

module_init(init_cec_module);
module_exit(cleanup_cec_module);

MODULE_DESCRIPTION("ADB28xx CEC");
MODULE_AUTHOR("plfreebox@gmail.com");
MODULE_LICENSE("GPL");

