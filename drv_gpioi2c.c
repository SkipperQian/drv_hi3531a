#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/miscdevice.h>
#include <linux/semaphore.h>

#include "drv_gpioi2c.h"

struct gpioi2c_info hi3531a_gpioi2c[HI3531A_MAX_GPIOI2C_NUM];

struct gpioi2c_mapping
{
	unsigned int i2c_chnl;
	unsigned int scl; /*which GPIO will be defined I2CSCL*/
	unsigned int sda; /* which GPIO will be defined I2CSDA*/
	volatile unsigned char *clk_dir; /*I2CSCL direction register*/
	volatile unsigned char *data_dir; /*I2CSDA direction register*/
	volatile unsigned char *sda_reg; /*SDA GPIO  relation data buffer register,indicate high or low level about input or output(u32GpioDirData confirm the direction)*/
	volatile unsigned char *scl_reg; /*SCL GPIO indicate high or low level about input or output(u32GpioDirData confirm the direction)*/
};

enum {
	I2C_BUS_SCL,
	I2C_BUS_SDA
};

static struct gpioi2c_mapping i2c_ctrl[HI3531A_MAX_GPIOI2C_NUM];

struct semaphore gpio_i2c_sem;
static DEFINE_SPINLOCK(gpio_i2c_lock);

#define HI3531A_GPIO_BIT_NUM 8


#define HI3531A_GPIO_0_BASE_ADDR  0x12150000

#define HI3531A_GPIO_DATA 0x3fc
#define HI3531A_GPIO_DIR  0x400
#define HI3531A_GPIO_IS   0x404
#define HI3531A_GPIO_IBE  0x408
#define HI3531A_GPIO_IEV  0x40c
#define HI3531A_GPIO_IE   0x410
#define HI3531A_GPIO_RIS  0x414
#define HI3531A_GPIO_MIS  0x418
#define HI3531A_GPIO_IC   0x41c

#define HI3531A_MIN_GPIO_GROUP 0
#define HI3531A_MAX_GPIO_GROUP 24
#define HI3531A_MAX_GPIO_PIN_NUM 200 /* 25(group) * 8(pin) */

struct hi_gpio {
	volatile unsigned char __iomem *regbase;
};

static struct hi_gpio hi_gpio_info[25];

#define HW_REG(reg)         *((volatile unsigned int *)(reg))
/*
#define MUXCTRL_BASE        0x120F0000
#define MUXCTRL_REG(offset) IO_ADDRESS(MUXCTRL_BASE + (offset))

#define GPIO_BASE        0x12160000
#define GPIO_REG(offset) IO_ADDRESS(GPIO_BASE + (offset))
*/
#define GPIO_REG(base, offset) IO_ADDRESS(base + (offset))

static void i2c_clr(int i2cnum, unsigned char whichline)
{
	unsigned char val;

	if (whichline == I2C_BUS_SCL) {
		val = readl(i2c_ctrl[i2cnum].clk_dir);
		val |= i2c_ctrl[i2cnum].scl;
		writel(val, i2c_ctrl[i2cnum].clk_dir);

		writel(0, i2c_ctrl[i2cnum].scl_reg);
	} else if (whichline == I2C_BUS_SDA) {
		val = readl(i2c_ctrl[i2cnum].data_dir);
		val |= i2c_ctrl[i2cnum].sda;
		writel(val, i2c_ctrl[i2cnum].data_dir);

		writel(0, i2c_ctrl[i2cnum].sda_reg);
	} else {
		printk(KERN_ERR"i2c_clr failed, param is error (whichline:%u)\n", whichline);
	}
}

static void i2c_set(int i2cnum, unsigned char whichline)
{
	unsigned char val;

	if (whichline == I2C_BUS_SCL) {
		val = readl(i2c_ctrl[i2cnum].clk_dir);
		val |= i2c_ctrl[i2cnum].scl;
		writel(val, i2c_ctrl[i2cnum].clk_dir);

		writel(i2c_ctrl[i2cnum].scl, i2c_ctrl[i2cnum].scl_reg);
	} else if (whichline == I2C_BUS_SDA) {
		val = readl(i2c_ctrl[i2cnum].data_dir);
		val |= i2c_ctrl[i2cnum].sda;
		writel(val, i2c_ctrl[i2cnum].data_dir);

		writel(i2c_ctrl[i2cnum].sda, i2c_ctrl[i2cnum].sda_reg);
	} else {
		printk(KERN_ERR"i2c_clr failed, param is error (whichline:%u)\n", whichline);
	}
}

static unsigned char i2c_data_read(int i2cnum)
{
	unsigned char val = 0;

	val = readl(i2c_ctrl[i2cnum].data_dir);
	val &= (~i2c_ctrl[i2cnum].sda);
	writel(val, i2c_ctrl[i2cnum].data_dir);

	val = readl(i2c_ctrl[i2cnum].sda_reg);

	return val & i2c_ctrl[i2cnum].sda;
}

static void i2c_start_bit(int i2cnum)
{
	udelay(1);
	i2c_set(i2cnum, I2C_BUS_SDA);
	i2c_set(i2cnum, I2C_BUS_SCL);
	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SDA);
	udelay(2);
}

static void i2c_stop_bit(int i2cnum)
{
	i2c_clr(i2cnum, I2C_BUS_SDA);
	udelay(1);
	i2c_set(i2cnum, I2C_BUS_SCL);
	udelay(1);
	i2c_set(i2cnum, I2C_BUS_SDA);
	udelay(1);
}

static void i2c_send_byte(int i2cnum, unsigned char c)
{
	int i;
	unsigned long flag;

	spin_lock_irqsave(&gpio_i2c_lock, flag);
	for (i = 0; i < 8; i++) {
		i2c_clr(i2cnum, I2C_BUS_SCL);
		udelay(1);
		if (c & (1 << (7 - i))) {
			i2c_set(i2cnum, I2C_BUS_SDA);
		} else {
			i2c_clr(i2cnum, I2C_BUS_SDA);
		}
		udelay(1);
		i2c_set(i2cnum, I2C_BUS_SCL);	
		udelay(1);
		udelay(1);
		i2c_clr(i2cnum, I2C_BUS_SCL);
	}
	udelay(1);
	spin_unlock_irqrestore(&gpio_i2c_lock, flag);
}

static unsigned char i2c_receive_byte(int i2cnum)
{
	int i;
	unsigned char val = 0;
	unsigned long flag;

	spin_lock_irqsave(&gpio_i2c_lock, flag);
	for (i = 0; i < 8; i++) {
		udelay(1);
		i2c_clr(i2cnum, I2C_BUS_SCL);
		udelay(2);
		i2c_set(i2cnum, I2C_BUS_SCL);
		udelay(1);
		if (i2c_data_read(i2cnum)) {
			val |= (1 << (7 - i));
		}
		udelay(1);
	}
	i2c_clr(i2cnum, I2C_BUS_SCL);
	udelay(1);
	spin_unlock_irqrestore(&gpio_i2c_lock, flag);
	return val;
}

static int i2c_receive_ack(int i2cnum)
{
	unsigned char val;
	unsigned char ack;

	val = readl(i2c_ctrl[i2cnum].data_dir);
	val &= ~(i2c_ctrl[i2cnum].sda);
	writel(val, i2c_ctrl[i2cnum].data_dir);

	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SCL);
	udelay(1);
	i2c_set(i2cnum, I2C_BUS_SCL);
	udelay(1);

	ack = i2c_data_read(i2cnum);
	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SCL);
	udelay(2);

	if (ack == 0) {
		return 0;
	} else {
		printk(KERN_ERR"gpioi2c receive ack error !\n");
		return -1;
	}
}

static void i2c_send_ack(int i2cnum)
{
	unsigned char val;

	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SCL);
	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SDA);
	udelay(1);
	i2c_set(i2cnum, I2C_BUS_SCL);
	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SCL);
	udelay(1);

	val = readl(i2c_ctrl[i2cnum].data_dir);
	val &= ~(i2c_ctrl[i2cnum].sda);
	writel(val, i2c_ctrl[i2cnum].data_dir);

	udelay(2);
}

static void i2c_send_noack(int i2cnum)
{
	unsigned char val;

	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SCL);
	udelay(1);
	i2c_clr(i2cnum, I2C_BUS_SDA);
	udelay(1);
	i2c_set(i2cnum, I2C_BUS_SCL);
	udelay(2);
	i2c_clr(i2cnum, I2C_BUS_SCL);
	udelay(1);

	val = readl(i2c_ctrl[i2cnum].data_dir);
	val &= ~(i2c_ctrl[i2cnum].sda);
	writel(val, i2c_ctrl[i2cnum].data_dir);

	udelay(1);
}


static int gpioi2c_configure(unsigned int i2cnum, unsigned int which_gpioclk, unsigned int which_gpiodata, unsigned int clockbit, unsigned int databit)
{
	volatile unsigned char *basedata;
	volatile unsigned char *baseclk;

	baseclk = hi_gpio_info[which_gpioclk].regbase;
	basedata = hi_gpio_info[which_gpiodata].regbase;

	i2c_ctrl[i2cnum].i2c_chnl = i2cnum;
	i2c_ctrl[i2cnum].scl = (1 << clockbit);
	i2c_ctrl[i2cnum].sda = (1 << databit);

	i2c_ctrl[i2cnum].clk_dir = baseclk + HI3531A_GPIO_DIR;
	i2c_ctrl[i2cnum].data_dir = basedata + HI3531A_GPIO_DIR;

	i2c_ctrl[i2cnum].scl_reg = baseclk + (i2c_ctrl[i2cnum].scl << 2);
	i2c_ctrl[i2cnum].sda_reg = basedata + (i2c_ctrl[i2cnum].sda << 2);

	hi3531a_gpioi2c[i2cnum].i2c_num = i2cnum;
	hi3531a_gpioi2c[i2cnum].scl_gpio_no = which_gpioclk * HI3531A_GPIO_BIT_NUM + clockbit;
	hi3531a_gpioi2c[i2cnum].sda_gpio_no = which_gpiodata * HI3531A_GPIO_BIT_NUM + databit;
	hi3531a_gpioi2c[i2cnum].is_used = 1;
	hi3531a_gpioi2c[i2cnum].count = 1;

	printk(KERN_INFO"gpio2ic_configure() success :(i2c num=%u, scl=gpio%u_%u, sda=gpio%u_%u).\n",
		i2cnum, which_gpioclk, clockbit, which_gpiodata, databit);

	return 0;
}


int gpioi2c_read(int i2cnum, unsigned char devaddr, unsigned char reg, unsigned char *val)
{
	if (down_interruptible(&gpio_i2c_sem)) {
		printk(KERN_ERR"semaphore lock is error");
		return -1;
	}

	if (i2cnum >= HI3531A_MAX_GPIOI2C_NUM || val == NULL || !hi3531a_gpioi2c[i2cnum].is_used) {
		up(&gpio_i2c_sem);
		return -1;
	}

	i2c_start_bit(i2cnum);

	i2c_send_byte(i2cnum, devaddr);
	i2c_receive_ack(i2cnum);

	i2c_send_byte(i2cnum, reg);
	i2c_receive_ack(i2cnum);


	i2c_start_bit(i2cnum);

	i2c_send_byte(i2cnum, devaddr| 1);
	i2c_receive_ack(i2cnum);

	*val = i2c_receive_byte(i2cnum);

	i2c_stop_bit(i2cnum);

	up(&gpio_i2c_sem);
	printk(KERN_INFO"i2c num:%d, dev addr:0x%x, register:0x%x, value:0x%x\n",
		i2cnum, devaddr, reg, *val);

	return 0;
}

int gpioi2c_write(int i2cnum, unsigned char devaddr, unsigned char reg, unsigned char val)
{
	if (down_interruptible(&gpio_i2c_sem)) {
		printk(KERN_ERR"semaphore lock is error\n");
		return -1;
	}

	if (i2cnum >= HI3531A_MAX_GPIOI2C_NUM || !hi3531a_gpioi2c[i2cnum].is_used) {
		up(&gpio_i2c_sem);
		return -1;
	}

	printk(KERN_INFO"i2c num:%d, dev addr:0x%x, register:0x%x, value:0x%x\n",
		i2cnum, devaddr, reg, val);

	i2c_start_bit(i2cnum);

	i2c_send_byte(i2cnum, devaddr);
	i2c_receive_ack(i2cnum);

	i2c_send_byte(i2cnum, reg);
	i2c_receive_ack(i2cnum);

	i2c_send_byte(i2cnum, val);
	i2c_receive_ack(i2cnum);

	up(&gpio_i2c_sem);

	return 0;
}

int gpioi2c_create(unsigned int *gpioi2c_no, unsigned int scl_gpio_no, unsigned int sda_gpio_no)
{
	unsigned int i;
	unsigned int valid_i2cnum;
	unsigned int i2cclk_gpiogroup;
	unsigned int i2cclk_gpiobit;
	unsigned int i2cdata_gpiogroup;
	unsigned int i2cdata_gpiobit;

	i2cclk_gpiogroup = scl_gpio_no / HI3531A_GPIO_BIT_NUM;
	i2cclk_gpiobit = scl_gpio_no % HI3531A_GPIO_BIT_NUM;
	i2cdata_gpiogroup = sda_gpio_no / HI3531A_GPIO_BIT_NUM;
	i2cdata_gpiobit = sda_gpio_no % HI3531A_GPIO_BIT_NUM;
	if (i2cclk_gpiogroup > HI3531A_MAX_GPIO_GROUP || i2cdata_gpiogroup > HI3531A_MAX_GPIO_GROUP) {
		printk(KERN_ERR "error: GPIO NO. %d and NO. %d is invalid!\n", scl_gpio_no, sda_gpio_no);
		return -1;
	}


	for (i = 0; i < HI3531A_MAX_GPIOI2C_NUM; i++) {
		if (hi3531a_gpioi2c[i].is_used) {
			if (scl_gpio_no == hi3531a_gpioi2c[i].scl_gpio_no && sda_gpio_no == hi3531a_gpioi2c[i].sda_gpio_no) {
				hi3531a_gpioi2c[i].count++;
				*gpioi2c_no = hi3531a_gpioi2c[i].i2c_num;
				return 0;
			}
			if (scl_gpio_no == hi3531a_gpioi2c[i].scl_gpio_no || sda_gpio_no == hi3531a_gpioi2c[i].scl_gpio_no) {
				printk(KERN_ERR"GPIO NO. %u is used to GpioClock!\n", hi3531a_gpioi2c[i].scl_gpio_no);
				return -1;
			}
			if (scl_gpio_no == hi3531a_gpioi2c[i].sda_gpio_no || sda_gpio_no == hi3531a_gpioi2c[i].sda_gpio_no) {
				printk(KERN_ERR"GPIO NO. %u is used to GpioData!\n", hi3531a_gpioi2c[i].sda_gpio_no);
				return -1;
			}
		} else {
			valid_i2cnum = i;
			break;
		}
	}

	if (i >= HI3531A_MAX_GPIOI2C_NUM) {
		printk(KERN_ERR"i2c channel all  have used ,request i2c channel fail  !\n");
		return -1;
	}

	gpioi2c_configure(valid_i2cnum, i2cclk_gpiogroup, i2cdata_gpiogroup, i2cclk_gpiobit, i2cdata_gpiobit);

	*gpioi2c_no = valid_i2cnum;

	return 0;
}

static long gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long ret = 0;
	unsigned int __user *argp = (unsigned int __user *)arg;
	struct gpio_regdata_t reg;
	struct gpioi2c_info gpioi2c;
	unsigned char gpio_group;
	struct i2c_data i2c_reg;

	switch (cmd) {
	case CMD_GPIO_READ_BIT:
		if (copy_from_user(&reg, argp, sizeof(reg))) {
			printk(KERN_ERR "copy data from user fail!\n");
			ret = -1;
			break;
		}
		if (reg.gpiono > HI3531A_MAX_GPIO_PIN_NUM)
			return -1;

		gpio_group = reg.gpiono / 8;
		reg.value = readl(hi_gpio_info[gpio_group].regbase + HI3531A_GPIO_DATA);

		if (copy_to_user(argp, &reg, sizeof(reg))) {
			printk(KERN_ERR "copy data to user fail!\n");
			ret = -1;
		}
		break;
	case CMD_GPIO_WRITE_BIT:
		if (copy_from_user(&reg, argp, sizeof(reg))) {
			printk(KERN_ERR "copy data from user fail!\n");
			ret = -1;
			break;
		}
		if (reg.gpiono > HI3531A_MAX_GPIO_PIN_NUM)
			return -1;

		gpio_group = reg.gpiono / 8;
		writel(reg.value, hi_gpio_info[gpio_group].regbase + HI3531A_GPIO_DATA);
		break;
	case CMD_GPIOI2C_CREATE:
		if (copy_from_user(&gpioi2c, argp, sizeof(gpioi2c))) {
			printk(KERN_ERR" copy data from user failed!\n");
			ret = -1;
			break;
		}
		if (gpioi2c_create(&(gpioi2c.i2c_num), gpioi2c.scl_gpio_no, gpioi2c.sda_gpio_no) == 0) {
			if (copy_to_user(argp, &gpioi2c, sizeof(gpioi2c))) {
				printk(KERN_ERR "copy data to user fail!\n");
				ret = -1;
			}
		}
		break;
	case CMD_GPIOI2C_READ:
		if (copy_from_user(&i2c_reg, argp, sizeof(i2c_reg))) {
			printk(KERN_ERR "copy data from user fail!\n");
			ret = -1;
			break;
		}
		if (i2c_reg.i2c_num >= HI3531A_MAX_GPIOI2C_NUM)
			return -1;
		
		gpioi2c_read(i2c_reg.i2c_num, i2c_reg.dev_addr, i2c_reg.reg_addr, &i2c_reg.reg_val);

		if (copy_to_user(argp, &i2c_reg, sizeof(i2c_reg))) {
			printk(KERN_ERR "copy data to user fail!\n");
			ret = -1;
		}
		break;
	case CMD_GPIOI2C_WRITE:
		if (copy_from_user(&i2c_reg, argp, sizeof(i2c_reg))) {
			printk(KERN_ERR "copy data from user fail!\n");
			ret = -1;
			break;
		}
		if (i2c_reg.i2c_num >= HI3531A_MAX_GPIOI2C_NUM)
			return -1;
		
		gpioi2c_write(i2c_reg.i2c_num, i2c_reg.dev_addr, i2c_reg.reg_addr, i2c_reg.reg_val);
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

static int gpio_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int gpio_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static struct file_operations gpio_fops =
{
	.owner = THIS_MODULE,
	.unlocked_ioctl = gpio_ioctl,
	.open = gpio_open,
	.release = gpio_close
};

static struct miscdevice gpio_dev =
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hi_gpioi2c",
	.fops = &gpio_fops,
};

static int __init hi3531a_gpioi2c_init(void)
{
	int ret;
	int i;

	ret = misc_register(&gpio_dev);
	if (ret)
		printk(KERN_ERR "register misc dev for hi_gpio failed !\n");

	for (i = 0; i < 25; i++) {
		hi_gpio_info[i].regbase = (volatile unsigned char __iomem *)IO_ADDRESS(HI3531A_GPIO_0_BASE_ADDR + (i * 0x10000));
	}

	sema_init(&gpio_i2c_sem, 1);

	return ret;
}

static void __exit hi3531a_gpioi2c_exit(void)
{
	misc_deregister(&gpio_dev);
}

module_init(hi3531a_gpioi2c_init);
module_exit(hi3531a_gpioi2c_exit);
