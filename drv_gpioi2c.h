#ifndef HI3531A_DRV_GPIOI2C_H
#define HI3531A_DRV_GPIOI2C_H


#define HI3531A_MAX_GPIOI2C_NUM 2


struct i2c_data {
	int i2c_num;
	unsigned char dev_addr;
	unsigned char reg_addr;
	unsigned char reg_val;
};

struct gpioi2c_info {
    unsigned int i2c_num;
    unsigned int scl_gpio_no;
    unsigned int sda_gpio_no;
    unsigned int is_used;
    unsigned int count;
};

struct gpio_regdata_t {
	unsigned char gpiono;
	unsigned char value;
};


#define HI3531A_GPIO_MAGIC 'w'

#define CMD_GPIO_READ_BIT _IOWR(HI3531A_GPIO_MAGIC, 0x0, struct gpio_regdata_t)
#define CMD_GPIO_WRITE_BIT _IOW(HI3531A_GPIO_MAGIC, 0x1, struct gpio_regdata_t)
#define CMD_GPIOI2C_CREATE _IOW(HI3531A_GPIO_MAGIC, 0x2, struct gpioi2c_info)
#define CMD_GPIOI2C_READ _IOWR(HI3531A_GPIO_MAGIC, 0x3, struct i2c_data)
#define CMD_GPIOI2C_WRITE _IOW(HI3531A_GPIO_MAGIC, 0x4, struct i2c_data)

#endif
