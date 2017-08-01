ifeq ($(PARAM_FILE), )
	PARAM_FILE:=../../Makefile.param
	include $(PARAM_FILE)
endif

obj-m := drv_gpioi2c.o
#tp_2802-y += drv_gpioi2c.o
EXTRA_CFLAGS += -I$(REL_INC)
EXTRA_CFLAGS += $(DRV_CFLAGS)

default:
	make -C $(LINUX_ROOT) M=$(PWD) modules 
	#cp tp2802.h $(REL_INC)
	cp *.ko  $(REL_KO)/extdrv/
clean: 
	make -C $(LINUX_ROOT) M=$(PWD) clean



