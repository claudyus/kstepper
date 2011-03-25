#obj-m := stepper-drv.o
obj-m := stepper-nopwm.o

KERNELDIR?= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)

all default:
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(PWD) modules

