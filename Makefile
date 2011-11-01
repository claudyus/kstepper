obj-m := stepper.o

KERNELDIR?= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)

all default:
	$(MAKE) -C $(KERNELDIR) SUBDIRS=$(PWD) modules

