# dkms passes KERNELRELEASE on the command line; fall back to the running
# kernel when invoked manually.
ifeq ($(KERNELRELEASE),)
KERNELRELEASE := $(shell uname -r)
endif
KDIR := /lib/modules/$(KERNELRELEASE)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

install:
	install -d -m 0755 $(DESTDIR)/lib/modules/$(kernelver)/updates/dkms
	install -m 0644 xpg-fusion.ko $(DESTDIR)/lib/modules/$(kernelver)/updates/dkms/

remove:
	rm -f $(DESTDIR)/lib/modules/$(kernelver)/updates/dkms/xpg-fusion.ko
