ROOT_DIR := $(notdir $(CURDIR))
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

HOST_MKIFS := mkifs

SUFFIXES := .build .ifs .raw

.PHONY: all clean

all: ifs-ti-am335x-beaglebone.bin ifs-ti-am335x-beaglebone-graphics.bin
wt512ram_edlcd: 512_edlcd_qnx-ifs
clean:
	$(RM_HOST) ifs-ti-am335x-beaglebone.bin ifs-ti-am335x-beaglebone-graphics.bin

ifs-ti-am335x-beaglebone.bin: beaglebone.build
	$(HOST_MKIFS) -r../install -v $(MKIFSFLAGS) $^ $@

ifs-ti-am335x-beaglebone-graphics.bin: ti-am335x-beaglebone-graphics.build
	$(HOST_MKIFS) -r../install -v $(MKIFSFLAGS) $^ $@