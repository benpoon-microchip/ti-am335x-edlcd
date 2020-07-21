ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

include ../../../prodroot_pkt.mk
include tgt.mk

LIBS = netdrvrS cacheS drvr cam

NAME = devnp-$(PROJECT)

define PINFO
PINFO DESCRIPTION=dm814x ethernet driver
endef


#####AUTO-GENERATED by packaging script... do not checkin#####
   INSTALL_ROOT_nto = $(PROJECT_ROOT)/../../../../install
   USE_INSTALL_ROOT=1
##############################################################

include $(MKFILES_ROOT)/qtargets.mk
-include $(SECTION_ROOT)/extra_libs.mk
-include $(SECTION_ROOT)/$(CPU)/extra_libs.mk

