SI_WORKAROUNDS := -DB0_SI_SOLN_RP

EXTRA_CFLAGS += -Idrivers/pci/ntb/common -DB0_SI_SOLN_CL

obj-$(CONFIG_RP_NTB) += rpntb.o
rpntb-objs := ntb_main.o ../common/ntb_lib.o ../common/ntb_common.o \
	../common/ntb_device.o
