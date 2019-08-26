LOCAL_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/camera/vdsp; pwd)
VDSP_DIR := $(BSP_BOARD_CAMERA_MODULE_VDSP_DEVICE)

ifeq ($(BSP_BOARD_CAMERA_MODULE_VDSP_DEVICE),Cadence)
sprd_vdsp-objs :=
vdsp_sipc-objs :=
vdsp_spipe-objs :=

# target source files
XRP_SRC := $(wildcard $(LOCAL_PATH)/$(VDSP_DIR)/xrp/*.c)
XRP_DIR := $(notdir $(XRP_SRC))
XRP_OBJ := $(patsubst %.c, $(VDSP_DIR)/xrp/%.o, $(XRP_DIR))
sprd_vdsp-objs += $(XRP_OBJ)

SIPC_SRC := $(shell find $(LOCAL_PATH)/$(VDSP_DIR)/uipc/sipc/src -name "*.c")
SIPC_DIR :=  $(SIPC_SRC:$(LOCAL_PATH)/$(VDSP_DIR)/uipc/sipc/src/%=%)
vdsp_sipc-objs += $(SIPC_DIR:%.c=$(VDSP_DIR)/uipc/sipc/src/%.o)

SPIPE_SRC := $(shell find $(LOCAL_PATH)/$(VDSP_DIR)/uipc/spipe/src -name "*.c")
SPIPE_DIR :=  $(SPIPE_SRC:$(LOCAL_PATH)/$(VDSP_DIR)/uipc/spipe/src/%=%)
vdsp_spipe-objs += $(SPIPE_DIR:%.c=$(VDSP_DIR)/uipc/spipe/src/%.o)

#target
obj-m += sprd_vdsp.o
obj-m += vdsp_sipc.o
obj-m += vdsp_spipe.o

# target includes
ccflags-y += -DDEBUG
ccflags-y += -DKERNEL_VERSION_414
ccflags-y += -I$(LOCAL_PATH)/../interface
ccflags-y += -I$(BSP_KERNEL_PATH)/drivers/staging/android/ion
ccflags-y += -I$(BSP_KERNEL_PATH)/drivers/devfreq/apsys/
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/Cadence/xrp
ccflags-y += -I$(BSP_ROOT_DIR)/modules/camera/vdsp/Cadence/uipc/sipc/inc
ifeq ($(CONFIG_COMPAT),y)
ccflags-y += -DCONFIG_COMPAT
endif
ifeq ($(TEST_ON_HAPS),true)
ccflags-y += -DTEST_ON_HAPS
endif

else ifeq ($(BSP_BOARD_CAMERA_MODULE_VDSP_DEVICE),Ceva)
sprd_vdsp-objs :=

# target source files
CEVA_SRC := $(shell find $(LOCAL_PATH)/$(VDSP_DIR)/ceva1.0/src -name "*.c")
CEVA_DIR :=  $(CEVA_SRC:$(LOCAL_PATH)/$(VDSP_DIR)/ceva1.0/src/%=%)
sprd_vdsp-objs += $(CEVA_DIR:%.c=$(VDSP_DIR)/ceva1.0/src/%.o)

#target
obj-m += sprd_vdsp.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/$(VDSP_DIR)/ceva1.0/inc/
ccflags-y += -I$(LOCAL_PATH)/../sensor/
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/../interface
ccflags-y += -I$(BSP_KERNEL_PATH)/drivers/staging/android/ion
ccflags-y += -I$(BSP_KERNEL_PATH)/drivers/devfreq/apsys/
else
$(warning "unknown vdsp device!!!!!!!!!!!!!!!!!!!!!!!!!!")
endif

.PHONY:modules

modules:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

