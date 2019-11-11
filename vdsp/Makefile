LOCAL_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/camera/vdsp; pwd)
VDSP_DIR := $(BSP_BOARD_CAMERA_MODULE_VDSP_DEVICE)

ifeq ($(BSP_BOARD_CAMERA_MODULE_VDSP_DEVICE),Ceva)
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

