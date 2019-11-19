LOCAL_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/camera/fd; pwd)
FD_DIR := $(BSP_BOARD_CAMERA_MODULE_FD_VERSION)

# target source files
sprd_fd-objs :=

SRC := $(shell find $(LOCAL_PATH)/$(FD_DIR) -name "*.c")
LOCAL_SRC := $(SRC:$(LOCAL_PATH)/%=%)
sprd_fd-objs += $(LOCAL_SRC:%.c=%.o)

# target
obj-m += sprd_fd.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../core/isp2.6/adpt/sharkl5pro/inc
ccflags-y += -I$(LOCAL_PATH)/../core/isp2.6/inc/
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/../interface
ccflags-y += -I$(BSP_KERNEL_PATH)/drivers/staging/android/ion

.PHONY:modules

modules:
	$(MAKE) $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	$(MAKE) $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	$(MAKE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

