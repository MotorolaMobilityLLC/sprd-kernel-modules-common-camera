LOCAL_PATH := $(PWD)
ISP_DIR := $(BSP_BOARD_CAMERA_MODULE_ISP_VERSION)

sprd_camera-objs :=

# target source files
SRC := $(shell find $(LOCAL_PATH)/$(ISP_DIR) -name "*.c")
LOCAL_SRC := $(SRC:$(LOCAL_PATH)/%=%)
sprd_camera-objs += $(LOCAL_SRC:%.c=%.o)

# target
obj-m += sprd_camera.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/../interface
ccflags-y += -I$(LOCAL_PATH)/../sensor
ccflags-y += -I$(LOCAL_PATH)/../flash/flash_drv
ccflags-y += -I$(LOCAL_PATH)/$(ISP_DIR)/inc
ccflags-y += -I$(BSP_KERNEL_PATH)/drivers/staging/android/ion

.PHONY:modules

modules:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

