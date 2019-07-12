LOCAL_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/camera/sensor; pwd)
CSI_DIR := $(BSP_BOARD_CAMERA_MODULE_CSI_VERSION)

# target source files
sprd_sensor-objs := sprd_sensor_core.o \
				sprd_sensor_drv.o

# compat ioctl for 32-bit user
ifeq ($(CONFIG_COMPAT),y)
sprd_sensor-objs += compat_sensor_drv.o
endif

CSI_SRC := $(shell find $(LOCAL_PATH)/csi2/sprd/$(CSI_DIR) -name "*.c")
LOCAL_CSI_SRC := $(CSI_SRC:$(LOCAL_PATH)/%=%)
sprd_sensor-objs += $(LOCAL_CSI_SRC:%.c=%.o)

# target
obj-m += sprd_sensor.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/../interface
ccflags-y += -I$(LOCAL_PATH)/csi2/sprd/$(CSI_DIR)

.PHONY:modules

modules:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

