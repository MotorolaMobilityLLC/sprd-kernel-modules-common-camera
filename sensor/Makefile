LOCAL_PATH := $(PWD)

ifeq ($(BSP_BOARD_CAMERA_MODULE_CSI_VERSION),r2p0)
CSI_DIR := receiver_r2p0
endif

# target source files
sprd_sensor-objs := sprd_sensor_core.o \
				sprd_sensor_drv.o \
				compat_sensor_drv.o

CSI_SRC := $(shell find $(LOCAL_PATH)/csi2/csi_sprd/$(CSI_DIR) -name "*.c")
LOCAL_CSI_SRC := $(CSI_SRC:$(LOCAL_PATH)/%=%)
sprd_sensor-objs += $(LOCAL_CSI_SRC:%.c=%.o)

# target
obj-m += sprd_sensor.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/../interface
ccflags-y += -I$(LOCAL_PATH)/csi2/csi_sprd/$(CSI_DIR)

.PHONY:modules

modules:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

