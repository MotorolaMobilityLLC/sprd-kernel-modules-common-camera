#ifneq ($(strip $(TARGET_BOARD_CAMERA_SENSOR_MODULE)),)

LOCAL_PATH := $(PWD)

# target source files
sprd_flash_drv-objs := flash_drv.o

# target
obj-m += sprd_flash_drv.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../../interface

.PHONY:modules

modules:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

#endif
