#ifneq ($(strip $(TARGET_BOARD_CAMERA_SENSOR_MODULE)),)

LOCAL_PATH := $(PWD)

# target source files
flash_ic_sc2703-objs := sc2703s_drv.o

# target
obj-m += flash_ic_sc2703.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../flash_drv
ccflags-y += -I$(LOCAL_PATH)/../../interface

.PHONY:modules

modules:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

#endif
