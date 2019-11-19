#ifneq ($(strip $(TARGET_BOARD_CAMERA_SENSOR_MODULE)),)

LOCAL_PATH := $(PWD)

# target source files
flash_ic_sy7807-objs := flash_ic_sy7807_drv.o

# target
obj-m += flash_ic_sy7807.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../flash_drv
ccflags-y += -I$(LOCAL_PATH)/../../interface

.PHONY:modules

modules:
	$(MAKE) $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	$(MAKE) $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	$(MAKE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

#endif
