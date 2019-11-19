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
	$(MAKE) $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) $(BSP_MAKE_EXTRA_ARGS) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	$(MAKE) $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) $(BSP_MAKE_EXTRA_ARGS) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	$(MAKE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

#endif
