#ifneq ($(strip $(TARGET_BOARD_CAMERA_SENSOR_MODULE)),)

LOCAL_PATH := $(PWD)

# target source files
flash_ic_sc2721-objs := sc2721s_drv.o

# target
obj-m += flash_ic_sc2721.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../flash_drv
ccflags-y += -I$(LOCAL_PATH)/../../interface

.PHONY:modules

modules:
	$(MAKE) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) $(BSP_MAKE_EXTRA_ARGS) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	$(MAKE) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) $(BSP_MAKE_EXTRA_ARGS) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	$(MAKE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

#endif
