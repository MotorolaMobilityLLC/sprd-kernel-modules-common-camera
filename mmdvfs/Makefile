LOCAL_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/camera/mmdvfs; pwd)


# target source files
mmdvfs-objs :=

# target source files for r1p0
ifneq ("$(BSP_BOARD_CAMERA_MODULE_DVFS)","")
SRC += $(shell find $(LOCAL_PATH)/r1p0/mm_dvfs_devfreq -name "*.c")
SRC += $(shell find $(LOCAL_PATH)/r1p0/mmsys_comm -name "*.c")
SRC += $(shell find $(LOCAL_PATH)/r1p0/dvfs_driver/dvfs_driver_common -name "*.c")
SRC += $(shell find $(LOCAL_PATH)/r1p0/dvfs_driver/dvfs_reg_param/$(BSP_BOARD_CAMERA_MODULE_DVFS) -name "*.c")

LOCAL_SRC := $(SRC:$(LOCAL_PATH)/%=%)

mmdvfs-objs += $(LOCAL_SRC:%.c=%.o)
# TODO: refine these includes
ccflags-y += -I$(LOCAL_PATH)/r1p0/mm_dvfs_devfreq
ccflags-y += -I$(LOCAL_PATH)/r1p0/mmsys_comm
ccflags-y += -I$(LOCAL_PATH)/r1p0/dvfs_driver/dvfs_driver_common
ccflags-y += -I$(LOCAL_PATH)/r1p0/dvfs_driver/dvfs_reg_param/$(BSP_BOARD_CAMERA_MODULE_DVFS)
endif

ccflags-y += -I$(LOCAL_PATH)/../common/
ccflags-y += -I$(srctree)/drivers/devfreq/
# target
obj-m += mmdvfs.o

# HAPS only
ifeq ($(TEST_ON_HAPS),true)
ccflags-y += -DTEST_ON_HAPS
endif

$(warning $(CURDIR))

.PHONY:modules

modules:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@
