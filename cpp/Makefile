LOCAL_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/camera/cpp; pwd)
CPP_DIR := $(BSP_BOARD_CAMERA_MODULE_CPP_VERSION)

sprd_cpp-objs :=

# target source files
SRC := $(shell find $(LOCAL_PATH)/$(CPP_DIR) -name "*.c")
LOCAL_SRC := $(SRC:$(LOCAL_PATH)/%=%)
sprd_cpp-objs += $(LOCAL_SRC:%.c=%.o)

# target
obj-m += sprd_cpp.o

# target includes
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/../interface

.PHONY:modules

modules:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_MODULES_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

