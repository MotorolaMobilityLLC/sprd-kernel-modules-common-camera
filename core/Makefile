LOCAL_PATH := $(shell cd $(BSP_ROOT_DIR)/modules/camera/core; pwd)
ISP_DIR := $(BSP_BOARD_CAMERA_MODULE_ISP_VERSION)
CSI_DIR := $(BSP_BOARD_CAMERA_MODULE_CSI_VERSION)
ADAPT_DIR := $(BSP_BOARD_CAMERA_MODULE_ISP_ADAPT_VERSION)

sprd_camera-objs :=

# target source files
ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_VERSION),dcam_r6p0_isp_r6p91)
SRC := $(shell find $(LOCAL_PATH)/$(ISP_DIR) -name "*.c")
LOCAL_SRC := $(SRC:$(LOCAL_PATH)/%=%)
sprd_camera-objs += $(LOCAL_SRC:%.c=%.o)
else
COMMON_SRC := $(wildcard $(LOCAL_PATH)/common/*.c)
COMMON_DIR := $(notdir $(COMMON_SRC))
COMMON_OBJ := $(patsubst %c, common/%o, $(COMMON_DIR))

CAM_SRC := $(wildcard $(LOCAL_PATH)/$(ISP_DIR)/src/*.c)
CAM_DIR := $(notdir $(CAM_SRC))
CAM_OBJ := $(patsubst %c, $(ISP_DIR)/src/%o, $(CAM_DIR))

ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_VERSION),isp2.6)
CAM_BLOCK_SRC := $(wildcard $(LOCAL_PATH)/$(ISP_DIR)/adpt/$(ADAPT_DIR)/block/*.c)
CAM_BLOCK_DIR := $(notdir $(CAM_BLOCK_SRC))
CAM_BLOCK_OBJ := $(patsubst %c, $(ISP_DIR)/adpt/$(ADAPT_DIR)/block/%o, $(CAM_BLOCK_DIR))

CAM_ADPT_SRC := $(wildcard $(LOCAL_PATH)/$(ISP_DIR)/adpt/$(ADAPT_DIR)/*.c)
CAM_ADPT_DIR := $(notdir $(CAM_ADPT_SRC))
CAM_ADPT_OBJ := $(patsubst %c, $(ISP_DIR)/adpt/$(ADAPT_DIR)/%o, $(CAM_ADPT_DIR))
else
CAM_BLOCK_SRC := $(wildcard $(LOCAL_PATH)/$(ISP_DIR)/block/*.c)
CAM_BLOCK_DIR := $(notdir $(CAM_BLOCK_SRC))
CAM_BLOCK_OBJ := $(patsubst %c, $(ISP_DIR)/block/%o, $(CAM_BLOCK_DIR))
endif

CAM_COMPAT_SRC := $(wildcard $(LOCAL_PATH)/$(ISP_DIR)/compat/*.c)
CAM_COMPAT_DIR := $(notdir $(CAM_COMPAT_SRC))
CAM_COMPAT_OBJ := $(patsubst %c, $(ISP_DIR)/compat/%o, $(CAM_COMPAT_DIR))

ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_VERSION),dcam_if_r4p0_isp_r6p11)
sprd_camera-objs += $(COMMON_OBJ)
endif

ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_VERSION),isp2.6)
sprd_camera-objs += $(CAM_ADPT_OBJ)
endif

sprd_camera-objs += $(CAM_OBJ) $(CAM_BLOCK_OBJ)
endif

ifeq ($(CONFIG_COMPAT),y)
sprd_camera-objs += $(CAM_COMPAT_OBJ)
endif

# target
obj-m += sprd_camera.o

# target includes
ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_VERSION),dcam_if_r4p0_isp_r6p11)
ccflags-y += -I$(LOCAL_PATH)/common
endif
ccflags-y += -I$(LOCAL_PATH)/../common
ccflags-y += -I$(LOCAL_PATH)/../interface
ccflags-y += -I$(LOCAL_PATH)/../sensor
ccflags-y += -I$(LOCAL_PATH)/../sensor/csi2/sprd/$(CSI_DIR)
ccflags-y += -I$(LOCAL_PATH)/../flash/flash_drv
ccflags-y += -I$(LOCAL_PATH)/$(ISP_DIR)/inc
ccflags-y += -I$(BSP_KERNEL_PATH)/drivers/staging/android/ion
ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_VERSION),dcam_r6p0_isp_r6p91)
ccflags-y += -I$(LOCAL_PATH)/$(ISP_DIR)/common
endif

ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_VERSION),isp2.6)
ccflags-y += -I$(LOCAL_PATH)/$(ISP_DIR)/adpt/$(ADAPT_DIR)/inc
endif

ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_ADAPT_VERSION), sharkl3)
ccflags-y += -DPROJ_SHARKL3
else ifeq ($(BSP_BOARD_CAMERA_MODULE_ISP_ADAPT_VERSION), sharkl5pro)
ccflags-y += -DPROJ_SHARKL5PRO
endif

.PHONY:modules

modules:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

modules_install:
	make $(BSP_MAKE_EXTRA_ARGS) ARCH=$(BSP_BOARD_ARCH) CROSS_COMPILE=$(BSP_KERNEL_CROSS_COMPILE) -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` INSTALL_MOD_PATH=$(BSP_MODULES_OUT) $@

clean:
	make -C $(BSP_KERNEL_PATH) O=$(BSP_KERNEL_OUT) M=`pwd` $@

