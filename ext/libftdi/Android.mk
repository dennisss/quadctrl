LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libftdi
LOCAL_SHARED_LIBRARIES += libusb1.0
#LOCAL_C_INCLUDES += $(LIBUSB_ROOT_ABS)
#LOCAL_CFLAGS := -std=c++11 -O3
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
LOCAL_SRC_FILES := ftdi.c \
                   ftdi_stream.c
include $(BUILD_SHARED_LIBRARY)
