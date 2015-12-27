LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libusb1.0
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)
LOCAL_SRC_FILES := core.c \
                   descriptor.c \
                   hotplug.c \
                   io.c \
                   sync.c \
                   strerror.c \
                   os/linux_usbfs.c \
                   os/poll_posix.c \
                   os/threads_posix.c \
                   os/linux_netlink.c

include $(BUILD_SHARED_LIBRARY)
