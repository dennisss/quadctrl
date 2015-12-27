EXTERNAL:= $(call my-dir)/../../../../ext

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

#OPENCV_CAMERA_MODULES := off
#OPENCV_INSTALL_MODULES := off
#OPENCV_LIB_TYPE := SHARED
#include $(EXTERNAL)/opencv/sdk/native/jni/OpenCV.mk


LOCAL_MODULE := libquadctrl
LOCAL_C_INCLUDES := $(EXTERNAL)/eigen
LOCAL_SHARED_LIBRARIES := libftdi libusb1.0 roscpp_android_ndk
LOCAL_CFLAGS := -std=c++11 -O3
LOCAL_SRC_FILES := node.cpp \
                   quadcopter.cpp \
                   usb_serial.cpp \
                   motors.cpp \
                   MadgwickAHRS.cpp \
                   inertial.cpp

include $(BUILD_SHARED_LIBRARY)


# Build libusb
include $(EXTERNAL)/libusb/Android.mk #android/jni/libusb.mk

# Build libftdi
include $(EXTERNAL)/libftdi/Android.mk

# Prebuilt ROS packages
include $(EXTERNAL)/roscpp_android_ndk/Android.mk
