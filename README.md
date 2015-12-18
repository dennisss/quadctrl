Quadcopter Controller
=====================

- Disclaimer: this is active research code; safety and functionality are not guranteed. Please observe caution while testing.
- Android Phone powered Quadcopter (with ROS for remote control)
- Compatible with Android 6.0 stock. No root required.
- Connects to motors via a USB cable connected to an Arduino (see arduino/arduino.ino)
- Requires rosjava with android_core to be installed on the computer (you need to change app/build.gradle to point to these packages)
- Uses custom versions of libftdi and libusb (included)

- For compiling, you need to separately compile `roscpp_android_ndk` and place it in the `ext` folfer
