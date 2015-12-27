Quadcopter Controller
=====================

- An implementation of many common quadcopter control and modeling things

- Position and attitude control based on
	- `Minimum Snap Trajectory Generation and Control for Quadrotors` by Daniel Mellinger and Vijay Kumar



Android
-------

Included is a fully functional Android implementation of the controller which can turn any smartphone into a quadcopter controller (with some additional USB hardware).

- Disclaimer: this is active research code; safety and functionality are not guranteed. Please observe caution while testing.
- Android Phone powered Quadcopter (with ROS for remote control)
- Compatible with Android 6.0 stock. No root required.
- Connects to motors via a USB cable connected to an Arduino (see arduino/arduino.ino)
- Requires rosjava with android_core to be installed on the computer (you need to change app/build.gradle to point to these packages)
- Uses custom versions of libftdi and libusb (included)

- For compiling, you need to separately compile `roscpp_android_ndk` and place it in the `ext` folfer
