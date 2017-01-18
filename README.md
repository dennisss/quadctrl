Quadcopter Controller
=====================

**Deprecated: I'm keeping this up as it contains many good notes and code snippets for Android. Should you want a more complete and maintained platform, see https://github.com/dennisss/tansa**


TODO:
-----

- Move networking/xbee code out of 'creator'
- Abstract remote quadcopters as a ros node with a common command interface
- So, say have a Quadcopter class: a RosQuadcopter would be an implementation of that


Overview
--------

- An implementation of many common quadcopter control and modeling things

- An onboard controller with XBee support

- A GUI for controlling quadcopters from an XBee connected computer

- Support for motion capture systems
	- Drivers included for OptiTrack Motive

- Position and attitude control based on
	- `Minimum Snap Trajectory Generation and Control for Quadrotors` by Daniel Mellinger and Vijay Kumar




Useful Commands
---------------

- For creating an access point on a linux machine (requires create\_ap)
	- `sudo create_ap -n wlp3s0 MyAP helloworld`


- Starting up the controller
	- `ROS_IP=192.168.12.1 ROS_HOSTNAME=192.168.12.1 ./scripts/keyboard_ctrl.py`


- Checkout "Sharing internet connection over Ethernet" under https://wiki.archlinux.org/index.php/NetworkManager

	Scenario: your device has internet connection over wi-fi and you want to share the internet connection to other devices over ethernet.
	Requirements:
	Install the dnsmasq package to be able to actually share the connection.
	You internet connected device and the other devices are connected over a suitable ethernet cable (this usually means a cross over cable or a switch in between).
	Steps:
	Run nm-connection-editor from terminal.
	Add a new ethernet connection.
	Give it some sensible name. For example "Shared Internet"
	Go to "IPv4 Settings".
	For "Method:" select "Shared to other computers".
	Save
	Now you should have a new option "Shared Internet" under the Wired connections in NetworkManager.



- See `ext/cores/teensy3` for Teensy3 makefile example


- Programming AfroESCs: `make program_tgy_afro_nfet.0` via usb pwm programmer


- A good read for multicast implementation
	- http://www.tldp.org/HOWTO/Multicast-HOWTO-6.html


Builds
------

Below are descriptions of each of the builds:

- `dancer`
	- High performance acrobatic quadrotor for perfroming live performances as a dancer
	- Currently uses a Teensy 3 as an onboard controller with I2C
		- Teensy libraries need to be downloaded @ https://github.com/PaulStoffregen/cores
			- On Archlinux this requires the `arm-none-eabi-gcc` package for compiling
		- ESCs used are the AfroESC 20A slims with I2C wiring
			- Teensy I2C Lib: https://forum.pjrc.com/threads/21680-New-I2C-library-for-Teensy3
		- Firmware for ESCS: https://github.com/balrog-kun/tgy
			- http://www.rcgroups.com/forums/showthread.php?t=1955848
			- http://www.rcgroups.com/forums/showthread.php?t=2032252



Libraries Used
--------------


- XBee Support via `https://github.com/attie/libxbee3`
	- Needs to be compiled with escaping mode enabled
- Arduino/Teensy version @ `https://github.com/andrewrapp/xbee-arduino`
- I2C Library: `https://github.com/jrowberg/i2cdevlib.git`
- I2C Teensy Addon: `https://github.com/nox771/i2c_t3`

- OGRE into QT5
	- http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Integrating+Ogre+into+QT5




Drivers
-------

- OptiTrack
	- Included is a driver for interfacing with OptiTrack Motive 1.9.0 via the NatNet SDK 2.9.0
	- Unix implementation of the NatNetClient with same interface as the SDK



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

- TODO: Add video streaming
	- Combine the native mvision code using the capture request to go to many places.
	- One of which should be like https://github.com/fyhertz/libstreaming
