QuadCtrl Hardware
=================

Motor Configuration
-------------------

- Motor 1 is in the top left and should spin clockwise. The numbering of motors is clockwise (motor 4 is bottom left).




Interface Board
---------------

A USB board used to control the motors, and read any external sensors. It uses an 115200 baud serial connection over USB to transmit motor commands.

- See the included Eagle CAD schematics in this folder
	- Designed for mounting under DJI F450 top plate
	- It includes circuitry for measuring rough 3S lipo battery voltage via an Arduino analog pin
	- Note: the throughhole components should be mounted on the bottom side
	- Uses an Adafruit Metro Mini
	- Connect the jumper to power the USB device with the UBEC

- Some implementation notes
	- Kills motors if no serial motor command is received within < 1s. It expects a > 100Hz motor update rate from the controller.
	- To prevent accidental re-arming of the motors, the board is powered by an UBEC connected to the motor battery. This way the ESCs are armed exactly once at battery connection and stay ready after plugging/unpluging phones. Note: the included design assumes OPTO ESCs

- It is recommended to use an Adafruit Metro Mini, Arduino Micro, or Arduino Pro Mini (NOT a Pro Micro)
	- TLDR: The Atmega board needs to be Arduino compatible and have Digital pins 3,9,10,11, at least one analog pin, and on board USB-serial.
	- Use PWM @ 490Hz to control each ESC
	- See the `arduino.ino` sketch. You may need to change the pin numbers to match your board and motor ordering
	- Warning: Do not use Arduino pins 5,6 to control the ESCs; on many boards, they run on a different clock that is at the wrong speed.
