QuadNode Hardware
=================

Motor Configuration
-------------------





Interface Board
---------------

A USB board used to control the motors, and read any external sensors. It uses an 115200 baud serial connection over USB to transmit motor commands.

- Some implementation notes
	- Kills motors if no serial motor command is received within < 1s. It expects a > 100Hz motor update rate from the controller.
	- To prevent accidental re-arming of the motors, the board is powered by an UBEC connected to the motor battery. This way the ESCs are armed exactly once at battery connection and stay ready after plugging/unpluging phones

- It is recommended to use an Arduino Pro Mini (NOT a Micro)
	- Use PWM @ 490Hz to control each ESC
	- See the `arduino.ino` sketch. You may need to change the pin numbers to match your board and motor ordering
	- Warning: Do not use Arduino pins 5,6 to control the ESCs; on many boards, they run on a different clock that is at the wrong speed.
