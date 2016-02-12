#include "onboard.h"

#include <i2c_t3.h>


// TODO: These should be externally configurable
// How many motors we have
#define NMOTORS 4
// Whether or not to flip each motor
#define REVERSE(i) ((i) == 0 || (i) == 3)



#define MOTOR_ADDR 0x29

static void arm_motor(byte i){

	Wire.beginTransmission(byte(MOTOR_ADDR + i));
	Wire.write(byte(0x00));
	Wire.write(byte(0xFF));
	Wire.write(byte(0xFF));
	Wire.endTransmission();

	delay(10);

	Wire.beginTransmission(byte(MOTOR_ADDR + i));
	Wire.write(byte(0x00));
	Wire.write(byte(0x00));
	Wire.write(byte(0x00));
	Wire.endTransmission();

	/*
   Stock tgy i2c
    Wire.beginTransmission(byte(0x29));
    Wire.write(byte(0x0));
    Wire.endTransmission();
    delay(10);
  */
}

// val should be a 15 bit value
static void write_motor(byte i, short val){

	Wire.beginTransmission(byte(MOTOR_ADDR + i));
	Wire.write(byte(0x00)); // Address

	byte vh = (val >> 8) & 0xff;
	byte vl = val & 0xff;

	bool reverse = false;

	if(REVERSE(i))
		reverse = true;

	if(reverse)
		vh = vh | 0x80;


	Wire.write(vh);
	Wire.write(vl);
	Wire.endTransmission();
}











void motors_arm(){

	for(int i = 0; i < NMOTORS; i++){
		arm_motor(i);
	}

}




void motors_set(float values[]){

	for(int i = 0; i < NMOTORS; i++){

		double v = values[i];

		if(v > 1)
			v = 1;
		else if(v < 0)
			v = 0;

		// Convert to 15bit ESC range
		short sv = 0x7fff * v;

		write_motor(i, sv);
	}

}




