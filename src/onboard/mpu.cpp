#include "onboard.h"

#include <WProgram.h>

#include <MPU9150.h>
#include <MadgwickAHRS.h>

#define PI 3.14159265359


MPU9150 mpu;

// Whether is not the MPU has something to read (this is set when the interrupt fires)
static volatile bool isDataReady = false;


void mpu_setup(){

	// Pull down interrupt pin
	*portConfigRegister(2) = PORT_PCR_MUX(1) | PORT_PCR_PE;


	// initialize device
	Serial.println("Initializing I2C devices...");
	mpu.initialize();


	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(mpu.testConnection() ? "MPU9150 connection successful!" : "MPU9150 connection failed");



	Serial.println("d");

	mpu.setInterruptLatch(0); // pulse
	mpu.setInterruptMode(1); // Active Low
	mpu.setInterruptDrive(1); // Open drain
	mpu.setRate(17); // Gyroscope sample rate (8kHz / (1+7)) = 1kHz

	Serial.println("c");

	// Full-scale range of the gyro sensors:
	// 0 = +/- 250 degrees/sec, 1 = +/- 500 degrees/sec, 2 = +/- 1000 degrees/sec, 3 = +/- 2000 degrees/sec
	mpu.setFullScaleGyroRange(0); // set gyro range to 250 degrees/sec

	// Full-scale accelerometer range.
	// The full-scale range of the accelerometer: 0 = +/- 2g, 1 = +/- 4g, 2 = +/- 8g, 3 = +/- 16g
	mpu.setFullScaleAccelRange(0); // set accelerometer to 2 g range


	mpu.setIntDataReadyEnabled(true); // Enable data ready interrupt

	Serial.println("b");


	mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
    mpu.setZAccelOffset(1200); // 1688 factory default for my test chip


	Serial.println("A");


		mpu.getIntStatus(); // Ensure interrupt is initially clear

	pinMode(2, INPUT_PULLUP);
	attachInterrupt(2, mpu_interrupt, FALLING);

		Serial.println("q");




	Serial.println("Done with MPU");

}


bool mpu_ready(){

	return isDataReady && mpu.getIntDataReadyStatus(); //mpu.getIntDataReadyStatus(); //isDataReady;
}


static uint32_t lastTime = 0;

int interval = 0;

void mpu_process(){

	interval++;

	isDataReady = false;

	int16_t raw_accel[3], raw_gyro[3];
	float accel[3], gyro[3];



	uint32_t time = micros();


	mpu.getMotion6(&raw_accel[0], &raw_accel[1], &raw_accel[2], &raw_gyro[0], &raw_gyro[1], &raw_gyro[2]);


	// Convert scale by resolution to floats
	for(int i = 0; i < 3; i++){
		accel[i] = raw_accel[i]*(2.0f/32768.0f); // For the +/-2g range
		gyro[i] = raw_gyro[i]*(250.0f/32768.0f); // For the +/-250deg range

		// Convert gyro[i] from degrees to radians
		gyro[i] = gyro[i] * (PI / 180.0f);


		if(interval % 100 == 0){
			Serial.print(accel[i]); Serial.print("\t");
		}
	}
	if(interval % 100 == 0)
		Serial.print("\n");



	float dt = 0.01;


	// Perform
	MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], dt);

}

void mpu_getorientation(float *buff){
	MadgwickAHRSgetquaternion(buff);
}



// TODO: Instead, just set a "data ready" flag in the interrupt function and have the reading/calculations happen in the main loop (that they we never have corruption)
void mpu_interrupt(){

	// Get interrupt type and reset it
//	isDataReady = mpu.getIntDataReadyStatus();

//	mpu.getIntStatus();

	isDataReady = true;

}
