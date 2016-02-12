/*
	Entry point for the onboard controller

	-


*/


#include "onboard.h"

#include "WProgram.h"



#include <I2Cdev.h>
//#include <MPU9150.h>










bool running = false;

float throttle = 0.00;

void network_process(packet *pkt, int size){

	if(pkt->type == PACKET_ARM){
		Serial.println("Arming...");

		motors_arm();
		running = true;
	}
	else if(pkt->type == PACKET_DISARM){
		running = false;
		delay(1000); // Ensure the ESCs disarm
	}
	else if(pkt->type == PACKET_THROTTLE){

		float *val = (float *)pkt->data;
		Serial.println("Throttle");
		Serial.println(*val);

		throttle = *val;
	}
	else if(pkt->type == PACKET_POSE){

		packet *resp = (packet *)malloc(sizeof(packet) + 4*sizeof(float));
		resp->type = PACKET_POSE;

		mpu_getorientation((float *)&resp->data);

		xbee_send(resp, 4*sizeof(float));

		free(resp);
	}






}





#define LED_PIN 13

bool blinkState = false;


extern "C" int main(void){

    //Wire.begin();
	Wire.begin(I2C_MASTER, NULL, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);


	pinMode(13, OUTPUT);
	digitalWrite(LED_PIN, 1);


	// Start USB serial for
	Serial.begin(115200);

//	// TODO: Don't do this if not waiting for a computer
//	while(!Serial) {
//		; // wait for serial port to connect
//	}


//	digitalWrite(LED_PIN, 0);


	delay(1000);

	Serial.println("OnboardCtrl");
	Serial.println("-----------");




	mpu_setup();
	xbee_setup();




	Serial.println("Looping...");



	uint32_t startTime = millis();

	// TODO: Disarm if haven't gotten a message in a second
//	uint32_t lastMessage = millis();


	// Event loop
	while(true){

		if(mpu_ready()){

			mpu_process();

			if(running){
				float vals[] = {throttle, throttle, throttle, throttle};
				motors_set(vals);
			}
		}

		if(xbee_ready()){

			Serial.println("Got mail");

			xbee_process();

		}


		uint32_t newTime = millis();
		if(newTime - startTime > 500){
			blinkState = !blinkState;
			digitalWrite(LED_PIN, blinkState);
			startTime = newTime;
		}
	}


}
