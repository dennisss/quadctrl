#ifndef ONBOARD_H_
#define ONBOARD_H_

#include "packet.h"


// Arm the motors
void motors_arm();

// Set the speeds of the 4 motors. Values should be strictly between 0-1
void motors_set(float values[]);



bool mpu_ready();

void mpu_process();

//
void mpu_setup();

// Reads the IMU data and updates the current attitude estimate
// Called when the IMU triggers a pin interrupt
void mpu_interrupt();

// Store as a w x y z quaternion
void mpu_getorientation(float *buff);



void xbee_setup();
bool xbee_ready();
void xbee_process();
void xbee_send(packet *pkt, unsigned int datalen);


void network_process(packet *pkt, int size);


#endif
