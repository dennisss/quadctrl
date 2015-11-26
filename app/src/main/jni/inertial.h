#ifndef INERTIAL_H_
#define INERTIAL_H_

#include <stdint.h>

typedef void (*inertial_listener)(float *acc, float *gyro, uint64_t time);

void inertial_init(); // Call at least once to initialize timers, etc.
void inertial_enable(); // Call when you want it to start reading
void inertial_disable(); // Call when you want to stop reading

void inertial_setlistener(inertial_listener listener);





#endif
