#ifndef INERTIAL_H_
#define INERTIAL_H_

#include <stdint.h>

// Set to not use the Android system-wide gyro drift compensation. Instead, bias will be calculated as part of the odometry filters
#define INERTIAL_FLAG_NO_DRIFT_COMP 1

typedef void (*inertial_listener)(float *acc, float *gyro, uint64_t time);

void inertial_init(); // Call at least once to initialize timers, etc.
void inertial_enable(int flags); // Call when you want it to start reading
void inertial_disable(); // Call when you want to stop reading

void inertial_setlistener(inertial_listener listener);


uint64_t gettime();


#endif
