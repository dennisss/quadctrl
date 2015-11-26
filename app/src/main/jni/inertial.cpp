/*
 * Code for getting raw IMU data
 * Internally, the accelerometer and gyroscope readings are received at different times,
 * so this code abstracts it such that they are returned simultaneously by interpolating readings
 * and returning those values and the
 */
#include "inertial.h"
#include "log.h"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include <android/looper.h>
#include <android/sensor.h>




static uint64_t startreal; // The start time relative to epoch
static uint64_t starttime; // The start time for the monotonic clock

#define RELATIVE_TIME ((gettime() - starttime) + startreal)

static uint64_t getrealtime(){
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	return (uint64_t) now.tv_nsec + ((uint64_t) now.tv_sec * 1000000000);
}

static uint64_t gettime(){
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC_RAW, &now);
	return (uint64_t) now.tv_nsec + ((uint64_t) now.tv_sec * 1000000000);
}





static ASensorManager *sensorManager;
static ASensorEventQueue *eventQueue;

static const ASensor *gyroSensor;
static const ASensor *magnSensor;
static const ASensor *accelSensor;


static inertial_listener current_listener = NULL;


uint64_t lastAccelRelTime = 0;
uint64_t lastRotatRelTime = 0;

uint64_t lastAccelTime;
uint64_t lastRotatTime;

static float lastAccel[3] = {0,0,0};
static float lastRotat[3] = {0,0,0};


//static uint64_t lastTime;

/*
	Receives events from the looper
 	We emit an event every time an gyroscope event is received. The accelerometer is interpolated

 */
static ASensorEvent eventBuffer[1];
static int get_sensor_events(int fd, int events, void *data) {

	ASensorEvent &event = eventBuffer[0];

	// TODO: Add interpolation of one of the values (depending on which one came in first)


	int n = ASensorEventQueue_getEvents(eventQueue, eventBuffer, 1);
	if(n == 1){ // If I got an event

		if(event.type == ASENSOR_TYPE_ACCELEROMETER){

			// We interpolate acceleration to the time of the last gyro event
			// If I already had a gyro event and I can interpolate, then emit an event
			if(lastRotatTime != 0 && lastAccelTime != 0){

				uint64_t span = event.timestamp - lastAccelTime; // Time between the events being interpolated
				uint64_t dt = event.timestamp - lastRotatTime;

				float alpha; // Percent of the last value to use

				if(span == 0)
					alpha = 0;
				else
					alpha = dt / span;

				// Perform interpolation
				float interp[3];
				interp[0] = alpha*event.vector.x  + (1. - alpha)*lastAccel[0];
				interp[1] = alpha*event.vector.y  + (1. - alpha)*lastAccel[1];
				interp[2] = alpha*event.vector.z  + (1. - alpha)*lastAccel[2];


				if(current_listener != NULL) {
					current_listener(/*lastAccel*/ interp, lastRotat, lastRotatRelTime);
				}

			}


			lastAccel[0] = event.vector.x;
			lastAccel[1] = event.vector.y;
			lastAccel[2] = event.vector.z;

			lastAccelRelTime = RELATIVE_TIME;
			lastAccelTime = event.timestamp; // TODO: Are these timestamps monotonic? And are they relative to epoch?


			LOGI("A: %llu", event.timestamp);

		}
		else if(event.type == ASENSOR_TYPE_GYROSCOPE){
			lastRotat[0] = event.vector.x;
			lastRotat[1] = event.vector.y;
			lastRotat[2] = event.vector.z;
			lastRotatRelTime =  RELATIVE_TIME;
			lastRotatTime = event.timestamp;

			LOGI("G: %llu", event.timestamp);
		}

		/*
		if(lastAccelTime != 0 && lastRotatTime != 0){



			lastAccelTime = 0;
			lastRotatTime = 0;
		}
		*/
	}


	//should return 1 to continue receiving callbacks, or 0 to unregister
	return 1;
}


void inertial_init(){

	startreal = getrealtime();
	starttime = gettime();

	LOGI("START TIME %llu", startreal);
}


void inertial_enable(){

	lastAccelTime = 0;
	lastRotatTime = 0;
	lastAccelRelTime = 0;
	lastRotatRelTime = 0;


    ALooper* looper = ALooper_forThread();

    if(looper == NULL)
        looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);


    sensorManager = ASensorManager_getInstance();

    gyroSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_GYROSCOPE);
    accelSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_ACCELEROMETER);
    //magnSensor = ASensorManager_getDefaultSensor(sensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);


    eventQueue = ASensorManager_createEventQueue(sensorManager, looper, 1, get_sensor_events, NULL);

    ASensorEventQueue_enableSensor(eventQueue, accelSensor);
    ASensorEventQueue_enableSensor(eventQueue, gyroSensor);
    //ASensorEventQueue_enableSensor(eventQueue, magnSensor);

    //Sampling rate: 100Hz
    int a = ASensor_getMinDelay(accelSensor);
    int b = ASensor_getMinDelay(gyroSensor);
    //int c = ASensor_getMinDelay(magnSensor);

	LOGI("min-delay: %d, %d",a,b);
    //LOGI("min-delay: %d, %d, %d",a,b,c);
    ASensorEventQueue_setEventRate(eventQueue, accelSensor, 10000); // (1000L/SAMP_PER_SEC)*1000
    ASensorEventQueue_setEventRate(eventQueue, gyroSensor, 10000);
    //ASensorEventQueue_setEventRate(eventQueue, magnSensor, 10000);

    LOGI("sensorValue() - START");

}


void inertial_disable(){
    ASensorManager_destroyEventQueue(sensorManager, eventQueue);
}

void inertial_setlistener(inertial_listener l){
	current_listener = l;
}
