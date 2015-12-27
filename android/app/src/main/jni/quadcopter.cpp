#include "quadcopter.h"
#include "inertial.h"
#include "motors.h"
#include "log.h"
#include "node.h"

#include "model.h"
#include "attitude_ctrl.h"

#include "MadgwickAHRS.h"

#include <jni.h>

#include <stdio.h>
#include <unistd.h>
#include <string>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


static AttitudeControl attCtrl;
static Model model;


static int running = 0;
static Quaternionf joypoint; // The additional rotation requested by a joystick

static Vector3f gyro_bias(0,0,0);



static odometry_listener listener;

static Matrix3f imu2motors;


Quaternionf get_inertial_pose(){
    float buf[4];
    MadgwickAHRSgetquaternion(buf);
    return Quaternionf(buf[0], buf[1], buf[2], buf[3]) * imu2motors;
}

Quaternionf get_motor_pose(){
    return get_inertial_pose()*Quaternionf(imu2motors);
}




Quadcopter::Quadcopter(){

    imu2motors << 0, 0, 1, // For the phone vertically
                  0, 1, 0,
                  -1, 0, 0;


    /*
    imu2motors << 1, 0, 0,  // This is for the phone laying down on the quadcopter
                  0, 1, 0,
                  0, 0, 1;
    */



	running = 0;

}

void Quadcopter::init(){

	inertial_init();
    inertial_setlistener(sensor_feedback);
    inertial_enable(0);

}

void Quadcopter::destroy(){

	// Stop listening
    inertial_disable();
    inertial_setlistener(NULL);

	//if(motors_initted){
		motors_destroy();
	//}

}


void Quadcopter::start(){

    attCtrl.set(0, get_motor_pose());

    joypoint = Quaternionf(1,0,0,0);

	running = 1;
}

void Quadcopter::stop(){
	running = 0;
}

static int npoints = 0;
void calibration_feedback(float *acc, float* gyro, uint64_t time) {
    gyro_bias += Vector3f(gyro[0], gyro[1], gyro[2]);
    npoints++;
}

void Quadcopter::calibrate(){
    sleep(1);

    gyro_bias = Vector3f(0,0,0);
    npoints = 0;
    inertial_setlistener(calibration_feedback);
    sleep(4);
    inertial_setlistener(sensor_feedback);

    gyro_bias /= npoints;

    LOGI("Biases: %.2f %.2f %.2f", gyro_bias[0], gyro_bias[1], gyro_bias[2]);

}


void Quadcopter::connectUSB(char *fspath, int usbVendorId, int usbProductId, int fd){
	motors_init(fspath, usbVendorId, usbProductId, fd);
}

void Quadcopter::setListener(odometry_listener l){
    listener = l;
};

void Quadcopter::setThrottle(float t){
    attCtrl.set(t);
}

void Quadcopter::joystickInput(Vector3f a){

    Matrix3f m;
    m = AngleAxisf(a[0], Vector3f::UnitX())
        * AngleAxisf(a[1],  Vector3f::UnitY())
        * AngleAxisf(a[2], Vector3f::UnitZ());

    joypoint = Quaternionf(m);
}

void Quadcopter::setGains(Vector3f p, Vector3f i, Vector3f d){
	gP = p;
	gI = i;
	gD = d;
}



static uint64_t last_time = 0; // TODO: Make sure this resets properly

void sensor_feedback(float *acc, float* gyro, uint64_t time){

    // Update state
    if(last_time != 0) {
        float dt = ((uint64_t)(time - last_time)) / 1.0e9;

        gyro[0] -= gyro_bias[0];
        gyro[1] -= gyro_bias[1];
        gyro[2] -= gyro_bias[2];

        MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], dt);
    }
    last_time = time;


    // Publish pose
	if(listener != NULL){
		listener(get_motor_pose(), time);
	}

	// Don't touch the motors if it is not started
	if(!running)
		return;

    if(setthrottle < 0.0001) {
        float zero[] = {0,0,0,0};
        motors_set(zero);
        return;
    }



    // Prepare state
    State s;
    s.orientation = get_motor_pose();
    s.omega = Vector3d(gyro[0], gyro[1], gyro[2]);

    // Compute the necessary control response
    Vector4d m = model.to_motors( attCtrl.compute(s) );

    // Set the motors
    float speeds[] = {m[0], m[1], m[2], m[3]};
    motor_listener(speeds);
    motors_set(speeds);
}
