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
static Quaterniond joypoint; // The additional rotation requested by a joystick

static Vector3f gyro_bias(0,0,0);



static odometry_listener listener;

static Matrix3d imu2motors;


Quaterniond get_inertial_pose(){
    float buf[4];
    MadgwickAHRSgetquaternion(buf);
    return Quaterniond(buf[0], buf[1], buf[2], buf[3]);
}

Quaterniond get_motor_pose(){
    return get_inertial_pose() * Quaterniond(imu2motors);
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

    Quaterniond pose = get_motor_pose();
    hoverpt = Quaterniond(pose.w(), 0, 0, pose.z()); // Retain only the yaw
    hoverpt.normalize();

    attCtrl.set(0, hoverpt);

    joypoint = Quaterniond(1,0,0,0);

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

void Quadcopter::setThrottle(double t){
    attCtrl.set(t*9.81);
}

void Quadcopter::joystickInput(Vector3d a){

    Matrix3d m;
    m = AngleAxisd(a[0], Vector3d::UnitX())
        * AngleAxisd(a[1],  Vector3d::UnitY())
        * AngleAxisd(a[2], Vector3d::UnitZ());

    joypoint = Quaterniond(m);

    attCtrl.set(hoverpt*joypoint);
}

void Quadcopter::setGains(Vector3d p, Vector3d i, Vector3d d){
    attCtrl.setGains(p, i, d);
}



static uint64_t last_time = 0; // TODO: Make sure this resets properly

void sensor_feedback(float *acc, float* gyro, uint64_t time){


    uint64_t stime = gettime();


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

    if(attCtrl.setthrust < 0.0001) {
        float zero[] = {0,0,0,0};
        motors_set(zero);
        return;
    }



    // Prepare state
    State s;
    s.orientation = get_motor_pose();
    s.omega = imu2motors * Vector3d(gyro[0], gyro[1], gyro[2]);

    // Compute the necessary control response
    Vector4d m = model.to_motors( attCtrl.compute(s) );


    // Set the motors
    float speeds[] = {m[0], m[1], m[2], m[3]};
    motors_set(speeds);
    motor_listener(speeds);

    uint64_t etime = gettime();

    LOGI("%.4f %.4f %.4f ", attCtrl.debug_error(0), attCtrl.debug_error(1), attCtrl.debug_error(2));

    //LOGI("Time to finish: %lld", etime - stime);

}
