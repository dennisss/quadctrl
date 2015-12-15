#include "quadcopter.h"
#include "inertial.h"
#include "motors.h"
#include "log.h"
#include "node.h"

//#include "MahonyAHRS.h"
#include "MadgwickAHRS.h"

#include <jni.h>

#include <stdio.h>
#include <unistd.h>
#include <string>
#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


static int running = 0;

static float setthrottle;
static Quaternionf setpoint; // The orientation that should be maintained

static Vector3f lastE; // Used for computing derivative
static Vector3f totalE; // Used for tracking the integral

static odometry_listener listener;

Quaternionf getpose(){
    float buf[4];
    MadgwickAHRSgetquaternion(buf);
    return Quaternionf(buf[0], buf[1], buf[2], buf[3]);
}



Quadcopter::Quadcopter(){

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
	setthrottle = 0.0;
	setpoint = getpose();
	lastE = Vector3f(0,0,0);
	totalE = Vector3f(0,0,0);

	running = 1;
}

void Quadcopter::stop(){
	running = 0;
}


void Quadcopter::connectUSB(char *fspath, int usbVendorId, int usbProductId, int fd){
	motors_init(fspath, usbVendorId, usbProductId, fd);
}

void Quadcopter::setListener(odometry_listener l){
    listener = l;
};

void Quadcopter::setThrottle(float t){
    setthrottle = t;
}


// Convert Quaternion to euler angles (needed for error representaion)
// Uses XYZ convention
static Vector3f to_euler(Quaternionf q){
    return q.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
}

// Convert a euler derivative to a body frame derivative
static Vector3f eulerRate_to_bodyRate(Vector3f e, Vector3f pose){

	Vector3f out;

    out = (AngleAxisf(pose.x(), Vector3f::UnitX())
          *AngleAxisf(pose.y(), Vector3f::UnitY()))
          *Vector3f(0, 0, e.z())
            +
          (AngleAxisf(pose.x(), Vector3f::UnitX()))
          *Vector3f(0, e.y(), 0)
            +
          Vector3f(e.x(), 0, 0);

    return out;
}


uint64_t last_time = 0; // TODO: Make sure this resets properly



void sensor_feedback(float *acc, float* gyro, uint64_t time){
    Matrix3f imu2motors;
    imu2motors << 1, 0, 0,  // This is for the phone laying down on the quadcopter
                  0, 1, 0,
                  0, 0, 1;



    if(last_time != 0) {
        float dt = ((uint64_t)(time - last_time)) / 1.0e9;

        MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], dt);
    }
    last_time = time;


    // From here on, all rotations are w.r.t the quadcopter/motor frame (x pointing forward, y right)
	Quaternionf q = Quaternionf(imu2motors)*getpose();

    // Publish pose
	if(listener != NULL){
		listener(q, time);
	}

	// Don't touch the motors if it is not started
	if(!running)
		return;

    if(setthrottle < 0.0001) {
        float zero[] = {0,0,0,0};
        motors_set(zero);
        return;
    }


    // The euler angle error
    //Vector3f e = to_euler(setpoint) - to_euler(q); //to_euler(setpoint * q.inverse());

    // Convert error to body-fixed frame
    //e = eulerRate_to_bodyRate(e, to_euler(q));
    //e = Vector3f(e[1], e[2], e[0]); // Switch to roll, pitch, yaw


    // Pure quaternion implementation of the error; first the change quaternion is computed and it is converted to an angular velocity vector
    // This is similar to the approach taken in "Full Quaternion Based Attitude Control for a Quadrotor"
    Quaternionf qe = setpoint * q.conjugate();
    if(qe.w() < 0) // Rotation of more than pi radians (meaning the change is not minimal)
        qe = qe.conjugate();

    Vector3f e = qe.coeffs().segment<3>(0); // extract x,y,z part

    LOGI("%0.4f %0.4f %0.4f", e[0], e[1], e[2]);



    // Weights/gains for PID filter
    // Order: (roll around x), (pitch around y), (yaw around z)
    Vector3f gP(1, 1, 1);
    Vector3f gI(0, 0, 0);
    Vector3f gD(0.1, 0.1, 0.1);

    float dt = 0.01;

    // Compute derivative
    Vector3f dE = (e - lastE) / dt;
    // Integrate
    totalE += e * dt;

    // Compute control output (desired angular moments that need to be applied)
    Vector3f control = gP.cwiseProduct(e) + gI.cwiseProduct(totalE) + gD.cwiseProduct(dE);

    // Scaling factor
    control *= 0.5;



    // Converts a vector represented in the imu frame into the motor frame
    // Note: If the phone is mounted differently, then this should be the only thing that needs to change
    //control = imu2motors * control;


    // TODO: For hover, this needs to account for the orientation
    float throttle = setthrottle; // The desired throttle


    // Convert to motor speeds
    float kF = 1.0; // Force/Thrust coefficient
    float kM = 1.0; // Momentum/Torque coefficient
    float L = 1.0; // Arm length
    float s = sin(M_PI / 4.0), c = cos(M_PI / 4.0);
    Matrix4f dynamics;
    dynamics << kF, kF, kF, kF, // Thrust
            kF*L*c, -kF*L*c, -kF*L*c, kF*L*c, // Roll moment
            -kF*L*s, -kF*L*s, kF*L*s, kF*L*s, // Pitch moment
            kM, -kM, kM, -kM; // Yaw moment


    Vector4f d(throttle, control(0), control(1), control(2));

    // Compute necessary motor controls (computes omega^2 for each motor)
    Vector4f m = dynamics.inverse() * d;

    float speeds[] = {m[0], m[1], m[2], m[3]};


    motor_listener(speeds);


    motors_set(speeds);
}





#ifdef __cplusplus
extern "C" {
#endif


JNIEXPORT void Java_me_denniss_quadnode_Quadcopter_setMotors(JNIEnv *env, jobject obj, jfloatArray speeds){
    float buf[4];
    env->GetFloatArrayRegion(speeds, 0, 4, buf);
    motors_set(buf);
}


#ifdef __cplusplus
}
#endif
