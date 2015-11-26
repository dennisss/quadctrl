
#include "inertial.h"
#include "motors.h"
#include "log.h"

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


// Convert Quaternion to euler angles (needed for error representaion)
// Uses ZYX rotation convention
static Vector3f to_euler(Quaternionf q){
    float q0 = q.w(), q1 = q.x(), q2 = q.y(), q3 = q.z();
    return Vector3f(
            atan2(2.0f*(q0*q1 + q2*q3), 1 - 2.0f*(q1*q1 + q2*q2)),
            asin(2.0f*(q0*q2 - q3*q1)),
            atan2(2.0f*(q0*q3 + q1*q2), 1 - 2.0f*(q2*q2 + q3*q3))
    );
}

static Vector3f eulerRate_to_bodyRate(Vector3f e, Vector3f pose){
    Vector3f b;
    b.x() = e.x() - sin(pose.y())*e.z();
    b.y() = cos(pose.x())*e.y() + sin(pose.x())*cos(pose.y())*e.z();
    b.z() = -sin(pose.x())*e.y() + cos(pose.y())*cos(pose.x())*e.z();

    return b;
/*
void AC_AttitudeControl::frame_conversion_ef_to_bf(const Vector3f& ef_vector, Vector3f& bf_vector)
{
    // convert earth frame rates to body frame rates
    bf_vector.x = ef_vector.x - _ahrs.sin_pitch() * ef_vector.z;
    bf_vector.y = _ahrs.cos_roll()  * ef_vector.y + _ahrs.sin_roll() * _ahrs.cos_pitch() * ef_vector.z;
    bf_vector.z = -_ahrs.sin_roll() * ef_vector.y + _ahrs.cos_pitch() * _ahrs.cos_roll() * ef_vector.z;
}

 */
}

Quaternionf getpose(){
    float buf[4];
    MadgwickAHRSgetquaternion(buf);
    return Quaternionf(buf[0], buf[1], buf[2], buf[3]);
}

int running = 0;

static float setthrottle;
Quaternionf setpoint; // The orientation that should be maintained

Vector3f lastE; // Used for computing derivative
Vector3f totalE; // Used for tracking the integral

void sensor_feedback(float *acc, float* gyro, uint64_t time){
    MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);

    if(!running || setthrottle < 0.0001) {
        float zero[] = {0,0,0,0};
        motors_set(zero);
        return;
    }

    Quaternionf q = getpose();

    // The euler angle error
    Vector3f e = to_euler(q * setpoint.inverse());

    // Convert error to body-fixed frame
    e = eulerRate_to_bodyRate(e, to_euler(getpose()));



    // Weights/gains for PID filter
    // Order: (roll around x), (pitch around y), (yaw around z)
    Vector3f gP(-1, -1, -1);
    Vector3f gI(0, 0, 0);
    Vector3f gD(0, 0, 0);

    float dt = 0.01;

    // Compute derivative
    Vector3f dE = (e - lastE) / dt;
    // Integrate
    totalE += e * dt;

    // Compute control output (desired angular moments that need to be applied)
    Vector3f control = gP.cwiseProduct(e) + gI.cwiseProduct(totalE) + gD.cwiseProduct(dE);



    // Converts a vector represented in the imu frame into the motor frame
    // Note: If the phone is mounted differently, then this should be the only thing that needs to change
    Matrix3f imu2motors;
    imu2motors << -1, 0, 0,  // This is for the phone laying down on the quadcopter
                  0, 1, 0,
                  0, 0, -1;
    control = imu2motors * control;



    float throttle = setthrottle; // The desired throttle // TODO: Does this need to be transformed as well?


    // Convert to motor speeds
    float kF = 1.0;
    float kM = 1.0;
    float L = 1.0;
    float s = sin(M_PI / 4.0), c = cos(M_PI / 4.0);
    Matrix4f dynamics;
    dynamics << kF, kF, kF, kF, // Thrust
            kF*L*c, -kF*L*c, -kF*L*c, kF*L*c, // Roll moment
            kF*L*s, kF*L*s, -kF*L*s, -kF*L*s, // Pitch moment
            kM, -kM, kM, -kM; // Yaw moment


    Vector4f d(throttle, control(0), control(1), control(2));

    // Compute necessary motor controls (computes omega^2 for each motor)
    Vector4f m = dynamics.inverse() * d;

    float speeds[] = {m[0], m[1], m[2], m[3]};

    motors_set(speeds);
}





#ifdef __cplusplus
extern "C" {
#endif



JNIEXPORT void Java_me_denniss_quadnode_Quadcopter_setup(JNIEnv *env, jobject obj, jstring jfspath, jint usbVendorId, jint usbProductId, jint fd) {

    char *fspath = (char *) env->GetStringUTFChars(jfspath, NULL);
    motors_init(fspath, usbVendorId, usbProductId, fd);
    env->ReleaseStringUTFChars(jfspath, fspath);

    //float buf[] = {0, 1, 1, 1};
    //for (int i = 0; i < 16; i++) {
    //    LOGI("Writing to usb...");
    //    motors_set(buf);
    //    sleep(1);
    //}

    inertial_init();
    inertial_setlistener(sensor_feedback);
    inertial_enable();
}

JNIEXPORT void Java_me_denniss_quadnode_Quadcopter_start(JNIEnv *env, jobject obj){

    setthrottle = 0.0;
    setpoint = getpose();
    lastE = Vector3f(0,0,0);
    totalE = Vector3f(0,0,0);


    running = 1;


}

JNIEXPORT void Java_me_denniss_quadnode_Quadcopter_setthrottle(JNIEnv *env, jobject obj, jfloat throttle){
    setthrottle = throttle;
}

JNIEXPORT void Java_me_denniss_quadnode_Quadcopter_stop(JNIEnv *env, jobject obj){

    running = 0;

    //motors_destroy();

    // Stop listening
    //inertial_disable();
    //inertial_setlistener(NULL);
}


#ifdef __cplusplus
}
#endif
