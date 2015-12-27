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

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;
using namespace Eigen;


static int running = 0;

static float setthrottle;
//static Quaternionf setpoint; // The orientation that should be maintained
static Quaternionf joypoint; // The additional rotation requested by a joystick

static Vector3f gyro_bias(0,0,0);



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
	setthrottle = 0;
	setpoint = getpose();
    joypoint = Quaternionf(1,0,0,0);
	lastE = Vector3f(0,0,0);
	totalE = Vector3f(0,0,0);

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
    setthrottle = t;
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




    float speeds[] = {m[0], m[1], m[2], m[3]};


    motor_listener(speeds);


    motors_set(speeds);

}

void run(){

    Matrix3f imu2motors;

    imu2motors << 0, 0, 1, // For the phone vertically
                  0, 1, 0,
                  -1, 0, 0;


    /*
    imu2motors << 1, 0, 0,  // This is for the phone laying down on the quadcopter
                  0, 1, 0,
                  0, 0, 1;
    */


    if(last_time != 0) {
        float dt = ((uint64_t)(time - last_time)) / 1.0e9;

        gyro[0] -= gyro_bias[0];
        gyro[1] -= gyro_bias[1];
        gyro[2] -= gyro_bias[2];

        MadgwickAHRSupdateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], dt);
    }
    last_time = time;


    // From here on, all rotations are w.r.t the quadcopter/motor frame (x pointing forward, y right)
	Quaternionf q = getpose()*Quaternionf(imu2motors);

    // Publish pose
	if(listener != NULL){
		listener(q, time);
	}

    //setthrottle = 2;

	// Don't touch the motors if it is not started
	if(!running)
		return;

    if(setthrottle < 0.0001) {
        float zero[] = {0,0,0,0};
        motors_set(zero);
        return;
    }


	// TODO : TODO: Make sure the state is converted by
	// The setpoint needs to be converted to body axes, TODO: Do this in start()
	//Quaternionf target = setpoint * Quaternionf(imu2motors);
	// Before being sent to compute
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
