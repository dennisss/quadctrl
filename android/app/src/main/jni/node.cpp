#include <jni.h>

#include <string>
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"

#include "quadcopter.h"
#include "log.h"
#include "motors.h"

using namespace std;


ros::NodeHandle *handle;
ros::AsyncSpinner *spinner;

ros::Publisher pose_pub, motor_pub;
ros::Subscriber set_sub;

static Quadcopter quad;




// Sends motor speeds
void motor_listener(float *speeds){

	std_msgs::Float32MultiArray msg;

	std_msgs::MultiArrayDimension d;
	d.label = "";
	d.size = 4;
	d.stride = 4;

	msg.layout.dim.push_back(d);
	msg.layout.data_offset = 0;


	for(int i = 0; i < 4; i++){
		msg.data.push_back(speeds[i]);
	}

	motor_pub.publish(msg);
}



// Send poses
void pose_listener(Quaternionf q, uint64_t time){

	geometry_msgs::PoseStamped msg;

	geometry_msgs::Point p;
	p.x = 0; p.y = 0; p.z = 0;

	geometry_msgs::Quaternion o;
	o.x = q.x(); o.y = q.y(); o.z = q.z(); o.w = q.w();

	msg.pose.position = p;
	msg.pose.orientation = o;

	pose_pub.publish(msg);
}


// Receive commands
void setpose_callback(const geometry_msgs::Twist::ConstPtr &msg) {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
	quad.setThrottle((float) msg->linear.z);
	quad.joystickInput(Vector3f(msg->angular.x, msg->angular.y, msg->angular.z));
	LOGI("Set throttle %f", msg->linear.z);
}




#ifdef __cplusplus
extern "C" {
#endif

jint JNI_OnLoad(JavaVM *vm, void *reserved) {
    return JNI_VERSION_1_6;
}

const char *node_name = "quadctrl";

JNIEXPORT void JNICALL Java_me_denniss_quadctrl_ControlNode_init(JNIEnv *env, jobject obj, jstring jmaster, jstring jip) {

    int argc = 3;
    // TODO: don't hardcode ip addresses
    const char *argv[] = {"..." , NULL, NULL};

	const char *master = env->GetStringUTFChars(jmaster, JNI_FALSE);
	const char *ip = env->GetStringUTFChars(jip, JNI_FALSE);

	string masterParam = string("__master:=") + master;
	string ipParam = string("__ip:=") + ip;

	env->ReleaseStringUTFChars(jmaster, master);
	env->ReleaseStringUTFChars(jip, ip);

	argv[1] = masterParam.c_str();
	argv[2] = ipParam.c_str();

	// TODO: Clean up this cast
    ros::init(argc, (char **)&argv[0], node_name);
    handle = new ros::NodeHandle();

    pose_pub = handle->advertise<geometry_msgs::PoseStamped>("/pose", 1000);
	motor_pub = handle->advertise<std_msgs::Float32MultiArray>("/motors", 1000);

    set_sub = handle->subscribe("/setpoint", 1000, setpose_callback);


	spinner = new ros::AsyncSpinner(1);
	spinner->start();


	quad.init();
	quad.setListener(pose_listener);

}

JNIEXPORT void JNICALL Java_me_denniss_quadctrl_ControlNode_destroy(JNIEnv *env, jobject obj) {
	spinner->stop();
	delete spinner;
	delete handle;
    ros::shutdown();
};



JNIEXPORT void JNICALL Java_me_denniss_quadctrl_ControlNode_start(JNIEnv *env, jobject obj) {
	vector<float> gP, gI, gD;
	if(handle->getParam("/gains/p", gP)
	   && handle->getParam("/gains/i", gI)
	   && handle->getParam("/gains/d", gD)){
		quad.setGains(
			Vector3f(gP[0], gP[1], gP[2]),
			Vector3f(gI[0], gI[1], gI[2]),
			Vector3f(gD[0], gD[1], gD[2])
		);
	}

	quad.start();
}

JNIEXPORT void JNICALL Java_me_denniss_quadctrl_ControlNode_stop(JNIEnv *env, jobject obj) {
	quad.stop();
}

JNIEXPORT void JNICALL Java_me_denniss_quadctrl_ControlNode_calibrate(JNIEnv *env, jobject obj) {
	quad.calibrate();
}


JNIEXPORT void JNICALL Java_me_denniss_quadctrl_ControlNode_connectUSB(JNIEnv *env, jobject obj, jstring jfspath, jint usbVendorId, jint usbProductId, jint fd) {

	char *fspath = (char *) env->GetStringUTFChars(jfspath, NULL);
	quad.connectUSB(fspath, usbVendorId, usbProductId, fd);
	env->ReleaseStringUTFChars(jfspath, fspath);

}

JNIEXPORT void Java_me_denniss_quadctrl_ControlNode_setMotors(JNIEnv *env, jobject obj, jfloatArray speeds){
    float buf[4];
    env->GetFloatArrayRegion(speeds, 0, 4, buf);
    motors_set(buf);
}



#ifdef __cplusplus
}
#endif
