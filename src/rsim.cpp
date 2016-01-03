#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"

#include "simulate.h"
#include "attitude_ctrl.h"

#include <iostream>

using namespace std;


Simulation *sim;


ros::Publisher pose_pub;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

bool first = true;

void motor_callback(const std_msgs::Float32MultiArray::ConstPtr &msg){

	ros::Time now = ros::Time::now();

	double t = now.sec + (now.nsec * 1e-9);

	if(first){
		sim->state.time = t;
		first = false;
	}

	sim->propagate(t);

	sim->setMotors(Vector4d(msg->data[0], msg->data[1], msg->data[2], msg->data[3]));




	Quaterniond q = sim->state.orientation;

	geometry_msgs::PoseStamped msg2;

	geometry_msgs::Point p;
	p.x = 0; p.y = 0; p.z = 0;

	geometry_msgs::Quaternion o;
	o.x = q.x(); o.y = q.y(); o.z = q.z(); o.w = q.w();

	msg2.pose.position = p;
	msg2.pose.orientation = o;

	pose_pub.publish(msg2);



	Vector3d eu = sim->state.orientation.matrix().eulerAngles(0, 1, 2);
	cout << t << " " << eu.y() << endl;

	//AttitudeControl att;
	//att.set(model.weight, Quaterniond(1,0,0,0));


}


int main(int argc, char **argv) {

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	State initial;
	initial.time = 0;
	initial.position = Vector3d(0,0,0);
	initial.velocity = Vector3d(0,0,0);
	initial.orientation = Quaterniond( AngleAxisd(3.14 / 4, Vector3d::UnitY()) );
	initial.omega = Vector3d(0,0,0);

	Model model;

	sim = new Simulation(model, initial);


	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose_sim", 1000);;

	ros::Subscriber sub = n.subscribe("/motors", 1000, motor_callback);


	ros::spin();

	return 0;
}
