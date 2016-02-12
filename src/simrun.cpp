#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float32MultiArray.h"

#include "simulate.h"
#include "attitude_ctrl.h"

#include <iostream>
#include <unistd.h>

using namespace std;



ros::Publisher pose_pub;
ros::Publisher motor_pub;

void run_sim(Simulation &sim){
	double freq = 100, until = 2;


	AttitudeControl ctrl;
	ctrl.set(sim.model.weight, Quaterniond(1,0,0,0));


	double dt = 1.0 / freq;
	for(double t = sim.state.time; t < until; t += dt){


		cout << t << endl;

		Vector4d c = ctrl.compute(sim.state);
		Vector4d m = sim.model.to_motors(c);
		sim.setMotors(m);

		sim.propagate(t);




		Quaterniond q = sim.state.orientation;

		geometry_msgs::PoseStamped msg;

		geometry_msgs::Point p;
		p.x = 0; p.y = 0; p.z = 0;

		geometry_msgs::Quaternion o;
		o.x = q.x(); o.y = q.y(); o.z = q.z(); o.w = q.w();

		msg.pose.position = p;
		msg.pose.orientation = o;

		pose_pub.publish(msg);





		std_msgs::Float32MultiArray msg2;

		std_msgs::MultiArrayDimension d;
		d.label = "";
		d.size = 4;
		d.stride = 4;

		msg2.layout.dim.push_back(d);
		msg2.layout.data_offset = 0;


		for(int i = 0; i < 4; i++){
			msg2.data.push_back(m[i]);
		}

		motor_pub.publish(msg2);



		//Vector3d eu = state.orientation.matrix().eulerAngles(0, 1, 2);
		//cout << t << " " << eu.y() << endl;

		ros::spinOnce();

		if(!ros::ok())
			exit(0);


		usleep(1000000 / 10);
	}



}



int main(int argc, char **argv) {

	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	State initial;
	initial.time = 0;
	initial.position = Vector3d(0,0,0);
	initial.velocity = Vector3d(0,0,0);
	initial.orientation = Quaterniond( AngleAxisd(3.14 / 4, Vector3d::UnitX()) * AngleAxisd(3.14 / 6, Vector3d::UnitY()) * AngleAxisd(3.14 / 6, Vector3d::UnitX()));
	initial.omega = Vector3d(0,0,0);

	Model model;

	Simulation sim(model, initial);

	//sim.run(att, 100, 10);


	pose_pub = n.advertise<geometry_msgs::PoseStamped>("/pose", 1000);
	motor_pub = n.advertise<std_msgs::Float32MultiArray>("/motors", 1000);
	//ros::Subscriber sub = n.subscribe("/motors", 1000, motor_callback);

	run_sim(sim);

	//ros::spin();

	return 0;
}
