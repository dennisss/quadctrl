#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "model.h"
#include "controller.h"


/*
	A controller takes the current state as input and gives control inputs to improve the state.



	Given a model of a quadcopter, and a controller that provides motor control inputs,

	Tracks the pose of the quadcoper

*/

#include <queue>

using namespace std;


class Simulation {

public:
	Simulation(Model &m, const State &initial);



	// Evaluate state upto the given time
	void propagate(double time);

	//
	void setMotors(Vector4d m);

	// Use the given controller to control the simulatio
	void run(Controller &ctrl, int freq, double until);




	State state;
	Model model;
private:

	Vector4d motors; // Currently set motor speeds


	queue<pair<double, Vector4d> > motor_delay;


	//Vector3d x; // Linear position
	//Vector3d v; // Linear velocity
	//Matrix3d R; // Angular orientation
	//Vector3d w; // Angular velocity



};





#endif
