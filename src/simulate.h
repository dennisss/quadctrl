#ifndef SIMULATION_H_
#define SIMULATION_H_

#include "model.h"
#include "controller.h"


/*
	A controller takes the current state as input and gives control inputs to improve the state.



	Given a model of a quadcopter, and a controller that provides motor control inputs,

	Tracks the pose of the quadcoper

*/


void simulate(Model &model, Controller &ctrl);


class Simulation {


};





#endif
