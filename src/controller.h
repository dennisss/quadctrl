#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Eigen/Dense>

using namespace Eigen;

// The body frame state of the quadcopter
struct State {

	double time;

	Vector3d position;
	Quaterniond orientation;

	Vector3d velocity;
	Vector3d omega; // Angular velocity
};


/**
 * Given the current state, the controller should give control outputs that can be fed through a model
 * In the case of a quadcopter, it returns a 4d vector of z-thrust and torques
 */
class Controller {
public:

	virtual Vector4d compute(const State &s) = 0;


};

#endif
