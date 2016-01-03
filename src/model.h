#ifndef MODEL_H_
#define MODEL_H_

#include <Eigen/Dense>

using namespace Eigen;

/*
	The vehicle model describes the physical parameters of the quadcopter and describes the rigid body dynamics related to the control inputs
*/

#define FRAME_QUAD_X 1
#define FRAME_QUAD_PLUS 2

#define GRAVITY 9.81

class Model {

public:

	Model();

	// Should know the moments of inertia, motor thrust/torque coefficients, # of motors, geometry,

	Vector4d to_motors(Vector4d controls);
	Vector4d to_controls(Vector4d motors);

	double mass;
	double weight;

	Matrix3d mInertia;

private:
	// Convert motor speeds to thrust/torques
	Matrix4d dynamics;




	// Given control inputs, output linear and angular accelerations/torques



};




#endif
