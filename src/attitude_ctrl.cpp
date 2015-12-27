#include "attitude_ctrl.h"


static Vector3d lastE(0,0,0); // Used for computing derivative
static Vector3d totalE(0,0,0); // Used for tracking the integral

// Weights/gains for PID filter
// Order: (roll around x), (pitch around y), (yaw around z)
static Vector3d gP(3, 3, 3);
static Vector3d gI(0.5, 0.5, 0.5);
static Vector3d gD(0, 0, 0);

AttitudeControl::AttitudeControl(){
	setpoint = Quaterniond(1,0,0,0);
	setthrust = 0;
};

Vector4d AttitudeControl::compute(const State &s){

	Quaterniond q = s.orientation;


	// Pure quaternion implementation of the error; first the change quaternion is computed and it is converted to an angular velocity vector
	// This is similar to the approach taken in "Full Quaternion Based Attitude Control for a Quadrotor"
	//Quaterniond qe = (joypoint*setpoint) * q.conjugate();
	Quaterniond qe = q.conjugate() * setpoint;

	if(qe.w() < 0) // Rotation of more than pi radians (meaning the change is not minimal)
		qe = qe.conjugate();

	// Error is in the range -1 to 1 : corresponding to -pi to pi radians (it is the sin(angle/2) which is fairly linear around 0)
	Vector3d e = qe.coeffs().segment<3>(0); // extract x,y,z part




	double dt = 0.001; //0.01;

	// Compute derivative
	Vector3d dE = (e - lastE) / dt;
	// Integrate
	totalE += e * dt;


	for(int i = 0; i < 3; i++) {
		if(totalE[i] > 1)
			totalE[i] = 1;
		if(totalE[i] < -1)
			totalE[i] = -1;
	}


	// Compute control output (desired angular moments that need to be applied)
	Vector3d control = gP.cwiseProduct(e) + gI.cwiseProduct(totalE) + gD.cwiseProduct( -s.omega /* dE */);


	// Compute angle between vertical and adjust thrust
//	double cost = Vector3f(0,0,1).dot(q._transformVector(Vector3f(0,0,1)));
//	double throttle = setthrottle / cost; // The desired throttle




	return Vector4d(setthrust, control(0), control(1), control(2));
}

void AttitudeControl::set(double thrust, Quaterniond orient){
	this->setthrust = thrust;
	this->setpoint = orient;

	lastE = Vector3d(0,0,0);
	totalE = Vector3d(0,0,0);
}
