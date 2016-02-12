#ifndef ATTITUDE_CTRL_H_
#define ATTITUDE_CTRL_H_

#include "controller.h"
#include "pid.h"


/**
 * Maintain an orientation using a PID controller and a fixed thrust
 * This is intended for basic hovers and being integrated into position controllers
 */
class AttitudeControl : public Controller {

public:

	AttitudeControl();

	Vector4d compute(const State &s);

	void set(double thrust);
	void set(double thrust, Quaterniond orient);
	void set(Quaterniond orient);

	void setGains(Vector3d p, Vector3d i, Vector3d d);

	Vector3d debug_error;

	//PID<3> pid;

//private:
	double setthrust;
	Quaterniond setpoint;
};





#endif
