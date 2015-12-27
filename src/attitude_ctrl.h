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

	void set(double thrust, Quaterniond orient);

	//PID<3> pid;

private:
	double setthrust;
	Quaterniond setpoint;
};





#endif
