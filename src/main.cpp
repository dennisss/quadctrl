
#include "model.h"
#include "attitude_ctrl.h"
#include "simulate.h"



int main(int argc, char *argv[]){

	State initial;
	initial.time = 0;
	initial.position = Vector3d(0,0,0);
	initial.velocity = Vector3d(0,0,0);
	initial.orientation = Quaterniond( AngleAxisd(3.14 / 6, Vector3d::UnitY()) * AngleAxisd(3.14 / 180, Vector3d::UnitX()) );
	initial.omega = Vector3d(0,0,0);

	Model model;
	AttitudeControl att;
	att.set(model.weight, Quaterniond(1,0,0,0));

	Simulation sim(model, initial);

	sim.run(att, 100, 10);

}
