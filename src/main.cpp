
#include "model.h"
#include "attitude_ctrl.h"
#include "simulate.h"



int main(int argc, char *argv[]){

	Model model;

	AttitudeControl att;
	att.set(model.weight, Quaterniond(1,0,0,0));


	simulate(model, att);

}
