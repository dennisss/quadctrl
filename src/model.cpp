#include "model.h"

#include <cmath>

#include <iostream>

Model::Model(){

	this->mass = 1.0;
	this->weight = this->mass * GRAVITY;


	// Moments of inertias : approximation computed by modeling as 4 point mass motors connected by 2 cylindrical rods with a central point mass battery
	mInertia << 0.0198, 0, 0,
				0, 0.0198, 0,
				0, 0, 0.02;


	// Reasonable parameters for a DJI F450 with a 2:1 thrust:weight ratio
	double twr = 2.0; // Thrust/weight ratio

	double kF = twr * this->weight / 4; // 0.5; // Force/Thrust coefficient
	double kM = 0.1; // Momentum/Torque coefficient
	double L = 0.450 / 2.0; // Arm length
	double s = sin(M_PI / 4.0), c = cos(M_PI / 4.0);

	// Quadcopter X geometry configuration
	dynamics << kF, kF, kF, kF, // Thrust
				kF*L*c, -kF*L*c, -kF*L*c, kF*L*c, // X/Roll moment
				-kF*L*s, -kF*L*s, kF*L*s, kF*L*s, // Y/Pitch moment
				kM, -kM, kM, -kM; // Z/Yaw moment
}

Vector4d Model::to_motors(Vector4d controls){

	// Convert to motor speeds
	// Compute necessary motor controls (computes omega^2 for each motor)
	Vector4d m = dynamics.inverse() * controls;




	// Sanity check the controls : ensure motor speeds stay between 0-1 for a constant thrust
	// TODO: This will not work for an unattainable thrust
	Vector4d tvec = controls;
	tvec(0) = 0;
	tvec.normalize();

	// Changing the motor vector by a fraction of dm which keep the same thrust and torque ratio, but will change the torque amplitude.
	Vector4d dm = dynamics.inverse() * tvec;

	// Get the torque into do-able motor limits
	for(int i = 0; i < 4; i++){
		double diff = 0;
		if(m(i) > 1)
			diff = 1 - m(i);
		if(m(i) < 0)
			diff = -m(i);

		if(dm(i) > 0.0001)
			m = m + (diff / dm(i))*dm;
	}

	return m;
}

Vector4d Model::to_controls(Vector4d speeds){
	return dynamics * speeds;
}
