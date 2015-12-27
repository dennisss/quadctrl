#include "model.h"

#include <cmath>


Model::Model(){

	this->mass = 1.0;
	this->weight = this->mass * GRAVITY;

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
	return dynamics.inverse() * controls;
}

Vector4d Model::to_controls(Vector4d speeds){
	return dynamics * speeds;
}
