#include "simulate.h"

#include <iostream>

using namespace std;


static Matrix3d CrossMat(Vector3d w){
	Matrix3d c;
	c << 0, -w.z(), w.y(),
		 w.z(), 0, -w.x(),
		 -w.y(), w.x(), 0;

	return c;
}


void simulate(Model &model, Controller &ctrl){

	Vector3d x(0,0,0); // Linear position
	Vector3d v(0,0,0); // Linear velocity

	Matrix3d R( AngleAxisd(3.14 / 4, Vector3d::UnitY()) ); // Angular orientation
	Vector3d w(0,0,0); // Angular velocity


	double g = GRAVITY;


	// Moments of inertias : approximation computed by modeling as 4 point mass motors connected by 2 cylindrical rods with a central point mass battery
	// TODO: Change these and move to
	Matrix3d I;
	I << 0.013, 0, 0,
		 0, 0.013, 0,
		 0, 0, 0.02;

	double dt = 0.001; // Integration step
	for(double t = 0; t < 10; t += dt){

		State s;
		s.position = x;
		s.orientation = Quaterniond(R);
		s.omega = w;

		Vector4d c = ctrl.compute(s);
		Vector4d m = model.to_motors(c);

		// Physical constraint: limit the motor speeds to 0-1
		for(int i = 0; i < 4; i++){
			if(m[i] < 0){
				m[i] = 0;
				cerr << "motor limits!" << endl;
			}
			else if(m[i] > 1){
				m[i] = 1;
				cerr << "motor limits!" << endl;
			}
		}

		// Recompute thrust/torques
		c = model.to_controls(m);


		Vector3d thrust = Vector3d(0, 0, c[0]); // Sum of linear forces

		// Get the torques by rigid body dynamics
		Vector3d tau = c.segment<3>(1);


		Vector3d a = Vector3d(0,0, -model.mass*g) + R * thrust;
		a /= model.mass;
		v = v + a*dt;
		x = x + v*dt;


		// Euler equation to get angular acceleration
		Vector3d dw = I.inverse() * (tau - w.cross(I*w));
		w = w + dw*dt;


		Matrix3d dR = CrossMat(w) * R;

		R += dR*dt;

		Vector3d e = R.eulerAngles(0, 1, 2);
		cout << t << " " << e.y() << endl;
	}

}

