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


Simulation::Simulation(Model &m, const State &initial){
	this->model = m;
	this->state = initial;
}

void Simulation::setMotors(Vector4d m){
	for(int i = 0; i < 4; i++){


		// Physical constraint: limit the motor speeds to 0-1
		if(m[i] < 0){
			m[i] = 0;
			cerr << "motor limits! " << m.transpose() << endl;
		}
		else if(m[i] > 1){
			m[i] = 1;
			cerr << "motor limits! " << m.transpose() << endl;
		}


		// Motor step constraint
		int step = m[i] * 200;
		m[i] = ((double)step) / 200.0;
	}


	// Overall motor delay : ~2ms for usb serial latency and ~2ms max delay for 490Hz PWM
	motor_delay.push(pair<double, Vector4d>(state.time + 0.004, m));
	//this->motors = m;
}

#define epsilon 0.0001

void Simulation::propagate(double time){

	double g = GRAVITY;

	double t = state.time, dt;
	while(t < time){
		// Find best integration interval
		if(t + epsilon <= time)
			dt = epsilon;
		else
			dt = time - t;


		// Unpack state
		Vector3d x = state.position;
		Vector3d v = state.velocity;
		Matrix3d R = state.orientation.matrix();
		Vector3d w = state.omega;


		// Recompute thrust/torques
		Vector4d c = model.to_controls(motors);


		Vector3d thrust = Vector3d(0, 0, c[0]); // Sum of linear forces

		// Get the torques by rigid body dynamics
		Vector3d tau = c.segment<3>(1);


		Vector3d a = Vector3d(0,0, -model.mass*g) + R * thrust;
		a /= model.mass;
		v = v + a*dt;
		x = x + v*dt;


		// Euler equation to get angular acceleration
		Vector3d dw = model.mInertia.inverse() * (tau - w.cross(model.mInertia*w));
		w = w + dw*dt;


		Quaterniond dq = Quaterniond(0, 0.5*w.x(), 0.5*w.y(), 0.5*w.z()) * state.orientation;
		state.orientation = Quaterniond(dt*dq.coeffs() + state.orientation.coeffs());
		state.orientation.normalize();

		Matrix3d dR = CrossMat(w) * R;
		R += dR*dt;




		// Repack state
		state.position = x;
		state.velocity = v;
		//state.orientation = Quaterniond(R);
		state.omega = w;




		t += dt;

		if(!motor_delay.empty() && t >= motor_delay.front().first){
			this->motors = motor_delay.front().second;
			motor_delay.pop();
		}
	}

	state.time = t;
}

void Simulation::run(Controller &ctrl, int freq, double until){

	double dt = 1.0 / freq;
	for(double t = state.time; t < until; t += dt){


		Vector4d c = ctrl.compute(state);
		Vector4d m = model.to_motors(c);
		this->setMotors(m);

		this->propagate(t);

		//Vector3d eu = state.orientation.matrix().eulerAngles(0, 1, 2);
		cout << t << " " << state.orientation.x() << " " << state.orientation.y() << " " << state.orientation.z() << endl;

	}

	this->propagate(until);
}

void simulate(Model &model, Controller &ctrl){

}

