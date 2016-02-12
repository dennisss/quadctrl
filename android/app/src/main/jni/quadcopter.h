#ifndef QUADCOPTER_H_
#define QUADCOPTER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

typedef void (*odometry_listener)(Quaterniond quat, uint64_t time);

void sensor_feedback(float *acc, float* gyro, uint64_t time);


class Quadcopter {

public:

	Quadcopter();

	void init();
	void destroy();


	void connectUSB(char *fspath, int usbVendorId, int usbProductId, int fd);

	void start();

	void stop();

	// Can be called any time after init
	void calibrate();

	void setThrottle(double t);

	// Angular joystick input
	void joystickInput(Vector3d a);


	void setListener(odometry_listener l);


	void setGains(Vector3d p, Vector3d i, Vector3d d);

private:

	Quaterniond hoverpt;



};

#endif
