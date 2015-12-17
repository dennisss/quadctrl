#ifndef QUADCOPTER_H_
#define QUADCOPTER_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

typedef void (*odometry_listener)(Quaternionf quat, uint64_t time);

void sensor_feedback(float *acc, float* gyro, uint64_t time);


class Quadcopter {

public:

	Quadcopter();

	void init();
	void destroy();


	void connectUSB(char *fspath, int usbVendorId, int usbProductId, int fd);

	void start();

	void stop();


	void setThrottle(float t);

	// Angular joystick input
	void joystickInput(Vector3f a);


	void setListener(odometry_listener l);


	void setGains(Vector3f p, Vector3f i, Vector3f d);



};

#endif
