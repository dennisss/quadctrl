#ifndef POSITION_CTRL_H_
#define POSITION_CTRL_H_

#include "controller.h"
#include "attitude_ctrl.h"

/**
 * Follows [x, y, z, yaw] trajectories
 */
class PositionControl : public Controller {

public:

	Vector4d compute(const State &S);



private:

	AttitudeControl attitude;

};


#endif
