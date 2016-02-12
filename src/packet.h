#ifndef QUADCTRL_PACKET_H_
#define QUADCTRL_PACKET_H_

#include "stdint.h"

/* Structure of packets going to between the planner and the onboard computer */

// Arm/disarm have no data
#define PACKET_ARM 1
#define PACKET_DISARM 2
#define PACKET_POSE 3
#define PACKET_THROTTLE 4

typedef struct {

	uint32_t type;

	// If Pose, then this is a float32 w,x,y,z quaternion
	char data[];

} packet;


#endif
