#ifndef QUADCTRL_NET_H_
#define QUADCTRL_NET_H_


#include "packet.h"

typedef void (*net_listener)(packet *pkt, int datalen);


void net_init();

void net_destroy();


void net_arm();
void net_disarm();

void net_setthrottle(float v);

void net_setlistener(net_listener list);

void net_request_pose();

#endif
