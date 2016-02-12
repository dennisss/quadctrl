#include "net.h"

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <xbee.h>



struct xbee *xbee;


static struct xbee_con *con1;


/*
	For the SparkFun mini-usb explorer
	idVendor           0x0403 Future Technology Devices International, Ltd
	idProduct          0x6015 Bridge(I2C/SPI/UART/FIFO)
*/



uint64_t addr1 = 0x0013A20040E8766E;
uint64_t addr2 = 0x0013A20040E87431;


net_listener curListener = NULL;


void net_setlistener(net_listener list){

	curListener = list;

}


void myCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data) {

	printf("Got a message!\n");

	if((*pkt)->dataLen > 0) {
		if ((*pkt)->data[0] == '@') {
			xbee_conCallbackSet(con, NULL, NULL);
			printf("*** DISABLED CALLBACK... ***\n");
		}
		//printf("rx: [%s]\n", (*pkt)->data);


		if(curListener != NULL){

			int length = (*pkt)->dataLen;


			packet *pack = (packet *)(*pkt)->data;

			//if(pack->type == PACKET_POSE){
			//	printf("Got a pose message %d\n", length);
			//}


			curListener(pack, length - sizeof(packet));

		}

	}
	//printf("tx: %d\n", xbee_conTx(con, NULL, "Hello\r\n"));
}




// Connect to a device
xbee_con *net_connect(uint64_t addr){


	struct xbee_con *con;
	struct xbee_conAddress address;

	xbee_err ret;


	memset(&address, 0, sizeof(address));
	address.addr64_enabled = 1;

	for(int i = 0; i < 8; i++){
		address.addr64[7-i] = addr & 0xff;
		addr = addr >> 8;
	}

	if((ret = xbee_conNew(xbee, &con, "Data", &address)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conNew() returned: %d (%s)", ret, xbee_errorToStr(ret));
		return NULL;
	}

	if((ret = xbee_conDataSet(con, xbee, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conDataSet() returned: %d", ret);
		return NULL;
	}

	if((ret = xbee_conCallbackSet(con, myCB, NULL)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conCallbackSet() returned: %d", ret);
		return NULL;
	}


	return con;
}


void net_send(struct xbee_con *con, packet *pkt, int datalen){
	xbee_connTx(con, NULL, (unsigned char *)pkt, sizeof(uint32_t) + datalen);

}


void net_arm(){

	packet pkt;
	pkt.type = PACKET_ARM;

	net_send(con1, &pkt, 0);
}

void net_request_pose(){
	packet pkt;
	pkt.type = PACKET_POSE;

	net_send(con1, &pkt, 0);
}


void net_disarm(){

	packet pkt;
	pkt.type = PACKET_DISARM;

	net_send(con1, &pkt, 0);
}

void net_setthrottle(float val){

	packet *pkt = (packet *)malloc(sizeof(packet) + sizeof(float));
	pkt->type = PACKET_THROTTLE;

	*((float*)pkt->data) = val;

	net_send(con1, pkt, sizeof(float));

	free(pkt);
}



void net_init(){



	xbee_err ret;

	if((ret = xbee_setup(&xbee, "xbeeZB", "/dev/ttyUSB0", 115200)) != XBEE_ENONE){
		printf("ret: %d (%s)\n", ret, xbee_errorToStr(ret));

		return;
		//return ret;
	}



	con1 = net_connect(addr1);

}


void net_destroy(){

	xbee_err ret;

	// TODO: Do this for all active connections
	if((ret = xbee_conEnd(con1)) != XBEE_ENONE) {
		xbee_log(xbee, -1, "xbee_conEnd() returned: %d", ret);
		return;
		//return ret;
	}


	xbee_shutdown(xbee);

}

