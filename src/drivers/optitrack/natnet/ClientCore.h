#ifndef NATNET_CLIENTCORE_H_
#define NATNET_CLIENTCORE_H_

#include "NatNetTypes.h"

#include <pthread.h>



// Data callback function prototype
typedef void ( *funcDataCallback) (sFrameOfMocapData* pFrameOfData, void* pUserData);


class ClientCore {

	friend void *data_server(void *arg);
	friend void *cmd_server(void *arg);

public:

	ClientCore();
	~ClientCore();


	/* Setup all the sockets and start the data servers */
	void start();
	void stop();


	/* Sends a ping command to the server */
	void ping();



	funcDataCallback data_callback;
	void *data_callback_arg;
	// also include the data associated with the callback





	void sendPacket(sPacket *packet);




//	struct sockaddr_in multi_addr;
//	struct sockaddr_in cmd_addr;


	// Data buffers
	sFrameOfMocapData *frame;
	sDataDescriptions *dataDescs;



private:

	bool running;

	int data_socket;
	pthread_t data_thread;
	int cmd_socket;
	pthread_t cmd_thread;



	void unpack(char *pData);
	void unpackFrame(char *ptr);
	void unpackDataDescs(char *ptr);


};


// Private functions for reading in separate threads
void *data_server(void *arg);
void *cmd_server(void *arg);


#endif
