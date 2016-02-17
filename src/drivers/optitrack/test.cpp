
#include "natnet/NatNetClient.h"

#include <unistd.h>

int main(int argc, char *argv[]){


	NatNetClient *client = new NatNetClient();

	client->Initialize(NULL, NULL);

	while(true){
		sleep(1);
	}

}
