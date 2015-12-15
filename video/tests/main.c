#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <sys/socket.h>
#include <netdb.h>

#include "stream.h"

const int width = 4096, height = 1;
const size_t bufSize = width * height;

char txBuf[bufSize];	
char rxBuf[bufSize];

int main(int argc, char* argv[])
{
	int txSock, rxSock;
	struct hostent* he;

	errno = 0;

	if(argc < 2){
		printf("Please provide a hostname as a commandline argument.\n");
		return -1;
	}

	if(!(he = gethostbyname(argv[1]))){
		printf("Failed to resolve hostname.\n");
		return -2;
	}


	if(!(txSock = socket(AF_INET, SOCK_DGRAM, 0))){
		return -3;
	}

	if(!(rxSock = socket(AF_INET, SOCK_DGRAM, 0))){
		return -3;
	}
	assert(!errno);

	// setup the port and destination for transmission
	struct sockaddr_in addr = { 
                .sin_family = AF_INET, 
                .sin_port   = htons(1337),  
		.sin_addr   = htonl(INADDR_ANY), 
       };
	bind(rxSock, (const struct sockaddr*)&addr, sizeof(addr));
	assert(!errno);
	printf("Sockets are all opened and bound\n");

	// get ready to start transmitting

	// open up random, write some static into te buffer
	int rand = open("/dev/random", O_RDONLY);
	assert(rand > 0 && !errno);
	read(rand, txBuf, bufSize);
	printf("Random data has been loaded\n");

	// retry until transmission works
	int res;
	do{
		res = txFrame(txSock, (const struct sockaddr*)&addr, width, height, txBuf);
	}while(res);

	printf("Transmission successful!\n");	
	sleep(1);
	
	while(memcmp(rxBuf, txBuf, bufSize)){
		frameHeader_t header = rxFrame(rxSock, (char**)&rxBuf);
		printf("Recieved a thing\nDims: (%d, %d)\nNumber: %d", header.region.w, header.region.h, header.number);
		if(!header.width) break;
	}
	printf("Recieved successfully!\n");	

	return 0;
}
