#include "protocol.h"

#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <errno.h>

#include <stdio.h>
#include <assert.h>

#define MSG_SIG 0xDEADBEEF

static rxProc_t RX_PROCESSORS[MSG_COUNT];

static int                SOCK;
static struct sockaddr_in HOST_ADDR;

static const uint32_t SIG = MSG_SIG;

int commInitClient(const char* hostname, uint16_t port, struct sockaddr_in* host)
{			
	struct sockaddr_in addr = {};
	struct hostent* he;
	int res = 0;

	addr.sin_family      = AF_INET;
	addr.sin_port        = htons(0);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	
	// open the socket
	if(!(SOCK = socket(AF_INET, SOCK_DGRAM, 0))){
		return -1;
	}

	// bind to the port
	if(bind(SOCK, (const struct sockaddr*)&addr, sizeof(addr)) < 0){
		return -2;
	}
	printf("Resolving '%s'\n", hostname);

	if(!(he = gethostbyname(hostname))){
		return -3;
	}

	HOST_ADDR.sin_family = AF_INET;
	HOST_ADDR.sin_port   = htons(port);
	memcpy((void*)&(HOST_ADDR.sin_addr), he->h_addr_list[0], he->h_length);
	
	memcpy(host, &HOST_ADDR, sizeof(HOST_ADDR));

	return 0;
}

int commInitHost(uint16_t port)
{
	struct sockaddr_in addr = {};

	addr.sin_family      = AF_INET;
	addr.sin_port        = htons(port);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	
	// open the socket
	if(!(SOCK = socket(AF_INET, SOCK_DGRAM, 0))){
		return -1;
	}

	// bind to the port
	if(bind(SOCK, (const struct sockaddr*)&addr, sizeof(addr)) < 0){
		return -2;
	}

	return 0;
}

int commSend(msgType_e type, const void* payload, size_t payloadSize, struct sockaddr_in* peer)
{
	msgHeader_t header = {};
	static char* buf;
	static size_t bufSize;


	header.signature = SIG;
	header.type      = type;

	if(!peer) return -2;

	if(!buf || bufSize < payloadSize){
		bufSize = (payloadSize ? sizeof(header) : payloadSize) * 2;
		buf = (char*)realloc((char*)buf, bufSize);
	}

	memcpy(buf, &header, sizeof(header));

	if(payloadSize){
		memcpy(buf + sizeof(header), payload, payloadSize);
	}
	
	return sendto(
		SOCK,
		buf,
		payloadSize + sizeof(header),
		0,
		(struct sockaddr*)peer,
		sizeof(struct sockaddr_in)
	);
}

void commRegisterRxProc(msgType_e type, rxProc_t processor)
{
	RX_PROCESSORS[type] = processor;
}

int commListen()
{
	struct sockaddr_in peer = {};
	socklen_t       socklen = sizeof(peer);
	msgHeader_t     header = {};
	struct timeval timeout = { 0, 10000 };
	fd_set readFd;

	FD_ZERO(&readFd);
	FD_SET(SOCK, &readFd);

	if(select(SOCK + 1, &readFd, 0, 0, &timeout) <= 0){
		return -1;
	}

	int bytes = recvfrom(SOCK, &header, sizeof(header), 0, (struct sockaddr*)&peer, &socklen);
	if(bytes != sizeof(header)){
		printf("Bad header size\n");
		return -2;
	}

	if(header.signature != MSG_SIG){
		printf("Bad signature\n");
		return -3;
	}

	assert(RX_PROCESSORS[header.type]);
	RX_PROCESSORS[header.type](SOCK, &peer);

	return 0;
}

