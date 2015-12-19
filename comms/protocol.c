#include "protocol.h"

#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <sys/select.h>

#define MSG_SIG 0xDEADBEEF

/*
typedef enum{
	MSG_GREETING = 0,
	MSG_STATE,
	MSG_TRACKING,
	MSG_VIDEO,
} msgType_e;

typedef struct{
	uint32_t  signature;
	msgType_e type;
} msgHeader_t;

typedef int(rxProc*)(int socket, struct sockaddr* peer) rxProc_t;
*/

static rxProc_t RX_PROCESSORS[MSG_COUNT];

static int                SOCK;
static struct sockaddr_in HOST_ADDR;

static const uint32_t SIG = MSG_SIG;

int commInitClient(const char* hostname)
{			
	struct hostent* he;

	// try to resolve the host's name
	if(!(he = gethostbyname(hostname))){
		return -1;
	}

	HOST_ADDR.sin_family = AF_INET;
	memcpy((void*)&HOST_ADDR.sin_addr, he->h_addr_list[0], he->h_length);

	// open the socket
	if(!(SOCK = socket(AF_INET, SOCK_DGRAM, 0))){
		return -2;
	}

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
	if(bind(SOCK, (const struct sockaddr*)&addr, sizeof(addr) < 0)){
		return -2;
	}

	return 0;
}

int commSend(msgType_e type, const void* payload, size_t payloadSize, struct sockaddr* peer)
{
	msgHeader_t header = {};
	static char* buf;
	static size_t bufSize;

	header.signature = SIG;
	header.type      = type;

	if(!payload) return -1;
	if(!peer) return -2;

	if(bufSize < payloadSize){
		bufSize = payloadSize * 2;
		buf = realloc(buf, bufSize);
	}

	memcpy(buf, &header, sizeof(header));
	memcpy(buf + sizeof(header), payload, payloadSize);

	return sendto(
		SOCK,
		buf,
		payloadSize+sizeof(header),
		0,
		peer,
		sizeof(*peer)
	);
}

void commRegisterRxProc(msgType_e type, rxProc_t processor)
{
	RX_PROCESSORS[type] = processor;
}

int commListen()
{
	struct sockaddr peer = {};
	socklen_t       socklen;
	msgHeader_t     header = {};
	struct timeval timeout = { 0, 5e5 };
	fd_set readFd;

	FD_ZERO(&readFd);
	FD_SET(SOCK, &readFd);

	if(select(SOCK + 1, &readFd, 0, 0, &timeout)){
		return -1;
	}

	recvfrom(SOCK, &header, sizeof(header), 0, &peer, &socklen);

	if(header.signature != MSG_SIG){
		return -2;
	}

	RX_PROCESSORS[header.type](SOCK, &peer);

	return 0;
}

