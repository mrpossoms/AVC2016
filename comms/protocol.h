#ifndef AVC_COMM_PROTOCOL
#define AVC_COMM_PROTOCOL

#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/ip.h>

#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	MSG_GREETING = 0,
	MSG_STATE    = 1,
	MSG_TRACKING = 2,
	MSG_VIDEO    = 3,
	MSG_COUNT    = 4,
} msgType_e;

typedef struct{
	uint32_t signature;
	uint32_t type;
} msgHeader_t;

typedef int(*rxProc_t)(int, struct sockaddr_in*);

int commInitClient(const char* hostname, uint16_t port, struct sockaddr_in* host);
int commInitHost(uint16_t port);

int commSend(msgType_e type, const void* payload, size_t payloadSize, struct sockaddr_in* peer);

void commRegisterRxProc(msgType_e type, rxProc_t processor);
int  commListen();

#ifdef __cplusplus
}
#endif

#endif
