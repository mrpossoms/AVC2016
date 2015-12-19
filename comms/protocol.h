#ifndef AVC_COMM_PROTOCOL
#define AVC_COMM_PROTOCOL

#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>

#ifdef __cplusplus
extern "C"{
#endif

typedef enum{
	MSG_GREETING = 0,
	MSG_STATE,
	MSG_TRACKING,
	MSG_VIDEO,
	MSG_COUNT,
} msgType_e;

typedef struct{
	uint32_t  signature;
	msgType_e type;
} msgHeader_t;

typedef int(*rxProc_t)(int, struct sockaddr*);

int commInitClient(const char* hostname);
int commInitHost(uint16_t port);

int commSend(msgType_e type, const void* payload, size_t payloadSize, struct sockaddr* peer);

void commRegisterRxProc(msgType_e type, rxProc_t processor);
int  commListen();

#ifdef __cplusplus
}
#endif

#endif
