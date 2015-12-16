#ifndef AVC_STREAM
#define AVC_STREAM

#include <sys/socket.h>
#include "compressor.h"

typedef struct{
	int    regions;
	float* regionVariance;
	int    regionLastId;
} txState_t;

int txFrame(int sock, const struct sockaddr* destination, int width, int heighti, txState_t* txState, const char* frameBuffer);
int rxFrame(int sock, frameHeader_t* header, char** frameBuffer);

#endif

