#ifndef AVC_STREAM
#define AVC_STREAM

#include <sys/socket.h>
#include "compressor.h"

int txFrame(int sock, const struct sockaddr* destination, int width, int height, const char* frameBuffer);
int rxFrame(int sock, frameHeader_t* header, char** frameBuffer);

#endif

