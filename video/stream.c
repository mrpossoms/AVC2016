#include <sys/socket.h>
#include <errno.h>

#include "compressor.h"
#include "stream.h"
#include "protocol.h"

#define INITIAL_MTU 4096 << 1 
//#define CMP_DEBUG

#ifdef CMP_DEBUG
#include <stdio.h>
#include <assert.h>
#endif

static char TX_BUF[INITIAL_MTU]; // buffer that will contain the compression stream and header of each chunk

// initial maximum transmission unit
// this will adjust if transmission failure occurs on account of length
static size_t STR_MTU = INITIAL_MTU;
static int FIRST_RUN = 1;

int txFrame(int sock, const struct sockaddr* destination, int width, int height, txState_t* state, const char* frameBuffer)
{
	if(!frameBuffer) return -1;

	// calculate some values that will be needed later on	
	size_t frameBufferSize = width * height; // size in bytes of the grayscale framebuffer

	if(FIRST_RUN){
		STR_MTU = frameBufferSize;

		while(STR_MTU > 6000){
			STR_MTU >>= 1;
		}

		FIRST_RUN = 0;
	}

	int regions = frameBufferSize / STR_MTU; // number of chunks that the framebuffer will be split up into
	size_t regionBytes = frameBufferSize / regions; // bytes in each chunk
	int regionHeight   = height / regions;          // height in pixels of each region


#ifdef CMP_DEBUG
	printf("Frame: %zu(B)\nRegions: %d\nRegion height: %d\n", 
		frameBufferSize,
		regions,
		regionHeight
	);
#endif

	// for each region, compress, and send
	for(int i = regions; i--;){	
		frameHeader_t header = vidCompressFrame(
			width, height, 
			frameBuffer + (i * regionBytes),
			regionBytes,
			TX_BUF + sizeof(frameHeader_t),
			STR_MTU - sizeof(frameHeader_t) // account for the header
		);

#ifdef CMP_DEBUG
		printf("Compressed region %d\n", i);
#endif

		// update the region information
		header.region.x = 0;
		header.region.y = regionHeight * i;
		header.region.w = width;
		header.region.h = regionHeight;

		// now that we have a header, copy it into the start of the txBuf
		((frameHeader_t*)TX_BUF)[0] = header;

#ifdef CMP_DEBUG
		printf("Region: (%d, %d) - (%d, %d)\n", header.region.x, header.region.y, header.region.w, header.region.h);
#endif

		commSend(MSG_VIDEO, NULL, 0, (struct sockaddr*)destination);

		int res = sendto(
			sock,
			TX_BUF,
			sizeof(frameHeader_t),
			0,
			destination,
			sizeof(*destination)
		);

		res = sendto(
			sock,
			TX_BUF + sizeof(frameHeader_t),
			header.bytes,
			0,
			destination,
			sizeof(*destination)
		);


#ifdef CMP_DEBUG
		printf("Transmission res %d (errno %d)\n", res, errno);
#endif

		// transmission failed, and we know it was because the 
		// chunk was too damn big
		if(res < 0 && errno == 40){
			errno = 0; // reset
			STR_MTU >>= 1;
			return -2;	
		}
	}

	return 0;
}
//-------------------------------------------------------------------------------------------
static char RX_BUF[INITIAL_MTU] = {};

int rxFrame(int sock, frameHeader_t* header, char** frameBuffer)
{
	size_t frameBufferSize;
	struct sockaddr sender = {};
	socklen_t addrLen;

	int res = recvfrom(
		sock, 
		header, 
		sizeof(frameHeader_t), 
		0,//MSG_WAITALL,
		&sender,
		&addrLen
	);

	if(res != sizeof(frameHeader_t))
	{
#ifdef CMP_DEBUG
		printf("Bad header size %d expected %zu\n", res, sizeof(frameHeader_t));
#endif
		return -1;
	}

#ifdef CMP_DEBUG
		printf("Region: (%d, %d) - (%d, %d)\n", header->region.x, header->region.y, header->region.w, header->region.h);
#endif

	frameBufferSize = header->width * header->height;
	if(!*frameBuffer){
		if(!(*frameBuffer = (char*)malloc(frameBufferSize))){
			return -2;
		}
#ifdef CMP_DEBUG
		printf("Allocated framebuffer %zu bytes\n", frameBufferSize);
#endif
	}

	res = recvfrom(
		sock, 
		RX_BUF, 
		header->bytes, 
		0,//MSG_WAITALL,
		&sender,
		&addrLen
	);

	if(res != header->bytes){
#ifdef CMP_DEBUG
		printf("Reading header failed\n");
#endif
		return -3;
	}

	vidDecompressFrame(
		header,
		RX_BUF + sizeof(frameHeader_t),
		(*frameBuffer) + (header->region.y * header->region.w) + header->region.x,
		header->bytes
	);
#ifdef CMP_DEBUG
	printf("decompressed %zu bytes\n", header->bytes);
#endif

	return 0;
}

