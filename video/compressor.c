#include "stream.h"
// #include <lz4.h>
#include <assert.h>
#include <stdio.h>

#ifdef __linux__
#include <string.h>
#else
#include <strings.h>
#endif

#define BLOCK_BYTES 4096
#define NO_COMP

// static char CMP_BUF[LZ4_COMPRESSBOUND(BLOCK_BYTES)];

enum{
	IS_DECOMPRESSING = 0,
	IS_COMPRESSING   = 1,
};

// static size_t vidBlockify(
// 	const char* src, size_t srcSize,
// 	char* dst,       size_t dstSize,
// 	int isCompressing)
// {
// 	static LZ4_stream_t*       compStr;
// 	static LZ4_streamDecode_t* decodeStr;
//
// 	int inpOffset     = 0;
// 	size_t resultSize = 0;
//
// 	if(!compStr){
// 		compStr = LZ4_createStream();
// 		decodeStr = LZ4_createStreamDecode();
// 	}
//
// 	if(isCompressing){
// 		LZ4_resetStream(compStr);
// 		while(1){
// 			const char* inpPtr = src + inpOffset;
// 			int inpBytes = srcSize - inpOffset;
//
// 			if(inpBytes > BLOCK_BYTES) inpBytes = BLOCK_BYTES;
// 			if (!inpBytes) break;
//
// 			{
// 				int compBytes = LZ4_compress_fast_continue(compStr, inpPtr, dst, inpBytes, dstSize, 2);
//
// 				if(compBytes <= 0){
// 					break;
// 				}
//
// 				dst += compBytes;
// 				resultSize += compBytes;
// 				inpOffset += inpBytes;
// 			}
// 		}
// 	}
// 	else{
// 		bzero(decodeStr, sizeof(LZ4_streamDecode_t));
// 		while(1){
// 			const char* inpPtr = src + inpOffset;
// 			int inpBytes = srcSize - inpOffset;
//
// 			if(inpBytes > BLOCK_BYTES) inpBytes = BLOCK_BYTES;
// 			if (!inpBytes) break;
//
// 			{
// 				int decompBytes = LZ4_decompress_fast_continue(decodeStr, inpPtr, dst, inpBytes);
//
// 				if(decompBytes <= 0){
// 					break;
// 				}
//
// 				dst += decompBytes;
// 				resultSize += decompBytes;
// 				inpOffset += inpBytes;
// 			}
// 		}
// 	}
//
//
// 	return resultSize;
// }

frameHeader_t vidCompressFrame(int width, int height, const char* src, size_t srcSize, char* dst, size_t dstSize)
{
	static uint32_t frameNumber;

	frameHeader_t* start = (frameHeader_t*)dst;
	frameHeader_t frame = {};

	frame.width  = width;
	frame.height = height;
	frame.number = frameNumber++;
	frame.bytes  = 0;

	// copy the new header into the beginning of the dst buffer
	// offset the dst pointer to clean memory
	start[0] = frame;
	dst += sizeof(frameHeader_t);
	dstSize -= sizeof(frameHeader_t);

#ifdef NO_COMP
	memcpy((void*)dst, (void*)src, start->bytes = srcSize);
#else
	// start->bytes += vidBlockify(src, srcSize, dst, dstSize, IS_COMPRESSING);
#endif
	return *start;
}

int vidDecompressFrame(frameHeader_t* header, const char* src, char* dst, size_t dstSize)
{
#ifdef NO_COMP
	memcpy((void*)dst, (void*)src, header->bytes);
#else
	// if(vidBlockify(src, header->bytes, dst, dstSize, IS_DECOMPRESSING) != dstSize){
	// 	return -1;
	// }
#endif

	return 0;
}
