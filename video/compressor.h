#ifndef AVC_COMPRESSOR 
#define AVC_COMPRESSOR

#include <inttypes.h>
#include <stdlib.h>
#include "comms/messages.h"

#ifdef __cplusplus
extern "C" {
#endif

//    ___             _   _             
//   | __|  _ _ _  __| |_(_)___ _ _  ___
//   | _| || | ' \/ _|  _| / _ \ ' \(_-<
//   |_| \_,_|_||_\__|\__|_\___/_||_/__/
//                                      
frameHeader_t vidCompressFrame(int width, int height, const char* src, size_t srcSize, char* dst, size_t dstSize);
int           vidDecompressFrame(frameHeader_t* header, const char* src, char* dst, size_t dstSize);

#ifdef __cplusplus
}
#endif

#endif
