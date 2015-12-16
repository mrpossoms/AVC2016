#ifndef AVC_COMPRESSOR 
#define AVC_COMPRESSOR

#include <inttypes.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

//    _____                  
//   |_   _|  _ _ __  ___ ___
//     | || || | '_ \/ -_|_-<
//     |_| \_, | .__/\___/__/
//         |__/|_|           
typedef struct{
	uint16_t x;
	uint16_t w;
	uint16_t y;
	uint16_t h;
} region_t;

typedef struct{
	uint16_t width, height;
	region_t region;
	uint32_t number;
	uint16_t bytes;
} frameHeader_t;

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
