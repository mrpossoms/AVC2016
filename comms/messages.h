#ifndef AVC_PROTOCOL_MESSAGES
#define AVC_PROTOCOL_MESSAGES

#include "types.h"
#include "system.h"

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

/*
typedef struct{
	vec3i16_t depth[MAX_FEATURES];
	uint16_t  detectedFeatures;	
} depthWindow_t;
*/
#ifdef __cplusplus
}
#endif

#endif
