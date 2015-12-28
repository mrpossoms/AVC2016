#ifndef AVC_RC
#define AVC_RC

#include<inttypes.h>

typedef struct{
	uint32_t id;
	uint8_t  throttle;
	uint8_t  steering;
} rcMessage_t;

#endif
