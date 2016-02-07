#ifndef AVC_SYSTEM_STATE
#define AVC_SYSTEM_STATE

#include <errno.h>
#include <syslog.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "types.h"
#include "sensors/imu.h"

#define MAX_FEATURES 900

#define SYS_ERR(fmt, ...){\
	char msg[256], prefix[512];\
	sprintf(prefix, "(%d)(%s @ ln %d): ", errno, __FILE__, __LINE__);\
	sprintf(msg, fmt, __VA_ARGS__);\
	strncat(prefix, msg, 512);\
	syslog(0, "%s", prefix);\
}\

#ifdef __cplusplus
extern "C"{
#endif
//    _____                  
//   |_   _|  _ _ __  ___ ___
//     | || || | '_ \/ -_|_-<
//     |_| \_, | .__/\___/__/
//         |__/|_|           
typedef struct{
	vec3i16_t depth[MAX_FEATURES];
	uint16_t  detectedFeatures;	
} depthWindow_t;

typedef struct{
	vec3f_t position;
	struct{
		vec3f_t linear;
		vec3f_t rotational;
	} velocity;
	vec3f_t heading;
	vec3f_t goalHeading;
}objectState_t;

typedef struct{
	imuState_t    imu;
	float         lastMeasureTime;
	float         lastEstTime;
	objectState_t measured;
	objectState_t estimated;
	uint8_t hasGpsFix;
}fusedObjState_t;

typedef struct{
	// tracking
	depthWindow_t   window;

	// sensor measurements and estimates
	fusedObjState_t body;

	struct{
		gpsWaypointCont_t *start, *currentWaypoint;
	} route;

	float timeUp; // time in seconds the system has been running
	int debugging;
} system_t;

//     ___ _     _          _    
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//                               
extern system_t SYS;

void sysTimerUpdate();

#ifdef __cplusplus
}
#endif
#endif
