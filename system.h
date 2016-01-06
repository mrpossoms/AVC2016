#ifndef AVC_SYSTEM_STATE
#define AVC_SYSTEM_STATE

#include "types.h"
#include "sensors/imu.h"

#define MAX_FEATURES 900

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
	vec3f_t velocity;
	vec3f_t heading;
}objectState_t;

typedef struct{
	imuState_t    imu;
	float         lastMeasureTime;
	float         lastEstTime;
	objectState_t measured;
	objectState_t estimated;
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
} system_t;

//     ___ _     _          _    
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//                               
extern system_t SYS;

#endif
