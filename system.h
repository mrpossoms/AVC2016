#ifndef AVC_SYSTEM_STATE
#define AVC_SYSTEM_STATE

#include "imu.h"

#define MAX_FEATURES 400

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
	// tracking
	depthWindow_t   window;

	// sensor readings
	sensorStatei_t  imu;	
} system_t;


//     ___ _     _          _    
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//                               
extern system_t SYS;

#endif
