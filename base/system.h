#ifndef AVC_SYSTEM_STATE
#define AVC_SYSTEM_STATE

#include <errno.h>
#include <syslog.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>
#include <time.h>

#include "types.h"
#include "sensors/types.h"
#include "constants.h"

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
typedef struct {
	vec3i16_t depth[MAX_FEATURES];
	uint16_t  detectedFeatures;
} depthWindow_t;

typedef struct {
	vec3f_t pos;
	vec3f_t heading;
	vec3f_t accFrame[3];
} pose_t;

typedef struct {
	imuState_t    imu;
	float         lastMeasureTime;
	float         lastEstTime;
	sensorStatef_t measured;
	sensorStatef_t filtered;
	readingFilter_t filters;
	uint8_t hasGpsFix;
} sensors_t;

typedef struct {
	gpsWaypoint_t followLocation;
	time_t updatedTime;
} sysSHM_t;

typedef struct {
	// tracking
	depthWindow_t   window;

	// sensor measurements and estimates
	sensors_t sensors;
	pose_t pose;

	struct {
		gpsWaypointCont_t *start, *currentWaypoint;
	} route;

	sysSHM_t* shm; // shared memory region

	float timeUp; // time in seconds the system has been running
	float dt;     // time since last update
	int debugging;
	int magCal;
	int maxSpeed;
	int following;
} system_t;

typedef struct {
	struct {
		sensorStatei_t raw;
		sensorStatef_t cal;
		sensorStatef_t filtered;
	} imu;

	gpsWaypoint_t currentWaypoint;
	gpsWaypoint_t nextWaypoint;
	uint8_t hasGpsFix;
} sysSnap_t;
//     ___ _     _          _
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//
extern system_t SYS;

void sysTimerUpdate();
sysSHM_t* sysAttachSHM();
sysSnap_t sysSnapshot(system_t* sys);

#ifdef __cplusplus
}
#endif
#endif
