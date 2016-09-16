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
#include "sensors/scanner.h"
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
	imuState_t    imu;
	scn_t         scanner;
	float         lastMeasureTime;
	float         lastEstTime;
	sensorStatef_t measured;
	sensorStatef_t filtered;
	readingFilter_t filters;

	struct {
		float len;
		float std_dev;
	} mag_expected;

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
		sensorStatef_t filtered;
	} sensors;
	pose_t pose;

	scn_datum_t lastDepth;
	scn_obstacle_t nearest_obs;

	gpsWaypoint_t waypoints[10];
	uint8_t hasGpsFix;

	struct {
		uint8_t throttle;
		uint8_t steering;
	} controls;
} __attribute__((packed)) sysSnap_t;
//     ___ _     _          _
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//
extern system_t SYS;

void sysPrintSnap(sysSnap_t* snap);
void sysTimerUpdate();
sysSHM_t* sysAttachSHM();
sysSnap_t sysSnapshot(system_t* sys);

#ifdef __cplusplus
}
#endif
#endif
