#ifndef AVC_IMU
#define AVC_IMU

#include <inttypes.h>
#include <sys/time.h>
#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <stdio.h>
#include <strings.h>

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SMF_SAMP_TYPE int16_t
#define WIN_SIZE 7

//    _____                  
//   |_   _|  _ _ __  ___ ___
//     | || || | '_ \/ -_|_-<
//     |_| \_, | .__/\___/__/
//         |__/|_|           
typedef struct{
	vec3i16_t linear;
	vec3i16_t rotational;
	vec3i16_t mag;
} sensorStatei_t;

typedef struct{
	vec3f_t linear;
	vec3f_t rotational;
	vec3f_t mag;
} sensorStatef_t;

typedef struct{
	SMF_SAMP_TYPE window[WIN_SIZE];
	int nextSample;
	SMF_SAMP_TYPE min, max, median;
} medianWindow_t;

typedef struct{
	medianWindow_t linear[3], rotational[3], mag[3];
} readingFilter_t;

typedef struct{
	sensorStatei_t  rawReadings;
	sensorStatef_t  adjReadings;
	readingFilter_t windows;
	sensorStatei_t  calibrationMinMax[2];
	sensorStatei_t  standardDeviations;
	int             isCalibrated;
}imuState_t;

static inline void print_v3i16(vec3i16_t* v)
{
	printf("(%d, %d, %d)", v->x, v->y, v->z);
}

static inline void print_v3f(vec3f_t* v)
{
	printf("(%f, %f, %f)", v->x, v->y, v->z);
}

//     ___      _ _ _             _   _          
//    / __|__ _| (_) |__ _ _ __ _| |_(_)___ _ _  
//   | (__/ _` | | | '_ \ '_/ _` |  _| / _ \ ' \
//    \___\__,_|_|_|_.__/_| \__,_|\__|_\___/_||_|
//                                               
int imuPerformCalibration(int fd_storage, int fd_imu, imuState_t* state);
int imuLoadCalibrationProfile(int fd_storage, imuState_t* state);

//    ___       _          ___     _ _ _           
//   |   \ __ _| |_ __ _  | _ \___| | (_)_ _  __ _ 
//   | |) / _` |  _/ _` | |  _/ _ \ | | | ' \/ _` |
//   |___/\__,_|\__\__,_| |_| \___/_|_|_|_||_\__, |
//                                           |___/ 
void imuUpdateState(int fd, imuState_t* state);
sensorStatei_t imuGetReadings(int fd);

#endif
#ifndef SLIDING_MEDIAN_FILTER
#define SLIDING_MEDIAN_FILTER

#include <sys/types.h>

void smfUpdate(medianWindow_t* win, SMF_SAMP_TYPE samp);

#ifdef __cplusplus
}
#endif
	

#endif
