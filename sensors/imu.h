#ifndef AVC_IMU
#define AVC_IMU

#include <inttypes.h>
#include <sys/types.h>
#include <sys/time.h>
#include <unistd.h>   // UNIX standard function definitions
#include <fcntl.h>    // File control definitions
#include <stdio.h>
#include <strings.h>

#include "base/types.h"
#include "base/constants.h"
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SMF_SAMP_TYPE int16_t
#define WIN_SIZE 21

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
int imuPerformGyroCalibration(int fd_storage, int fd_imu, imuState_t* state);
int imuLoadCalibrationProfile(int fd_storage, imuState_t* state);

//    ___       _          ___     _ _ _
//   |   \ __ _| |_ __ _  | _ \___| | (_)_ _  __ _
//   | |) / _` |  _/ _` | |  _/ _ \ | | | ' \/ _` |
//   |___/\__,_|\__\__,_| |_| \___/_|_|_|_||_\__, |
//                                           |___/
int imuSetup(int fd, imuState_t* state);
int imuUpdateState(int fd, imuState_t* state, int contCal);
sensorStatei_t imuGetReadings(int fd);

#ifdef __cplusplus
}
#endif


#endif
