#ifndef AVC_IMU
#define AVC_IMU

#include <inttypes.h>
#include <sys/time.h>
#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <stdio.h>
#include <strings.h>

//    _____                  
//   |_   _|  _ _ __  ___ ___
//     | || || | '_ \/ -_|_-<
//     |_| \_, | .__/\___/__/
//         |__/|_|           
typedef union{
	int16_t v[3];
	struct{
		int16_t x, y, z;
	};
} vec3i16_t;

typedef union{
	float v[3];
	struct{
		float x, y, z;
	};
} vec3f_t;

typedef struct{
	vec3i16_t accLinear;
	vec3i16_t accRotational;
	vec3i16_t mag;
} readings_t;

typedef struct{
	readings_t     lastReadings;
	vec3f_t        linearVel;
	vec3f_t        angularVel;
	struct timeval lastTime;
	readings_t     calibrationMinMax[2];
	int            isCalibrated;
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
readings_t imuGetReadings(int fd);

#endif
