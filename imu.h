#ifndef AVC_IMU
#define AVC_IMU

#include <inttypes.h>
#include <time.h>
#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <stdio.h>
#include <strings.h>

typedef union{
	int16_t v[3];
	struct{
		int16_t x, y, z;
	};
} vec3i16_t;

typedef union{
	float v[3],
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
	readings_t lastReadings;
	vec3f_t    linearVel;
	vec3f_t    angularVel;
	time_t     lastTime;
	readings_t calibrationMinMax[2];
}imuState_t;

int imuPerformCalibration(int fd_storage, int fd_imu);
int imuLoadCalibrationProfile(int fd_storage, imuState_t* state);

int imuConfigSerial(int fd, int baud);
void imuSynch(int fd);
readings_t imuGetReadings(int fd);
int imuPerformCalibration(int fd_storage, int fd_imu, imuState_t* state)

#endif