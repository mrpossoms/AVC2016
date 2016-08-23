#ifndef AVC_SENSOR_TYPES
#define AVC_SENSOR_TYPES

#include <inttypes.h>
#include "base/types.h"

#define CAL_MIN 0
#define CAL_MAX 1

#define MAG 0
#define STD_DEV 1

typedef struct{
	vec3i16_t acc;
	vec3i16_t gyro;
	vec3i16_t mag;
} sensorStatei_t;

typedef struct{
	vec3f_t acc;
	vec3f_t gyro;
	vec3f_t mag;
	vec3d_t gps;
	float   enc_dist;
	float   enc_dist_delta;
} sensorStatef_t;

typedef struct{
	kf_t acc, gyro, mag, gps;
	sensorStatef_t  means;
	sensorStatef_t  stdDevs;
	float   samples;
	uint8_t isSetup;
} readingFilter_t;

typedef struct{
	sensorStatei_t  raw;
	sensorStatef_t  cal;
	sensorStatei_t  calMinMax[2];
	int             isCalibrated;
}imuState_t;

void log_senI(sensorStatei_t* s);
void log_senF(sensorStatef_t* s);

#endif
