#include "imu.h"
#include <fcntl.h>    // File control definitions 
#include <stdlib.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <arpa/inet.h>
#include <math.h>

#ifdef __linux__
#include <linux/i2c-dev.h>
#endif

#define ADDR_ACC_MAG 0x1D
#define ADDR_GYRO    0x6B

#define ACC_REG 0x28
#define MAG_REG 0x08
#define GYR_REG 0x28

#define G 9.8
#define WARMUP_SAMPLES 100

static int32_t ACCEL_MEAN[3];
static int READINGS_COLLECTED;
static sensorStatei_t READINGS[WARMUP_SAMPLES];

//     ___                      
//    / __|___ _ __  _ __  ___  
//   | (__/ _ \ '  \| '  \(_-<_ 
//    \___\___/_|_|_|_|_|_/__(_)
//                              
static void endianSwapVec3(vec3i16_t* v)
{
	return;
	v->x = ntohs(v->x);
	v->y = ntohs(v->y);
	v->z = ntohs(v->z);
}

static int sendByte(int fd, uint8_t devAddr, uint8_t dstReg, uint8_t byte)
{
#ifndef __linux__
	return 1;
#else

	uint8_t buf[] = { dstReg, byte };

	ioctl(fd, I2C_SLAVE, devAddr);
	write(fd, buf, 2);

	return 0;
#endif
}

static int requestBytes(int fd, uint8_t devAddr, uint8_t srcReg, void* dstBuf, size_t bytes)
{
#ifndef __linux__
	return 1;
#else

	uint8_t commByte;
	ioctl(fd, I2C_SLAVE, devAddr);
	commByte = 0x80 | srcReg;
	write(fd, &commByte, 1);

	if(read(fd, dstBuf, bytes) != bytes){
		return -1;
	}

	return 0;
#endif
}

//    ___       _          ___     _ _ _           
//   |   \ __ _| |_ __ _  | _ \___| | (_)_ _  __ _ 
//   | |) / _` |  _/ _` | |  _/ _ \ | | | ' \/ _` |
//   |___/\__,_|\__\__,_| |_| \___/_|_|_|_||_\__, |
//                                           |___/ 
sensorStatei_t imuGetReadings(int fd)
{
	static int isSetup;
	sensorStatei_t reading = {};
	int res = 0;
	
	if(!isSetup){
		sendByte(fd, ADDR_ACC_MAG, 0x20, 0x67);
		sendByte(fd, ADDR_ACC_MAG, 0x21, 0x00);
		sendByte(fd, ADDR_ACC_MAG, 0x26, 0x00);
		sendByte(fd, ADDR_GYRO, 0x20, 0x0F);		

		isSetup = 1;
		usleep(100000);
	}

	res += requestBytes(fd, ADDR_ACC_MAG, ACC_REG, &reading.linear, sizeof(vec3i16_t));
	res += requestBytes(fd, ADDR_ACC_MAG, GYR_REG, &reading.rotational, sizeof(vec3i16_t));
	res += requestBytes(fd, ADDR_ACC_MAG, MAG_REG, &reading.mag, sizeof(vec3i16_t));

	assert(res == 0);

	endianSwapVec3(&reading.linear);
	endianSwapVec3(&reading.rotational);
	endianSwapVec3(&reading.mag);

	return reading;
}

static float elapsedSeconds(imuState_t* state)
{
	// compute the elapsed time in seconds
	struct timeval now;
	gettimeofday(&now, NULL);
	int64_t usElapsed = (now.tv_sec - state->lastTime.tv_sec) * 1000000 +
	                    (now.tv_usec - state->lastTime.tv_usec);
	state->lastTime = now;

	return usElapsed / 1000000.0;	
}

static int comp(const void* a, const void* b)
{
	return *((SMF_SAMP_TYPE*)a) - *((SMF_SAMP_TYPE*)b);
}

void smfUpdate(medianWindow_t* win, SMF_SAMP_TYPE samp)
{
	SMF_SAMP_TYPE *window = win->window;
	window[win->nextSample++] = samp;

	// round robin
	win->nextSample %= WIN_SIZE;

	// sort the window
	qsort(window, WIN_SIZE, sizeof(SMF_SAMP_TYPE), comp);	

	win->median = window[WIN_SIZE / 2];
}

static float gaussian(float x, float mu, float sig)
{
	const float sqrt2pi = sqrtf(2 * M_PI);
	float xmmSqr = x - mu; xmmSqr *= xmmSqr;
	float num = powf(M_E, xmmSqr / (2 * sig) * (2 * sig));

	return 2.5 * sig * num / sig * sqrt2pi;
}

static void filterReading(imuState_t* state)
{
	assert(gaussian(0, 0, 32) == 1);

	for(int i = 3; i--;){
		smfUpdate(state->windows.linear + i,     state->lastReadings.linear.v[i]);
		smfUpdate(state->windows.rotational + i, state->lastReadings.rotational.v[i]);
		smfUpdate(state->windows.mag + i,        state->lastReadings.mag.v[i]);
	
		// apply a gaussian weighting to the readings to try to further remove noise
		// TODO: this may be useful for removing the offset instead mu could accomplish that.
		float w = gaussian(
			state->lastReadings.linear.v[i],
			ACCEL_MEAN[i],
			state->standardDeviations.linear.v[i]
		);

		state->standardDeviations.linear.v[i] -= state->standardDeviations.linear.v[i] * w;
	}
} 

static int obtainedStatisticalProps(imuState_t* state)
{
	static int isFinished;

	if(isFinished) return 1;
	if(READINGS_COLLECTED < WARMUP_SAMPLES){
		ACCEL_MEAN[0] += state->lastReadings.linear.x;
		ACCEL_MEAN[1] += state->lastReadings.linear.y;
		ACCEL_MEAN[2] += state->lastReadings.linear.z;
		READINGS[READINGS_COLLECTED++] = state->lastReadings;
		bzero(&state->lastReadings, sizeof(sensorStatei_t));

		return 0;
	}

	// compute the mean when still
	ACCEL_MEAN[0] /= WARMUP_SAMPLES;
	ACCEL_MEAN[1] /= WARMUP_SAMPLES;
	ACCEL_MEAN[2] /= WARMUP_SAMPLES;

	// compute the variance for standard deviation
	bzero(&state->standardDeviations, sizeof(sensorStatei_t));
	double variance[3] = {};
	for(int i = WARMUP_SAMPLES; i--;){
		for(int j = 3; j--;){
			double v = READINGS[i].linear.v[j] - ACCEL_MEAN[j];
			variance[j] += (v * v) / (float)WARMUP_SAMPLES;
		}
	}

	// set standard deviations
	state->standardDeviations.linear.x = sqrt(variance[0]);
	state->standardDeviations.linear.y = sqrt(variance[1]);
	state->standardDeviations.linear.z = sqrt(variance[2]);

	isFinished = 1;
	return 1;
}

void imuUpdateState(int fd, imuState_t* state)
{
	sensorStatei_t reading = {};

#ifdef __linux__
	reading = imuGetReadings(fd);
#endif

	state->lastReadings = reading;

	if(!obtainedStatisticalProps(state)) return;

	reading.linear.x -= ACCEL_MEAN[0];
	reading.linear.y -= ACCEL_MEAN[1];
	reading.linear.z -= ACCEL_MEAN[2];

	filterReading(state);

	// if we don't have a start time yet, don't bother to compute the velocities
	if(state->lastTime.tv_usec){
		float dt = elapsedSeconds(state);
		vec3f_t acc;

		if(state->isCalibrated){
			vec3i16_t* accMin = &state->calibrationMinMax[0].linear;
			vec3i16_t* accMax = &state->calibrationMinMax[1].linear;

			// map the readings to the 1G calibration window that was obtained
			// from the calibration profile
			acc.x = G * reading.linear.x * 2 / (float)(accMax->x - accMin->x);
			acc.y = G * reading.linear.y * 2 / (float)(accMax->y - accMin->y);
			acc.z = G * reading.linear.z * 2 / (float)(accMax->z - accMin->z);

		}
		else{
			// no calibration, just spit out the literal value
			acc.x = reading.linear.x;
			acc.y = reading.linear.y;
			acc.z = reading.linear.z;	
		}

		// integrate
		state->velocities.linear.x += acc.x * dt;
		state->velocities.linear.y += acc.y * dt;
		state->velocities.linear.z += acc.z * dt;

	}
	else{
		elapsedSeconds(state);
	}
}

//     ___      _ _ _             _   _          
//    / __|__ _| (_) |__ _ _ __ _| |_(_)___ _ _  
//   | (__/ _` | | | '_ \ '_/ _` |  _| / _ \ ' \
//    \___\__,_|_|_|_.__/_| \__,_|\__|_\___/_||_|
//                                               
int16_t axisAcc(char axis, int isMax, int fd_imu)
{
	printf(isMax ? "(+%c) [Press any key]\n" : "(-%c) [Press any key]\n", axis);

	getchar();
	sensorStatei_t readings = imuGetReadings(fd_imu);

	switch(axis){
		case 'X':
		case 'x':
			return readings.linear.x;
		case 'Y':
		case 'y':
			return readings.linear.y;
		case 'Z':
		case 'z':
			return readings.linear.z;
	}

	return 0;
}


int imuPerformCalibration(int fd_storage, int fd_imu, imuState_t* state)
{
	printf("Point each axis toward the center of the earth when prompted.\n");
	printf("[Press any key to begin]\n");
	getchar();

	printf("Calibrating accelerometer\n");
	state->calibrationMinMax[0].linear.x = axisAcc('x', 0, fd_imu);
	state->calibrationMinMax[1].linear.x = axisAcc('x', 1, fd_imu);
	state->calibrationMinMax[0].linear.y = axisAcc('y', 0, fd_imu);
	state->calibrationMinMax[1].linear.y = axisAcc('y', 1, fd_imu);
	state->calibrationMinMax[0].linear.z = axisAcc('z', 0, fd_imu);
	state->calibrationMinMax[1].linear.z = axisAcc('z', 1, fd_imu); 

	// store the results
	write(fd_storage, &state->calibrationMinMax, sizeof(sensorStatei_t) * 2);

	return 0;
}

int imuLoadCalibrationProfile(int fd_storage, imuState_t* state)
{
	int isOk = read(fd_storage, &state->calibrationMinMax, sizeof(sensorStatei_t) * 2) == sizeof(sensorStatei_t) * 2;

	if(isOk){
		state->isCalibrated = 1;
	}

	return 1;
}
