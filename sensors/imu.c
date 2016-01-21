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

static int32_t ACCEL_MEAN[3], GYRO_MEAN[3];
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
	res += requestBytes(fd, ADDR_GYRO,    GYR_REG, &reading.rotational, sizeof(vec3i16_t));
	res += requestBytes(fd, ADDR_ACC_MAG, MAG_REG, &reading.mag, sizeof(vec3i16_t));

	assert(res == 0);

	endianSwapVec3(&reading.linear);
	endianSwapVec3(&reading.rotational);
	endianSwapVec3(&reading.mag);

	return reading;
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
	//assert(gaussian(0, 0, 32) == 1);

	for(int i = WIN_SIZE; i--;){
		smfUpdate(state->windows.linear + i,     state->rawReadings.linear.v[i]);
		smfUpdate(state->windows.rotational + i, state->rawReadings.rotational.v[i]);
		smfUpdate(state->windows.mag + i,        state->rawReadings.mag.v[i]);
	}
} 

static int obtainedStatisticalProps(imuState_t* state)
{
	static int isFinished;

	if(isFinished) return 1;
	if(READINGS_COLLECTED < WARMUP_SAMPLES){
		ACCEL_MEAN[0] += state->rawReadings.linear.x;
		ACCEL_MEAN[1] += state->rawReadings.linear.y;
		ACCEL_MEAN[2] += state->rawReadings.linear.z;

		GYRO_MEAN[0] += state->rawReadings.rotational.x;
		GYRO_MEAN[1] += state->rawReadings.rotational.y;
		GYRO_MEAN[2] += state->rawReadings.rotational.z;

		READINGS[READINGS_COLLECTED] = state->rawReadings;
		bzero(&state->rawReadings, sizeof(sensorStatei_t));

		return 0;
	}

	// compute the mean when still
	ACCEL_MEAN[0] /= WARMUP_SAMPLES;
	ACCEL_MEAN[1] /= WARMUP_SAMPLES;
	ACCEL_MEAN[2] /= WARMUP_SAMPLES;
	GYRO_MEAN[0] /= WARMUP_SAMPLES;
	GYRO_MEAN[1] /= WARMUP_SAMPLES;
	GYRO_MEAN[2] /= WARMUP_SAMPLES;
	

	// compute the variance for standard deviation
	bzero(&state->standardDeviations, sizeof(sensorStatei_t));
	double varLin[3] = {}, varRot[3] = {};
	for(int i = WARMUP_SAMPLES; i--;){
		for(int j = 3; j--;){
			double v = READINGS[i].linear.v[j] - ACCEL_MEAN[j];
			double w = READINGS[i].rotational.v[j] - GYRO_MEAN[j];
			varLin[j] += (v * v) / (float)WARMUP_SAMPLES;
			varRot[j] += (w * w) / (float)WARMUP_SAMPLES;
		}
	}

	// set standard deviations
	state->standardDeviations.linear.x = sqrt(varLin[0]);
	state->standardDeviations.linear.y = sqrt(varLin[1]);
	state->standardDeviations.linear.z = sqrt(varLin[2]);
	state->standardDeviations.rotational.x = sqrt(varRot[0]);
	state->standardDeviations.rotational.y = sqrt(varRot[1]);
	state->standardDeviations.rotational.z = sqrt(varRot[2]);

	isFinished = 1;
	return 1;
}

static float map(float x, float min, float max)
{
	return (x - min) * 2 / (max - min) - 1.0f;
}

void imuUpdateState(int fd, imuState_t* state)
{
	sensorStatei_t reading = {};

	// get fresh data from the device
	reading = imuGetReadings(fd);
	state->rawReadings = reading;
	++READINGS_COLLECTED;

	// wait a little bit to grab the mean of the accelerometer
	if(!obtainedStatisticalProps(state)){
		return;
	}

	reading.linear.x -= ACCEL_MEAN[0];
	reading.linear.y -= ACCEL_MEAN[1];
	reading.linear.z -= ACCEL_MEAN[2];
	reading.rotational.x -= ACCEL_MEAN[0];
	reading.rotational.y -= ACCEL_MEAN[1];
	reading.rotational.z -= ACCEL_MEAN[2];

	filterReading(state);

	if(state->isCalibrated){
		vec3i16_t* accMin = &state->calMinMax[0].linear;
		vec3i16_t* accMax = &state->calMinMax[1].linear;
		vec3i16_t* magMin = &state->calMinMax[0].mag;
		vec3i16_t* magMax = &state->calMinMax[1].mag;

		// update the mag window
		int updatedMagWindow = 0;
		if(reading.mag.x > magMax->x){ magMax->x = reading.mag.x; updatedMagWindow = 1; }
		if(reading.mag.x < magMin->x){ magMin->x = reading.mag.x; updatedMagWindow = 1; }
		if(reading.mag.y > magMax->y){ magMax->y = reading.mag.y; updatedMagWindow = 1; }
		if(reading.mag.y < magMin->y){ magMin->y = reading.mag.y; updatedMagWindow = 1; }

		// write new mag min and max if applicable
		if(updatedMagWindow){
			int calFd = open("./imu.cal", O_WRONLY);

			assert(calFd > 0);
			write(calFd, &state->calMinMax, sizeof(state->calMinMax));
			close(calFd);
		}

		// map the readings to the 1G [-1, 1] calibration window that was obtained
		// from the calibration profile
		state->adjReadings.linear.x = G * map(reading.linear.x, accMin->x, accMax->x);
		state->adjReadings.linear.y = G * map(reading.linear.y, accMin->y, accMax->y);
		state->adjReadings.linear.z = G * map(reading.linear.z, accMin->z, accMax->z);

		// map the mag from [-1, 1] based on the measured range
		state->adjReadings.mag.x = map(reading.mag.x, magMin->x, magMax->x);
		state->adjReadings.mag.y = map(reading.mag.y, magMin->y, magMax->y);
		state->adjReadings.mag.z = map(reading.mag.z, magMin->z, magMax->z);
	}
	else{
		// no calibration, just spit out the literal value
		state->adjReadings.linear.x = reading.linear.x;
		state->adjReadings.linear.y = reading.linear.y;
		state->adjReadings.linear.z = reading.linear.z;	
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
	state->calMinMax[0].linear.x = axisAcc('x', 0, fd_imu);
	state->calMinMax[1].linear.x = axisAcc('x', 1, fd_imu);
	state->calMinMax[0].linear.y = axisAcc('y', 0, fd_imu);
	state->calMinMax[1].linear.y = axisAcc('y', 1, fd_imu);
	state->calMinMax[0].linear.z = axisAcc('z', 0, fd_imu);
	state->calMinMax[1].linear.z = axisAcc('z', 1, fd_imu);

	// store the results
	write(fd_storage, &state->calMinMax, sizeof(state->calMinMax));

	return 0;
}

int imuLoadCalibrationProfile(int fd_storage, imuState_t* state)
{
	int isOk = read(fd_storage, &state->calMinMax, sizeof(state->calMinMax)) == sizeof(state->calMinMax);
	if(isOk){
		state->isCalibrated = 1;
	}

	return 0;
}
