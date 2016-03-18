#include "imu.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#define ADDR_ACC_MAG 0x1D
#define ADDR_GYRO    0x6B

#define ACC_REG 0x28
#define MAG_REG 0x08
#define GYR_REG 0x28

#define WARMUP_SAMPLES 10

static int32_t ACCEL_MEAN[3], GYRO_MEAN[3];
static int READINGS_COLLECTED;
static sensorStatei_t READINGS[WARMUP_SAMPLES];

//     ___
//    / __|___ _ __  _ __  ___
//   | (__/ _ \ '  \| '  \(_-<_
//    \___\___/_|_|_|_|_|_/__(_)
//



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
		i2cSendByte(fd, ADDR_ACC_MAG, 0x20, 0x67);
		i2cSendByte(fd, ADDR_ACC_MAG, 0x21, 0x00);
		i2cSendByte(fd, ADDR_ACC_MAG, 0x26, 0x00);
		i2cSendByte(fd, ADDR_GYRO, 0x20, 0x0F);

		isSetup = 1;
		usleep(1000000);
	}

	res += i2cReqBytes(fd, ADDR_ACC_MAG, ACC_REG, &reading.linear, sizeof(vec3i16_t));
	res += i2cReqBytes(fd, ADDR_GYRO,    GYR_REG, &reading.rotational, sizeof(vec3i16_t));
	res += i2cReqBytes(fd, ADDR_ACC_MAG, MAG_REG, &reading.mag, sizeof(vec3i16_t));

	assert(res == 0);

	return reading;
}

static int filterReading(imuState_t* state)
{
	int err = 0;
	readingFilter_t* f = &state->filter;

	if(!state->filter.isSetup){
		for(int i = 3; i--;){
			kf_t* filter = (&f->linear) + i;
			kfCreateFilter(filter, 3);
			kfMatScl(filter->matQ, filter->matQ, 0.001);
			kfMatScl(filter->matR, filter->matR, 0.1);

			if(err) return err;
		}

		f->isSetup = 1;
	}

	for(int i = 3; i--;){
		kf_t* filter = (&f->linear) + i;
		float* x = (&state->adjReadings.linear)[i].v;

		kfPredict(filter, NULL);
		kfUpdate(filter, x, x);
	}

	return err;
}

static int hasStatProps(imuState_t* state)
{
	static int isFinished;

	if(isFinished) return 1;
	if(READINGS_COLLECTED < WARMUP_SAMPLES){
		// vec3Add(state->means.linear, state->mean.linear, state->rawReadings.linear);
		for(int i = 3; i--;){
			ACCEL_MEAN[i] += state->rawReadings.linear.v[i];
			GYRO_MEAN[i]  += state->rawReadings.rotational.v[i];
		}
		++READINGS_COLLECTED;
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
	double varLin[3] = {};
	for(int i = WARMUP_SAMPLES; i--;){
		for(int j = 3; j--;){
			double v = READINGS[i].linear.v[j] - ACCEL_MEAN[j];
			//double w = READINGS[i].rotational.v[j] - GYRO_MEAN[j];
			varLin[j] += (v * v) / (float)WARMUP_SAMPLES;
		}
	}

	// set standard deviations
	state->standardDeviations.linear.x = sqrt(varLin[0]);
	state->standardDeviations.linear.y = sqrt(varLin[1]);
	state->standardDeviations.linear.z = sqrt(varLin[2]);
	// state->standardDeviations.rotational.x = sqrt(varRot[0]);
	// state->standardDeviations.rotational.y = sqrt(varRot[1]);
	// state->standardDeviations.rotational.z = sqrt(varRot[2]);

	isFinished = 1;
	return 1;
}

static float map(float x, float min, float max)
{
	return (x - min) * 2 / (max - min) - 1.0f;
}

void imuUpdateState(int fd, imuState_t* state, int contCal)
{
	sensorStatei_t reading = {};

	// get fresh data from the device
	reading = imuGetReadings(fd);
	state->rawReadings = reading;

	if(!hasStatProps(state)) return;


	if(state->isCalibrated){
		vec3i16_t* accMin = &state->calMinMax[0].linear;
		vec3i16_t* accMax = &state->calMinMax[1].linear;
		vec3i16_t* magMin = &state->calMinMax[0].mag;
		vec3i16_t* magMax = &state->calMinMax[1].mag;

		// update the mag window
		int updatedMagWindow = 0;

		if(contCal){
			if(reading.mag.x > magMax->x){ magMax->x = reading.mag.x; updatedMagWindow = 1; }
			if(reading.mag.x < magMin->x){ magMin->x = reading.mag.x; updatedMagWindow = 1; }
			if(reading.mag.y > magMax->y){ magMax->y = reading.mag.y; updatedMagWindow = 1; }
			if(reading.mag.y < magMin->y){ magMin->y = reading.mag.y; updatedMagWindow = 1; }
			if(reading.mag.z > magMax->z){ magMax->z = reading.mag.z; updatedMagWindow = 1; }
			if(reading.mag.z < magMin->z){ magMin->z = reading.mag.z; updatedMagWindow = 1; }
		}

		// write new mag min and max if applicable
		if(updatedMagWindow){
			int calFd = open("./imu.cal", O_WRONLY);

			assert(calFd > 0);
			write(calFd, &state->calMinMax, sizeof(state->calMinMax));
			close(calFd);
		}

		// map the readings to the 1G [-1, 1] calibration window that was obtained
		// from the calibration profile
		sensorStatef_t* adj_r = &state->adjReadings;
		sensorStatei_t* raw = &state->rawReadings;
		adj_r->linear.x = LIL_G * map(raw->linear.x, accMin->x, accMax->x);
		adj_r->linear.y = LIL_G * map(raw->linear.y, accMin->y, accMax->y);
		adj_r->linear.z = LIL_G * map(raw->linear.z, accMin->z, accMax->z);

		// map the mag from [-1, 1] based on the measured range
		{
			vec3f_t magRad = {};
			vec3f_t magOff = {};

			// compute the radii for each axis
			vec3Sub(magRad, *magMax, *magMin);
			vec3Scl(magRad, magRad, 1.0f / 2.0f);

			assert(!vec3fIsNan(&magRad));

			// compute the offset vector
			vec3Sub(magOff, *magMax, magRad);

			assert(!vec3fIsNan(&magOff));

			// do the rest!
			vec3Sub(adj_r->mag, raw->mag, magOff);

			//assert(magRad.x > 0 && magRad.y > 0 && magRad.z > 0);

			vec3Div(adj_r->mag, adj_r->mag, magRad);

			assert(!vec3fIsNan(&adj_r->mag));
/*
			adj_r->mag.x = map(raw->mag.x, magMin->x, magMax->x);
			adj_r->mag.y = map(raw->mag.y, magMin->y, magMax->y);
			adj_r->mag.z = map(raw->mag.z, magMin->z, magMax->z);
*/
		}

		adj_r->rotational.x = raw->rotational.x - GYRO_MEAN[0];
		adj_r->rotational.y = raw->rotational.y - GYRO_MEAN[1];
		adj_r->rotational.z = raw->rotational.z - GYRO_MEAN[2];

		filterReading(state);
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

static float elapsed(struct timeval last, struct timeval now)
{
	float us = (now.tv_sec - last.tv_sec) * 1000000 + (now.tv_usec - last.tv_usec);
	return us / 1000000.0;
}

float axisGyro(char axis, imuState_t* imu, int fd_imu)
{
	vec3f_t lastHeading = {}, heading = {};
	struct timeval lastTime = {}, now = {};
	float w = 0;
	float sf = 1;

	//printf(isMax ? "(+%c) [Press any key]\n" : "(-%c) [Press any key to start sampling]\n", axis);
	//getchar();


	for(int i = 1000; i--;){
		imuUpdateState(fd_imu, imu, 0);
		usleep(1000);
	}

	gettimeofday(&lastTime, NULL);

	lastHeading.x = imu->adjReadings.mag.x;
	lastHeading.y = imu->adjReadings.mag.y;
	lastHeading.z = 0;
	lastHeading = vec3fNorm(&lastHeading);
	printf("lastHeading = %f, %f, %f\n", lastHeading.x, lastHeading.y, lastHeading.z);

	for(int i = 1000; i--;){
		float dt = 0;

		// wait for ~10ms to elapse
		float acc = 0;
		unsigned int samples = 0;
		while((dt = elapsed(lastTime, now)) < 0.01){
			imuUpdateState(fd_imu, imu, 0);
			switch(axis){
				case 'X':
				case 'x':
					acc += imu->adjReadings.rotational.x;
				case 'Y':
				case 'y':
					acc += imu->adjReadings.rotational.y;
				case 'Z':
				case 'z':
					acc += imu->adjReadings.rotational.z;
			}
			++samples;
			gettimeofday(&now, NULL);
		}
		acc /= samples;

		heading.x = imu->adjReadings.mag.x;
		heading.y = imu->adjReadings.mag.y;
		heading.z = 0;
		heading = vec3fNorm(&heading);

		printf("heading = %f, %f, %f\n", heading.x, heading.y, heading.z);

		float w0 = w;
		float dw = acos(vec3fDot(&heading, &lastHeading)) / dt;
		w += dw;

		float expAcc = (w - w0) / dt;

		// expAcc = acc * sf
		// sf = expAcc / acc;
		if(acc != 0){
			sf = expAcc / acc;
		}

		printf("%0.3f rads/s %0.3f rads/s^2 (%f)\n", w, expAcc, sf);

		lastTime = now;
		lastHeading = heading;
	}

	return sf;
}

int imuPerformGyroCalibration(int fd_storage, int fd_imu, imuState_t* state)
{
	printf("Rotate on the z axis, back and forth.\n");
	printf("[Press any key to begin]\n");
	getchar();

	axisGyro('z', state, fd_imu);

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
