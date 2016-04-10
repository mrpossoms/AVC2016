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

//    _  _     _
//   | || |___| |_ __  ___ _ _ ___
//   | __ / -_) | '_ \/ -_) '_(_-<
//   |_||_\___|_| .__/\___|_| /__/
//              |_|
static float map(float x, float min, float max)
{
	return (x - min) * 2 / (max - min) - 1.0f;
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
		i2cSendByte(fd, ADDR_ACC_MAG, 0x20, 0x67);
		i2cSendByte(fd, ADDR_ACC_MAG, 0x21, 0x00);
		i2cSendByte(fd, ADDR_ACC_MAG, 0x26, 0x00);
		i2cSendByte(fd, ADDR_GYRO, 0x20, 0x0F);

		isSetup = 1;
		usleep(1000000);
	}

	res += i2cReqBytes(fd, ADDR_ACC_MAG, ACC_REG, &reading.acc, sizeof(vec3i16_t));
	res += i2cReqBytes(fd, ADDR_GYRO,    GYR_REG, &reading.gyro, sizeof(vec3i16_t));
	res += i2cReqBytes(fd, ADDR_ACC_MAG, MAG_REG, &reading.mag, sizeof(vec3i16_t));

	assert(res == 0);

	return reading;
}

//    ___ _ _ _           _
//   | __(_) | |_ ___ _ _(_)_ _  __ _
//   | _|| | |  _/ -_) '_| | ' \/ _` |
//   |_| |_|_|\__\___|_| |_|_||_\__, |
//                              |___/
static int filterReading(imuState_t* state)
{
	int err = 0;
	readingFilter_t* f = &state->filters;

	if(!state->filters.isSetup){
		for(int i = 3; i--;){
			kf_t* filter = (&f->acc) + i;
			kfCreateFilter(filter, 3);

			if(!i){
				kfMatScl(filter->matQ, filter->matQ, 0.001);
				kfMatScl(filter->matR, filter->matR, 10);
			}
			else{
				kfMatScl(filter->matQ, filter->matQ, 0.001);
				kfMatScl(filter->matR, filter->matR, 0.1);
			}

			if(err) return err;
		}

		f->isSetup = 1;
	}

	for(int i = 3; i--;){
		kf_t* filter = (&f->acc) + i;
		float* src = (&state->cal.acc)[i].v;      // calibrated measurements
		float* dst = (&state->filtered.acc)[i].v; // filtered calibrated measurements

		kfPredict(filter, NULL);
		kfUpdate(filter, dst, src);
	}

	return err;
}
//-----------------------------------------------------------------------------
static int hasStatProps(imuState_t* state)
{
	static int isFinished;

	if(isFinished) return 1;
	if(READINGS_COLLECTED < WARMUP_SAMPLES){
		// vec3Add(state->means.acc, state->mean.acc, state->raw.acc);
		for(int i = 3; i--;){
			ACCEL_MEAN[i] += state->raw.acc.v[i];
			GYRO_MEAN[i]  += state->raw.gyro.v[i];
		}
		++READINGS_COLLECTED;
		return 0;
	}

	// compute the mean when still
	for(int i = 3; i--;){
		ACCEL_MEAN[i] /= WARMUP_SAMPLES;
		GYRO_MEAN[i] /= WARMUP_SAMPLES;
	}

	// compute the variance for standard deviation
	bzero(&state->standardDeviations, sizeof(sensorStatei_t));
	double varLin[3] = {};
	for(int i = WARMUP_SAMPLES; i--;){
		for(int j = 3; j--;){
			double v = READINGS[i].acc.v[j] - ACCEL_MEAN[j];
			//double w = READINGS[i].gyro.v[j] - GYRO_MEAN[j];
			varLin[j] += (v * v) / (float)WARMUP_SAMPLES;
		}
	}

	// set standard deviations
	for(int i = 3; i--;){
		state->standardDeviations.acc.v[i] = sqrt(varLin[i]);
	}

	isFinished = 1;
	return 1;
}

//    __  __      _
//   |  \/  |__ _(_)_ _
//   | |\/| / _` | | ' \
//   |_|  |_\__,_|_|_||_|
//                          
int imuSetup(int fd, imuState_t* imu)
{
	do
	{
		imu->raw = imuGetReadings(fd);
		usleep(1000);
	}
	while(!hasStatProps(imu));
	
	return 0;
}

int imuUpdateState(int fd, imuState_t* imu, int contCal)
{
	sensorStatei_t reading = imuGetReadings(fd);
	imu->raw = reading;

	if(imu->isCalibrated){
		vec3i16_t* accMin = &imu->calMinMax[0].acc;
		vec3i16_t* accMax = &imu->calMinMax[1].acc;
		vec3i16_t* magMin = &imu->calMinMax[0].mag;
		vec3i16_t* magMax = &imu->calMinMax[1].mag;

		// update the mag window
		int updatedMagWindow = 0;

		if(contCal){
			// check each axis of the min and max windows, update them accordingly
			for(int i = 3; i--;){
				if(reading.mag.v[i] > magMax->v[i]){
					magMax->v[i] = reading.mag.v[i];
					updatedMagWindow = 1;
				}

				if(reading.mag.v[i] < magMin->v[i]){
					magMin->v[i] = reading.mag.v[i];
					updatedMagWindow = 1;
				}

				if(updatedMagWindow)
				printf("%d ", reading.mag.v[i]);
			} 
			if(updatedMagWindow) printf("\n");
		}

		// write new mag min and max if applicable
		if(updatedMagWindow){
			
			int calFd = open("./imu.cal", O_WRONLY);

			assert(calFd > 0);
			write(calFd, &imu->calMinMax, sizeof(imu->calMinMax));
			close(calFd);
		}

		// map the readings to the 1G [-1, 1] calibration window that was obtained
		// from the calibration profile
		sensorStatef_t* cal = &imu->cal;
		sensorStatei_t* raw = &imu->raw;

		// scale the accelerometer readings into the calibrated range
		for(int i = 3; i--;){
			cal->acc.v[i] = LIL_G * map(raw->acc.v[i], accMin->v[i], accMax->v[i]);
		}

		// subtract the gyro bias
		for(int i = 3; i--;){
			cal->gyro.v[i] = raw->gyro.v[i] - GYRO_MEAN[i];
		}

		// map the mag from [-1, 1] based on the measured range
		{
			vec3f_t magRad = {};
			vec3f_t magOff = {};

			// compute the radii for each axis
			vec3Sub(magRad, *magMax, *magMin);
			vec3Scl(magRad, magRad, 1.0f / 2.0f);

			// compute the offset vector
			vec3Sub(magOff, *magMax, magRad);

			// do the rest!
			vec3Sub(cal->mag, raw->mag, magOff);
			vec3Div(cal->mag, cal->mag, magRad);
		}

		filterReading(imu);
	}

	return 0;
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
			return readings.acc.x;
		case 'Y':
		case 'y':
			return readings.acc.y;
		case 'Z':
		case 'z':
			return readings.acc.z;
	}

	return 0;
}
//-----------------------------------------------------------------------------
int imuPerformCalibration(int fd_storage, int fd_imu, imuState_t* state)
{
	printf("Point each axis toward the center of the earth when prompted.\n");
	printf("[Press any key to begin]\n");
	getchar();

	printf("Calibrating accelerometer\n");
	state->calMinMax[0].acc.x = axisAcc('x', 0, fd_imu);
	state->calMinMax[1].acc.x = axisAcc('x', 1, fd_imu);
	state->calMinMax[0].acc.y = axisAcc('y', 0, fd_imu);
	state->calMinMax[1].acc.y = axisAcc('y', 1, fd_imu);
	state->calMinMax[0].acc.z = axisAcc('z', 0, fd_imu);
	state->calMinMax[1].acc.z = axisAcc('z', 1, fd_imu);

	// store the results
	write(fd_storage, &state->calMinMax, sizeof(state->calMinMax));

	return 0;
}
//-----------------------------------------------------------------------------
int imuLoadCalibrationProfile(int fd_storage, imuState_t* state)
{
	int isOk = read(fd_storage, &state->calMinMax, sizeof(state->calMinMax)) == sizeof(state->calMinMax);
	if(isOk){
		state->isCalibrated = 1;
	}

	return 0;
}
