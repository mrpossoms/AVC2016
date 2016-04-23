#include "imu.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#define ADDR_ACC_MAG 0x1D
#define ADDR_GYRO    0x6B

#define ACC_REG 0x28
#define MAG_REG 0x08
#define GYR_REG 0x28

#define WARMUP_SAMPLES 100

sensorStatef_t applyCalibration(sensorStatei_t* raw, sensorStatei_t* calMinMax);
int contMagCal(sensorStatei_t* raw, sensorStatei_t* calMinMax);

static double ACCEL_MEAN[3], GYRO_MEAN[3], MAG_MEAN[3];
static int READINGS_COLLECTED;
static sensorStatef_t READINGS[WARMUP_SAMPLES];

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
	vec3f_t* stdDevs = (vec3f_t*)&state->standardDeviations;

	if(!state->filters.isSetup){
		for(int i = 3; i--;){
			kf_t* filter = (&f->acc) + i;
			kfCreateFilter(filter, 3);

			// copy collected standard deviations into the measurement
			// covariance matrices for the filters
			for(int j = 3; j--;){
				for(int k = 3; k--;){
					if(j == k){
						filter->matR.col[j][k] = stdDevs[i].v[j];
					}
				}
			}

			// set the proc-noise covariance matrix to an arbitrary value
			switch(i){
			case 0: // acc
			kfMatIdent(filter->matR);
			kfMatScl(filter->matR, filter->matR, 100);
			kfMatScl(filter->matQ, filter->matQ, 0.0001);
				break;
			case 1: // gyro
			kfMatScl(filter->matQ, filter->matQ, 0.01);
				break;
			case 2: // mag
			kfMatScl(filter->matQ, filter->matQ, 0.01);
				break;
			}

			// if(!i){
			// 	kfMatScl(filter->matQ, filter->matQ, 0.001);
			// 	kfMatScl(filter->matR, filter->matR, 10);
			// }
			// else{
			// 	kfMatScl(filter->matQ, filter->matQ, 0.001);
			// 	kfMatScl(filter->matR, filter->matR, 0.1);
			// }

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
	sensorStatef_t cal = applyCalibration(&state->raw, state->calMinMax);

	if(isFinished) return 1;
	if(READINGS_COLLECTED < WARMUP_SAMPLES){
		// vec3Add(state->means.acc, state->mean.acc, state->raw.acc);
		for(int i = 3; i--;){
			ACCEL_MEAN[i] += cal.acc.v[i];
			GYRO_MEAN[i]  += cal.gyro.v[i];
			MAG_MEAN[i]   += cal.mag.v[i];
		}
		READINGS[READINGS_COLLECTED] = cal;
		++READINGS_COLLECTED;
		return 0;
	}

	// compute the mean when still
	for(int i = 3; i--;){
		ACCEL_MEAN[i] /= WARMUP_SAMPLES;
		GYRO_MEAN[i]  /= WARMUP_SAMPLES;
		MAG_MEAN[i]   /= WARMUP_SAMPLES;
	}

	// compute the variance for standard deviation
	bzero(&state->standardDeviations, sizeof(sensorStatei_t));
	double varLin[3] = {}, varRot[3] = {}, varMag[3] = {};
	for(int i = WARMUP_SAMPLES; i--;){
		for(int j = 3; j--;){
			double av = READINGS[i].acc.v[j]  - ACCEL_MEAN[j];
			double gv = READINGS[i].gyro.v[j] - GYRO_MEAN[j];
			double mv = READINGS[i].mag.v[j]  - MAG_MEAN[j];
			varLin[j] += (av * av);
			varRot[j] += (gv * gv);
			varMag[j] += (mv * mv);
		}
	}

	// set standard deviations
	for(int i = 3; i--;){
		varLin[i] /= WARMUP_SAMPLES;
		varRot[i] /= WARMUP_SAMPLES;
		varMag[i] /= WARMUP_SAMPLES;		

		state->standardDeviations.acc.v[i]  = varLin[i] = sqrt(varLin[i]);
		state->standardDeviations.gyro.v[i] = varRot[i] = sqrt(varRot[i]);
		state->standardDeviations.mag.v[i]  = varMag[i] = sqrt(varMag[i]);
	}

	printf("acc u: %f, %f, %f\n", ACCEL_MEAN[0], ACCEL_MEAN[1], ACCEL_MEAN[2]);
	printf("acc stddev: %f, %f, %f\n", varLin[0], varLin[1], varLin[2]);
	printf("gry stddev: %f, %f, %f\n", varRot[0], varRot[1], varRot[2]);
	printf("mag stddev: %f, %f, %f\n", varMag[0], varMag[1], varMag[2]);

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
//------------------------------------------------------------------------------
int imuUpdateState(int fd, imuState_t* imu, int contCal)
{
	sensorStatei_t* raw = &imu->raw;
	sensorStatei_t reading = imuGetReadings(fd);

	*raw = reading;

	if(imu->isCalibrated){
		// update the mag window
		if(contCal){
			contMagCal(raw, imu->calMinMax);
		}

		imu->cal = applyCalibration(raw, imu->calMinMax);

		// subtract the gyro bias
		for(int i = 3; i--;){
			imu->cal.gyro.v[i] = raw->gyro.v[i] - GYRO_MEAN[i];
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
int contMagCal(sensorStatei_t* raw, sensorStatei_t calMinMax[2])
{
	vec3i16_t* magMin = &calMinMax[0].mag;
	vec3i16_t* magMax = &calMinMax[1].mag;
	int updatedMagWindow = 0;

	// check each axis of the min and max windows, update them accordingly
	for(int i = 3; i--;){
		if(raw->mag.v[i] > magMax->v[i]){
			magMax->v[i] = raw->mag.v[i];
			updatedMagWindow = 1;
		}

		if(raw->mag.v[i] < magMin->v[i]){
			magMin->v[i] = raw->mag.v[i];
			updatedMagWindow = 1;
		}
	}

	// write new mag min and max if applicable
	if(updatedMagWindow){
		int calFd = open("./imu.cal", O_WRONLY);

		if(calFd < 0){
			return -1;
		}

		write(calFd, &calMinMax, sizeof(sensorStatei_t) * 2);
		close(calFd);
	}

	return 0;
}
//------------------------------------------------------------------------------
sensorStatef_t applyCalibration(sensorStatei_t* raw, sensorStatei_t calMinMax[2])
{
	vec3i16_t* accMin = &calMinMax[0].acc;
	vec3i16_t* accMax = &calMinMax[1].acc;
	vec3i16_t* magMin = &calMinMax[0].mag;
	vec3i16_t* magMax = &calMinMax[1].mag;

	sensorStatef_t cal = {};

	// scale the accelerometer readings into the calibrated range
	for(int i = 3; i--;){
		cal.acc.v[i] = LIL_G * map(raw->acc.v[i], accMin->v[i], accMax->v[i]);
	}

	// just copy the gyro TODO: scale the rate
	for(int i = 3; i--;){
		cal.gyro.v[i] = raw->gyro.v[i];
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
		vec3Sub(cal.mag, raw->mag, magOff);
		vec3Div(cal.mag, cal.mag, magRad);
	}

	return cal;
}
//-----------------------------------------------------------------------------
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
