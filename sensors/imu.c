#include "imu.h"
#include <stdlib.h>
#include <assert.h>
#include <math.h>

#define ADDR_ACC_MAG 0x1D
#define ADDR_GYRO    0x6B

#define ACC_REG 0x28
#define MAG_REG 0x08
#define GYR_REG 0x28

int contMagCal(sensorStatei_t* raw, sensorStatei_t* calMinMax);

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

//    __  __      _
//   |  \/  |__ _(_)_ _
//   | |\/| / _` | | ' \
//   |_|  |_\__,_|_|_||_|
//

int imuUpdateState(int fd, imuState_t* imu, int contCal)
{
	sensorStatei_t* raw = &imu->raw;
	sensorStatei_t reading = imuGetReadings(fd);
	*raw = reading;

	assert(imu->isCalibrated);

	// update the mag window
	if(contCal){
		contMagCal(raw, imu->calMinMax);
	}

	sensorStatef_t cal = imuApplyCalibration(raw, imu->calMinMax);
	cal.enc_dist = imu->cal.enc_dist;
	imu->cal = cal;

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
sensorStatef_t imuApplyCalibration(sensorStatei_t* raw, sensorStatei_t calMinMax[2])
{
	vec3i16_t* accMin = &calMinMax[0].acc;
	vec3i16_t* accMax = &calMinMax[1].acc;
	vec3i16_t* magMin = &calMinMax[0].mag;
	vec3i16_t* magMax = &calMinMax[1].mag;
	vec3i16_t* gyrMin = &calMinMax[0].gyro;
	vec3i16_t* gyrMax = &calMinMax[1].gyro;

	sensorStatef_t cal = {};

	// scale the accelerometer readings into the calibrated range
	for(int i = 3; i--;){
		cal.acc.v[i] = LIL_G * map(raw->acc.v[i], accMin->v[i], accMax->v[i]);
	}

	// just copy the gyro TODO: scale the rate
	for(int i = 3; i--;){
		cal.gyro.v[i] = map(raw->gyro.v[i], gyrMin->v[i], gyrMax->v[i]);
	}
	cal.gyro.x = cal.gyro.y = 0;

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
		case 'w':
		case 'W':
			return readings.gyro.z;
	}

	return 0;
}
//-----------------------------------------------------------------------------
int imuPerformCalibration(int fd_imu, imuState_t* state)
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

	return 0;
}
//-----------------------------------------------------------------------------
int imuPerformGyroCalibration(int fd_imu, imuState_t* state, float rot_ps)
{
	printf("Spin at a constant known speed in one direction, then the other\n");
	printf("[Press any key to begin]\n");
	getchar();

	float rad_sec = 2 * M_PI / rot_ps;

	state->calMinMax[0].gyro.z = axisAcc('w', 0, fd_imu) / rad_sec;	
	state->calMinMax[1].gyro.z = axisAcc('w', 1, fd_imu) / rad_sec;

	return 0;
}
//-----------------------------------------------------------------------------
int imuLoadCalibrationProfile(int fd_storage, imuState_t* state)
{
	int isOk = read(fd_storage, state->calMinMax, sizeof(state->calMinMax)) == sizeof(state->calMinMax);
	if(isOk){
		printf("\nmin: "); log_senI(state->calMinMax + 0); printf("\n");
		printf("max: "); log_senI(state->calMinMax + 1); printf("\n");

		state->isCalibrated = 1;
		return 0;
	}

	return -1;
}
