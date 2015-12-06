#include "imu.h"
#include <fcntl.h>    // File control definitions 
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <assert.h>
#include <arpa/inet.h>

#define ADDR_ACC_MAG 0x1D
#define ADDR_GYRO    0x6B

#define ACC_REG 0x28
#define MAG_REG 0x08
#define GYR_REG 0x28

#define G 9.8

static int32_t ACCEL_OFFSET[3];
static int READINGS_COLLECTED;


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
	uint8_t buf[] = { dstReg, byte };

	ioctl(fd, I2C_SLAVE, devAddr);
	write(fd, buf, 2);

	return 0;
}

static int requestBytes(int fd, uint8_t devAddr, uint8_t srcReg, void* dstBuf, size_t bytes)
{
	uint8_t commByte;
	ioctl(fd, I2C_SLAVE, devAddr);
	commByte = 0x80 | srcReg;
	write(fd, &commByte, 1);

	if(read(fd, dstBuf, bytes) != bytes){
		return -1;
	}

	return 0;
}

//    ___       _          ___     _ _ _           
//   |   \ __ _| |_ __ _  | _ \___| | (_)_ _  __ _ 
//   | |) / _` |  _/ _` | |  _/ _ \ | | | ' \/ _` |
//   |___/\__,_|\__\__,_| |_| \___/_|_|_|_||_\__, |
//                                           |___/ 
readings_t imuGetReadings(int fd)
{
	static int isSetup;
	readings_t reading = {};
	int res = 0;
	
	if(!isSetup){
		sendByte(fd, ADDR_ACC_MAG, 0x20, 0x67);
		sendByte(fd, ADDR_ACC_MAG, 0x21, 0x00);
		sendByte(fd, ADDR_ACC_MAG, 0x26, 0x00);
		sendByte(fd, ADDR_GYRO, 0x20, 0x0F);		

		isSetup = 1;
		usleep(100000);
	}

	res += requestBytes(fd, ADDR_ACC_MAG, ACC_REG, &reading.accLinear, sizeof(vec3i16_t));
	res += requestBytes(fd, ADDR_ACC_MAG, GYR_REG, &reading.accRotational, sizeof(vec3i16_t));
	res += requestBytes(fd, ADDR_ACC_MAG, MAG_REG, &reading.mag, sizeof(vec3i16_t));

	assert(res == 0);

	endianSwapVec3(&reading.accLinear);
	endianSwapVec3(&reading.accRotational);
	endianSwapVec3(&reading.mag);

	// const int samples = 100;
	// if(READINGS_COLLECTED++ < samples){
	// 	ACCEL_OFFSET[0] += reading.accLinear.x;
	// 	ACCEL_OFFSET[1] += reading.accLinear.y;
	// 	ACCEL_OFFSET[2] += reading.accLinear.z;
	// }
	// else{
	// 	reading.accLinear.x -= ACCEL_OFFSET[0];
	// 	reading.accLinear.y -= ACCEL_OFFSET[1];
	// 	reading.accLinear.z -= ACCEL_OFFSET[2];
	// }
	// if(READINGS_COLLECTED == samples){
	// 	ACCEL_OFFSET[0] /= samples;
	// 	ACCEL_OFFSET[1] /= samples;
	// 	ACCEL_OFFSET[2] /= samples;
	// }

	return reading;
}

int16_t axisAcc(char axis, int isMax, int fd_imu)
{
	printf(isMax ? "(+%c) [Press any key]\n" : "(-%c) [Press any key]\n", axis);

	getchar();
	readings_t readings = imuGetReadings(fd_imu);

	switch(axis){
		case 'X':
		case 'x':
			return readings.accLinear.x;
		case 'Y':
		case 'y':
			return readings.accLinear.y;
		case 'Z':
		case 'z':
			return readings.accLinear.z;
	}

	return 0;
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

void imuUpdateState(int fd, imuState_t* state)
{
	readings_t readings = state->lastReadings = imuGetReadings(fd);

	// if we don't have a start time yet, don't bother to compute the velocities
	if(state->lastTime.tv_usec){
		float dt = elapsedSeconds(state);
		vec3f_t acc;

		if(state->isCalibrated){
			vec3i16_t* accMin = &state->calibrationMinMax[0].accLinear;
			vec3i16_t* accMax = &state->calibrationMinMax[1].accLinear;

			// map the readings to the 1G calibration window that was obtained
			// from the calibration profile
			acc.x = G * readings.accLinear.x * 2 / (float)(accMax->x - accMin->x);
			acc.y = G * readings.accLinear.y * 2 / (float)(accMax->y - accMin->y);
			acc.z = G * readings.accLinear.z * 2 / (float)(accMax->z - accMin->z);
		}
		else{
			// no calibration, just spit out the literal value
			acc.x = readings.accLinear.x;
			acc.y = readings.accLinear.y;
			acc.z = readings.accLinear.z;	
		}

		// integrate
		state->linearVel.x += acc.x * dt;
		state->linearVel.y += acc.y * dt;
		state->linearVel.z += acc.z * dt;
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
int imuPerformCalibration(int fd_storage, int fd_imu, imuState_t* state)
{
	printf("Point each axis toward the center of the earth when prompted.\n");
	printf("[Press any key to begin]\n");
	getchar();

	printf("Calibrating accelerometer\n");
	state->calibrationMinMax[0].accLinear.x = axisAcc('x', 0, fd_imu);
	state->calibrationMinMax[1].accLinear.x = axisAcc('x', 1, fd_imu);
	state->calibrationMinMax[0].accLinear.y = axisAcc('y', 0, fd_imu);
	state->calibrationMinMax[1].accLinear.y = axisAcc('y', 1, fd_imu);
	state->calibrationMinMax[0].accLinear.z = axisAcc('z', 0, fd_imu);
	state->calibrationMinMax[1].accLinear.z = axisAcc('z', 1, fd_imu); 

	// store the results
	write(fd_storage, &state->calibrationMinMax, sizeof(readings_t) * 2);

	return 0;
}

int imuLoadCalibrationProfile(int fd_storage, imuState_t* state)
{
	int isOk = read(fd_storage, &state->calibrationMinMax, sizeof(readings_t) * 2) == sizeof(readings_t) * 2;

	if(isOk){
		state->isCalibrated = 1;
	}

	return 1;
}

