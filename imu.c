#include "imu.h"
#include <fcntl.h>    // File control definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <sys/ioctl.h>
#include <assert.h>
#include <arpa/inet.h>

static int32_t ACCEL_OFFSET[3];
static int READINGS_COLLECTED;

static void endianSwapVec3(vec3i16_t* v)
{
	v->x = ntohs(v->x);
	v->y = ntohs(v->y);
	v->z = ntohs(v->z);
}

int imuConfigSerial(int fd, int baud)
{
	struct termios opts = {};

	tcgetattr(fd, &opts);

	cfsetispeed(&opts, baud);
	cfsetospeed(&opts, baud);

	// 8N1
	opts.c_cflag = CS8;
	// no flow control

    // opts.c_cflag &= ~HUPCL; // disable hang-up-on-close to avoid reset
	// turn on READ & ignore ctrl lines
	opts.c_cflag = CREAD | CLOCAL;

	// turn off s/w flow ctrl
	opts.c_lflag = ICANON; // make raw
	opts.c_oflag = 0; // make raw
	opts.c_iflag = IGNPAR; // make raw

	cfmakeraw(&opts);

	// see: http://unixwiz.net/techtips/termios-vmin-vtime.html
	opts.c_cc[VMIN]  = 0;
	opts.c_cc[VTIME] = 0;

	tcsetattr(fd, TCSANOW, &opts);

	return 0;
}

void imuSynch(int fd)
{
	int synched = 0;
	char buf[5] = {};

	while(!synched){
		tcflush(fd, TCIOFLUSH);
		write(fd, "b", 1);
		write(fd, "s", 1);
		usleep(100000);


		int matchingLetters = 0;
		for(int tries = 128; tries--;){
			char c = 'x';
			read(fd, &c, 1);
				
			if("SYNCH"[matchingLetters] == c){
				++matchingLetters;
			}
			else{
				matchingLetters = 0;
			}

			if(matchingLetters == 5){
				synched = 1;				
				break;
			}
		}
	}

	struct termios opts = {};
	tcgetattr(fd, &opts);
	opts.c_cc[VMIN] = sizeof(readings_t);
	tcsetattr(fd, TCSANOW, &opts);
}

readings_t imuGetReadings(int fd)
{
	readings_t reading = {};
	size_t bytes = read(fd, &reading, sizeof(readings_t));

	// write(1, (void*)&reading, sizeof(reading_t));

	endianSwapVec3(&reading.accLinear);
	endianSwapVec3(&reading.accRotational);
	endianSwapVec3(&reading.mag);

	const int samples = 100;
	if(READINGS_COLLECTED++ < samples){
		ACCEL_OFFSET[0] += reading.accLinear.x;
		ACCEL_OFFSET[1] += reading.accLinear.y;
		ACCEL_OFFSET[2] += reading.accLinear.z;
	}
	else{
		reading.accLinear.x -= ACCEL_OFFSET[0];
		reading.accLinear.y -= ACCEL_OFFSET[1];
		reading.accLinear.z -= ACCEL_OFFSET[2];
	}
	if(READINGS_COLLECTED == samples){
		ACCEL_OFFSET[0] /= samples;
		ACCEL_OFFSET[1] /= samples;
		ACCEL_OFFSET[2] /= samples;
	}

	return reading;
}

int16_t axisAcc(char axis, int isMax, int fd_imu)
{
	printf(isMax ? "(+%c) [Press any key]\n" : "(-%c) [Press any key]\n", axis);

	getchar();
	readings = imuGetReadings(fd_imu);

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
	return !(read(fd_storage, &state->calibrationMinMax, sizeof(readings_t) * 2) == sizeof(readings_t) * 2);
}

static float elapsedSeconds(imuState_t* state)
{
	// compute the elapsed time in seconds
	struct timeval time;
	gettimeofday(&time, NULL);
	int64_t usElapsed = (time.tv_sec - state->lastTime.tv_sec) * 1000000 +
	                    (time.tv_usec - state->lastTime.tv_usec);
	return usElapsed / 1000000.0;	
}

void imuUpdateState(int fd, imuState_t* state)
{
	state->lastReadings = imuGetReadings(fd);

	// if we don't have a start time yet, don't bother to compute the velocities
	if(state->lastTime){
		float dt = elapsedSeconds(state);
	}

	state->lastTime = time;
}