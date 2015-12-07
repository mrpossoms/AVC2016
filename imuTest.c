#include "imu.h"
#include <time.h>

typedef struct{
	float accLinear[3];
	float accRotational[3];
	float mag[3];
	time_t begin;	
} average_t;

imuState_t IMU_STATE;

int main(int argc, char* argv[])
{
	if(argc < 2){
		return 1;
	}

	int fd = open(argv[1], O_RDWR);

	if(argc <= 3){
		int cal_fd = open(argv[2], O_RDONLY);
		imuLoadCalibrationProfile(cal_fd, &IMU_STATE);


		print_v3i16(&IMU_STATE.calibrationMinMax[0].accLinear);
		printf(" - ");
		print_v3i16(&IMU_STATE.calibrationMinMax[1].accLinear);
		printf("\n");

		sleep(2);
	}

	// write(fd, "s", 1);
	int samples = 0;
	time_t start = time(NULL);

	while(1){
		imuUpdateState(fd, &IMU_STATE);

		// have enough samples, store and start over
		if(samples >= 100){
			printf("Vel = (%f, %f, %f)\n", IMU_STATE.linearVel.x, IMU_STATE.linearVel.y, IMU_STATE.linearVel.z);
			
			samples = 0;
		}
		else{
			++samples;
		}

		if(time(NULL) - start >= 1){
			printf("Vel = (%f, %f, %f)\n", IMU_STATE.linearVel.x, IMU_STATE.linearVel.y, IMU_STATE.linearVel.z);
			break;
		}
	}

	return 0;
}
