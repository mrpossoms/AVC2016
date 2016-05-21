#include <stdio.h>
#include <unistd.h>
#include "sensors/imu.h"

int hasOpt(char* argv[], int argc, const char* target){
        for(int i = argc; i--;){
                if(!strcmp(argv[i], target)) return 1;
        }

        return 0;
}

int main(int argc, char* argv[])
{
	if(argc < 3){
		printf("Please pass the path to the i2c device. And a path to a calibration file.\n");
		return 1;
	}

	int imu_fd = open(argv[1], O_RDWR);
	int cal_fd = open(argv[2], O_RDWR | O_CREAT);
	imuState_t state = {};
	imuLoadCalibrationProfile(cal_fd, &state);

	lseek(cal_fd, SEEK_SET, 0);

	if(hasOpt(argv, argc, "-a")){
		imuPerformCalibration(imu_fd, &state);
	}

	if(hasOpt(argv, argc, "-g")){
		imuPerformGyroCalibration(imu_fd, &state, 5.6445f);
	}

	write(cal_fd, state.calMinMax, sizeof(state.calMinMax));	

	close(cal_fd);
	close(imu_fd);

	return 0;
}
