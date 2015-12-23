#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

#include "controls/servos.h"
#include "imu.h"

// File descriptors
int FD_IMU;

int main(int argc, char* argv[])
{
	// start servo controlling
	int res = conInit();
	if(res) return res;

	// open I2C bus	
	FD_IMU = open("/dev/i2c-1", O_RDWR);	
	if(FD_IMU <= 0){
		fprintf(stderr, "Error, failed to open I2C bus\n");
	}

	conSet(0, 50);	

	return 0;
}

