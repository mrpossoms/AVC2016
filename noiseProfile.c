#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include "imu.h"

#define SAMPLES 1000

int main()
{
	int fd = open("/dev/i2c-1", O_RDWR);
	readings_t samples[SAMPLES] = {};
	
	// collect 1000 samples
	for(int i = SAMPLES; i--;){
		samples[i] = imuGetReadings(fd);
		usleep(10000);
	}

	// compute the means
	double mean[3] = {};
	for(int i = SAMPLES; i--;){
		for(int j = 3; j--;)
			mean[j] += samples[i].accLinear.v[j] / (float)SAMPLES;
	}

	double variance[3] = {};
	for(int i = SAMPLES; i--;){
		for(int j = 3; j--;){
			double v = samples[i].accLinear.v[j] - mean[j];
			variance[j] += (v * v) / (float)SAMPLES;
		}
	}

	double dev[3] = {
		sqrt(variance[0]),
		sqrt(variance[1]),
		sqrt(variance[2]),
	};

	printf("Mean: (%f, %f, %f)\n", mean[0], mean[1], mean[2]);
	printf("Variance: (%f, %f, %f)\n", variance[0], variance[1], variance[2]);
	printf("Std dev: (%f, %f, %f)\n", dev[0], dev[1], dev[2]);

	return 0;
}
