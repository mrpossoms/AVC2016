#include "imu.h"

typedef struct{
	float accLinear[3];
	float accRotational[3];
	float mag[3];
	time_t begin;	
} average_t;

int main(int argc, char* argv[])
{
	if(argc < 2){
		return 1;
	}

	int fd = open(argv[1], O_RDWR);
	int sampFd = open("./samples.bin", O_WRONLY | O_CREAT);

	imuConfigSerial(fd, 9600);
	imuSynch(fd);

	// write(fd, "s", 1);

	average_t averageSamples = {
		.begin = time(NULL),
	};
	int samples = 0;

	while(1){
		readings_t reading = imuGetReadings(fd);

		// record sample
		averageSamples.accLinear[0] += reading.accLinear.x / 1000.0;
		averageSamples.accLinear[1] += reading.accLinear.y / 1000.0;
		averageSamples.accLinear[2] += reading.accLinear.z / 1000.0;

		averageSamples.accRotational[0] += reading.accRotational.x / 1000.0;
		averageSamples.accRotational[1] += reading.accRotational.y / 1000.0;
		averageSamples.accRotational[2] += reading.accRotational.z / 1000.0;

		averageSamples.mag[0] += reading.mag.x / 1000.0;
		averageSamples.mag[1] += reading.mag.y / 1000.0;
		averageSamples.mag[2] += reading.mag.z / 1000.0;

		// have enough samples, store and start over
		if(samples >= 100){
			// save this average
			write(sampFd, &averageSamples, sizeof(average_t));

			printf("acc = (%f, %f, %f) ", averageSamples.accLinear[0], averageSamples.accLinear[1], averageSamples.accLinear[2]);
			printf("gyro = (%f, %f, %f) ", averageSamples.accRotational[0], averageSamples.accRotational[1], averageSamples.accRotational[2]);
			printf("mag = (%f, %f, %f)\n", averageSamples.mag[0], averageSamples.mag[1], averageSamples.mag[2]);

			samples = 0;
			// reset all averages
			bzero(&averageSamples, sizeof(average_t));
			// record the new start time
			averageSamples.begin = time(NULL);
		}
		else{
			++samples;
		}
	}

	return 0;
}
