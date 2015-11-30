#include <stdio.h>
#include <unistd.h>   // UNIX standard function definitions 
#include <fcntl.h>    // File control definitions 
#include <termios.h>  // POSIX terminal control definitions 
#include <string.h>   // String function definitions 
#include <inttypes.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <arpa/inet.h>
#include <time.h>

typedef union{
	int16_t v[3];
	struct{
		int16_t x, y, z;
	};
} vec3_t;

typedef struct{
	vec3_t accLinear;
	vec3_t accRotational;
	vec3_t mag;
} readings_t;

typedef struct{
	float accLinear[3];
	float accRotational[3];
	float mag[3];
	time_t begin;	
} average_t;

void endianSwapVec3(vec3_t* v)
{
	v->x = ntohs(v->x);
	v->y = ntohs(v->y);
	v->z = ntohs(v->z);
}

int configSerial(int fd, int baud)
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

void synch(int fd)
{
	int synched = 0;
	char buf[5] = {};

	write(1, "Synching...", 11);
	while(!synched){
		// size_t bytes;
		// while((bytes = read(fd, buf, 5)) > 0)
		// {
		// 	printf("bytes %zu\n", bytes);
		// }

		write(fd, "b", 1);
		usleep(100000);
		write(fd, "s", 1);
		usleep(100000);

		int matchingLetters = 0;
		for(int tries = 128; tries--;){
			char c = 'x';
			read(fd, &c, 1);
				
			if("SYNCH"[matchingLetters] == c){
				++matchingLetters;
				write(1, &c, 1);
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

	printf("done!\n");
}

int main(int argc, char* argv[])
{
	if(argc < 2){
		return 1;
	}

	int fd = open(argv[1], O_RDWR);
	int sampFd = open("./samples.bin", O_WRONLY | O_CREAT);

	configSerial(fd, 9600);
	synch(fd);

	// write(fd, "s", 1);

	average_t averageSamples = {
		.begin = time(NULL),
	};
	int samples = 0;

	while(1){
		readings_t reading = {};
		size_t bytes = read(fd, &reading, sizeof(readings_t));

		// write(1, (void*)&reading, sizeof(reading_t));

		endianSwapVec3(&reading.accLinear);
		endianSwapVec3(&reading.accRotational);
		endianSwapVec3(&reading.mag);

		assert(bytes == sizeof(readings_t));

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
		if(samples >= 1000){
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
