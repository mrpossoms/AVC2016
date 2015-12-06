#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <unistd.h>
#include <indicurses.h>
#include <pthread.h>
#include "imu.h"

int IMU_FD;
pthread_t  IMU_THREAD;
readings_t IMU_READING;

void* imuHandler(void* param)
{
	const int samples = 50;
	int i = 0;

	int readings[3][50] = {{},{},{}};
	int origin[100];

	for(int i = 100; i--; origin[i] = 0)
		bzero(readings[0], sizeof(int) * samples);

	while(1){
		IMU_READING = imuGetReadings(IMU_FD);
		int topLeft[2] = { 5, IC_TERM_HEIGHT * 0.75 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};	
		
		int center = (topLeft[1] - bottomRight[1]) / 2;

		const int scale = 500;

		readings[0][i] = center + IMU_READING.accLinear.x / scale;
		readings[1][i] = center + IMU_READING.accLinear.y / scale;
		readings[2][i] = center + IMU_READING.accLinear.z / scale;

		++i;
		i %= samples;

		clear();
		icLineGraph(topLeft, bottomRight, '-', origin, 100);
		icLineGraph(topLeft, bottomRight, 'x', readings[0], samples);
		icLineGraph(topLeft, bottomRight, 'y', readings[1], samples);
		icLineGraph(topLeft, bottomRight, 'z', readings[2], samples);

		icPresent();
		usleep(100);
	}

	return NULL;
}


int main()
{
	CvCapture* cap = NULL;
	IplImage *frame;

	cap = cvCreateCameraCapture(0);	
	assert(cap);

	// set capture size
	cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH,  320);
	cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, 240);

	IMU_FD = open("/dev/i2c-1", O_RDWR);
	
	icInit();
	pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);

	while(1){
		frame = cvQueryFrame(cap);
		
		if(!frame) continue;

		usleep(100);		
	}

	cvReleaseCapture(&cap);
	return 0;
}
