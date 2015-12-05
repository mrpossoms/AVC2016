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
	imuSynch(IMU_FD);

	int readings[3][samples] = {{},{},{}};
	int origin[100];
	int center = (IC_TERM_HEIGHT / 2) - 4;

	for(int i = 100; i--; origin[i] = center)
		bzero(readings[0], sizeof(int) * samples);

	while(1){
		long var = variance(readings[0], i, samples);

		IMU_READING = imuGetReadings(IMU_FD);
		
		const int scale = 500;

		readings[0][i] = center + IMU_READING.accLinear.x / scale;
		readings[1][i] = center + IMU_READING.accLinear.y / scale;
		readings[2][i] = center + IMU_READING.accLinear.z / scale;

		++i;
		i %= samples;

		int topLeft[2] = { 25, 2 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};
		clear();
		icLineGraph(topLeft, bottomRight, '-', origin, 100);
		icLineGraph(topLeft, bottomRight, 'x', readings[0], samples);
		icLineGraph(topLeft, bottomRight, 'y', readings[1], samples);
		icLineGraph(topLeft, bottomRight, 'z', readings[2], samples);

		char count[32] = {};
		sprintf(count, "%d", var);
		icText(2, 2, count);

		icPresent();
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
	
	icInit();
	pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);

	while(1){
		frame = cvQueryFrame(cap);
		
		if(!frame) continue;

		
	}

	cvReleaseCapture(&cap);
	return 0;
}
