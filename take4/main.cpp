#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <unistd.h>
#include <indicurses.h>
#include <pthread.h>
#include "imu.h"

int IMU_FD;
pthread_t  IMU_THREAD;
sensorStatei_t IMU_READING;

using namespace cv;
using namespace std;

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
		int topLeft[2] = { 5, 2 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};	
		
		int center = (bottomRight[1] - topLeft[1]) / 2;

		const int scale = 500;

		readings[0][i] = center + IMU_READING.linear.x / scale;
		readings[1][i] = center + IMU_READING.linear.y / scale;
		readings[2][i] = center + IMU_READING.linear.z / scale;

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
	const int MAX_CORNERS = 400;
	Mat frame, frameGrey, greyProc[2];
	vector<Point2f> corners[2];
	vector<unsigned char> statusVector;
	vector<float> errorVector;
	int dblBuff = 0;
	int isReady = 0;

	VideoCapture cap(0);	
	assert(cap.isOpened());

	// set capture size
	cap.set(CV_CAP_PROP_FRAME_WIDTH,  640);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	IMU_FD = open("/dev/i2c-1", O_RDWR);
	
	// icInit();
	// pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);

	namedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	int cornerCount = MAX_CORNERS;

	while(1){
		Mat currFrame;
		cap >> currFrame;
		if(currFrame.empty()) continue;
		if(!isReady){
			frame = currFrame.clone();
			cvtColor(frame, frameGrey, CV_BGR2GRAY);
			greyProc[0] = frameGrey.clone();
			greyProc[1] = frameGrey.clone();
		}
		
		currFrame.copyTo(frame);
		
		// convert the frame to black and white
		cvtColor(frame, greyProc[dblBuff], CV_BGR2GRAY);
		
		if(isReady)
		calcOpticalFlowPyrLK(
			greyProc[!dblBuff],
			greyProc[dblBuff],
			corners[!dblBuff],
			corners[dblBuff],
			statusVector,      // list of bools which inicate feature matches
			errorVector
		);

		for(int i = corners[dblBuff].size(); i--;){
			circle(
				frame,
				Point(corners[dblBuff][i].x, corners[dblBuff][i].y),
				3,
				Scalar(255, 0, 0, 255),
				1,
				8,
				0
			);
			
			if(!statusVector[i]) continue;

			float dx = (corners[dblBuff][i].x - corners[!dblBuff][i].x) * 10;
			float dy = (corners[dblBuff][i].y - corners[!dblBuff][i].y) * 10;
			float depth = dx * dx + dy * dy;

			depth = pow(depth, 64);

			line(
				frame,
				Point(corners[dblBuff][i].x, corners[dblBuff][i].y),
				Point((corners[dblBuff][i].x + dx) , (corners[dblBuff][i].y + dy)),
				Scalar(depth, dy + 128, dx + 128, 255 / (errorVector[i] + 1)),
				1, 
				8,
				0
			);
		}

		imshow("AVC", frame);

		// detect corners
		cornerCount = 400;
		goodFeaturesToTrack(
			greyProc[dblBuff],
			corners[dblBuff],
			MAX_CORNERS,
			0.01,        // quality
			0.01,        // min distance
			Mat(),        // mask for ROI
			3,           // block size
			0,           // use harris detector
			0.04         // not used (free param of harris)
		);

		dblBuff = !dblBuff;
		isReady = 1;
	}

	return 0;
}
