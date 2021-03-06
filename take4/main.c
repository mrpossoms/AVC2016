#ifdef __cplusplus
#warning !!!It worked!!!
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#else
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif

#include <stdio.h>
#include <unistd.h>
#include <indicurses.h>
#include <pthread.h>
#include "imu.h"

int IMU_FD;
pthread_t  IMU_THREAD;
sensorStatei_t IMU_READING;

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
	CvCapture* cap = NULL;
	IplImage *frame, *frameGrey, *greyProc[2];
	IplImage *tmp, *eig;
	IplImage *pryProc[2];
	CvPoint2D32f corners[2][MAX_CORNERS];
	char  statusVector[MAX_CORNERS];
	float errorVector[MAX_CORNERS];
	int dblBuff = 0;
	int isReady = 0;

	cap = cvCreateCameraCapture(0);	
	assert(cap);

	// set capture size
	cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH,  640);
	cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, 480);

	IMU_FD = open("/dev/i2c-1", O_RDWR);
	
	// icInit();
	// pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);

	cvNamedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	int cornerCount = MAX_CORNERS;

	while(1){
		IplImage* currFrame = cvQueryFrame(cap);
		if(!currFrame) continue;
		if(!frame){
			frame = cvCloneImage(cvQueryFrame(cap));
			CvSize frameSize = cvSize(frame->width, frame->height);
			frameGrey    = cvCreateImage(frameSize, 8, 1);
			greyProc[0]  = cvCreateImage(frameSize, 8, 1);
			greyProc[1]  = cvCloneImage(greyProc[0]);
			pryProc[0]   = cvCloneImage(greyProc[0]);
			pryProc[1]   = cvCloneImage(greyProc[0]);

			tmp = cvCreateImage(frameSize, IPL_DEPTH_32F, 1);
			eig = cvCreateImage(frameSize, IPL_DEPTH_32F, 1);
		}
		
		cvCopy(currFrame, frame, 0);
		
		// convert the frame to black and white
		cvCvtColor(frame, greyProc[dblBuff], CV_BGR2GRAY);
		// cvPyrDown(frameGrey, greyProc[dblBuff], CV_GAUSSIAN_5x5);

		if(isReady)
		cvCalcOpticalFlowPyrLK(
			greyProc[!dblBuff],
			greyProc[dblBuff],
			pryProc[!dblBuff],
			pryProc[dblBuff],
			corners[!dblBuff],
			corners[dblBuff],
			cornerCount,
			cvSize(3, 3),
			5,                 // pyr level (0 not used)
			statusVector,      // list of bools which inicate feature matches
			errorVector,
			cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.01),
			1
		);

		for(int i = cornerCount; i--;){
			cvCircle(
				frame,
				cvPoint(corners[dblBuff][i].x, corners[dblBuff][i].y),
				3,
				cvScalar(255, 0, 0, 255),
				1,
				8,
				0
			);
			
			if(!statusVector[i]) continue;

			float dx = (corners[dblBuff][i].x - corners[!dblBuff][i].x) * 10;
			float dy = (corners[dblBuff][i].y - corners[!dblBuff][i].y) * 10;
			float depth = dx * dx + dy * dy;

			depth = pow(depth, 64);

			cvLine(
				frame,
				cvPoint(corners[dblBuff][i].x, corners[dblBuff][i].y),
				cvPoint((corners[dblBuff][i].x + dx) , (corners[dblBuff][i].y + dy)),
				cvScalar(depth, dy + 128, dx + 128, 255 / (errorVector[i] + 1)),
				1, 
				8,
				0
			);
		}

		cvShowImage("AVC", frame);

		// detect corners
		cornerCount = 400;
		cvGoodFeaturesToTrack(
			greyProc[dblBuff],
			NULL,
			NULL,
			corners[dblBuff],
			&cornerCount,
			0.01,        // quality
			0.01,        // min distance
			NULL,        // mask for ROI
			5,           // block size
			0,           // use harris detector
			0.04         // not used (free param of harris)
		);

		dblBuff = !dblBuff;
		isReady = 1;
	}

	cvReleaseImage(&frame);
	cvReleaseCapture(&cap);
	return 0;
}
