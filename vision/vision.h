#pragma once

#ifdef __linux__
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#else

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#endif

#include "system.h"

using namespace cv;
using namespace std;

typedef struct{
	Point2f topLeft, bottomRight;
} boundingBox_t;

typedef struct{
	CvPoint         frameCenter;
	vector<Point2f> features[2];
	vector<Point2f> flow;
	float           featureDepths[2][MAX_FEATURES];
	int             dblBuff;
	vector<unsigned char> statusVector;
	vector<float>         errorVector;

	boundingBox_t   regions[256];
	int             maxRegion;
} trackingState_t;

int  visionInit(trackingState_t* tracking, int width, int height);
void visionUpdate(trackingState_t* tracking);

