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

#include <limits.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <indicurses.h>
#include <pthread.h>

#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "TrackingMat.h"
#include "system.h"
#include "stream.h"
#include "timer.h"
#include "comms/protocol.h"
#include "comms/messages.h"
#include "sensors/aggergate.h"

using namespace cv;
using namespace std;

//     ___             _            _
//    / __|___ _ _  __| |_ __ _ _ _| |_ ___
//   | (__/ _ \ ' \(_-<  _/ _` | ' \  _(_-<
//    \___\___/_||_/__/\__\__,_|_||_\__/__/
//
const float FOCAL_PLANE = 1;

//    _____
//   |_   _|  _ _ __  ___ ___
//     | || || | '_ \/ -_|_-<
//     |_| \_, | .__/\___/__/
//         |__/|_|
typedef struct{
	CvPoint frameCenter;
	struct {
		vector<Point2f> points[2];
		vector<Point2f> delta;
		vector<uint8_t> regionIndex;
	} features;
	float featureDepths[2][MAX_FEATURES];
	int   dblBuff;
	vector<unsigned char> statusVector;
	vector<float>         errorVector;
} trackingState_t;

//     ___ _     _          _
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//
int          FRAME_NUMBER;
int          MAX_REGION;
TrackingMat* TRACKING_SPACE;

static txState_t        TRANSMIT_STATE;
static depthWindow_t    DEPTH_WINDOW;
static trackingState_t  ts;
static int              MY_SOCK;
static struct sockaddr_in* PEER;
static int NO_VIDEO_FRAMES;

int WIDTH, HEIGHT;

//    ___           _   _      ___    _   _            _   _
//   |   \ ___ _ __| |_| |_   | __|__| |_(_)_ __  __ _| |_(_)___ _ _
//   | |) / -_) '_ \  _| ' \  | _|(_-<  _| | '  \/ _` |  _| / _ \ ' \
//   |___/\___| .__/\__|_||_| |___/__/\__|_|_|_|_\__,_|\__|_\___/_||_|
//            |_|
void computeDepths(TrackingMat* tracking)
{

}

// static uint8_t regionIndexForDelta(trackingState_t* ts, int i)
// {
// 	float s = sqrtf(ts->features.delta[i].x * ts->features.delta[i].x + ts->features.delta[i].y * ts->features.delta[i].y);
// 	if(ts->errorVector[i] > 1) s /= ts->errorVector[i];
// 	return (s / 1000) * TRK_REGIONS;
// }

static Point2f featureIndexes(int i)
{
	float side = sqrtf(MAX_FEATURES);
	return Point2f((i % (int)side) * WIDTH / side, (i / side) * HEIGHT / side);
}

Scalar regionColor(unsigned char theta)
{
	unsigned char rgb[3] = {};
	unsigned char hsv[3] = {
		theta,
		255,
		255
	};
	unsigned char region, remainder, p, q, t;

	region = hsv[0] / 43;
	remainder = (hsv[0] - (region * 43)) * 6;

	p = 0;
	q = (hsv[2] * (255 - ((hsv[1] * remainder) >> 8))) >> 8;
	t = (hsv[2] * (255 - ((hsv[1] * (255 - remainder)) >> 8))) >> 8;

	switch (region)
	{
		 case 0:
			  rgb[0] = hsv[2]; rgb[1] = t; rgb[2] = p;
			  break;
		 case 1:
			  rgb[0] = q; rgb[1] = hsv[2]; rgb[2] = p;
			  break;
		 case 2:
			  rgb[0] = p; rgb[1] = hsv[2]; rgb[2] = t;
			  break;
		 case 3:
			  rgb[0] = p; rgb[1] = q; rgb[2] = hsv[2];
			  break;
		 case 4:
			  rgb[0] = t; rgb[1] = p; rgb[2] = hsv[2];
			  break;
		 default:
			  rgb[0] = hsv[2]; rgb[1] = p; rgb[2] = q;
			  break;
	}

	return Scalar(rgb[0], rgb[1], rgb[2]);
}

static int computeRegionMeans(Mat* frame, trackingState_t* ts)
{
	int wait = 0;

	for(int i = ts->features.points[ts->dblBuff].size(); i--;){
		if(!ts->statusVector[i]) continue;
		int x = i % TRACKING_SPACE->dimensions.width;
		int y = i / TRACKING_SPACE->dimensions.height;
		trkMatFeature_t feature = (*TRACKING_SPACE)[x][y];
		Point2f delta = feature.delta;
		//
		// if(r > MAX_REGION) MAX_REGION = r;

		// float s = sqrtf(ts->features.delta[i].x * ts->features.delta[i].x + ts->features.delta[i].y * ts->features.delta[i].y);
		// if(ts->errorVector[i] > 1) s /= ts->errorVector[i];
		float s = 4;

		if(feature.deltaMag >= TRK_THRESHOLD)
		line(
			*frame,
			feature.position,
			feature.position + delta,
			regionColor(feature.deltaMag * 10),
			1
		);

		if(feature.histBucket >= 0){
			line(
				*frame,
				feature.position,
				feature.position + Point2f(2, 0),
				regionColor(feature.histBucket << 4),
				2
			);
		}

		// int r = feature.region;
		// if(r > -1){// && TRACKING_SPACE->regions[r].flags == TRK_REGION_ACTIVE){
		// 	line(
		// 		*frame,
		// 		feature.position,
		// 		feature.position + Point2f(2, 0),
		// 		regionColor(r << 4),
		// 		10
		// 	);
		// 	wait++;
		// }

		// ts->features.regionIndex[i] = r;
		// REGIONS[r].mean += pos;
		// REGIONS[r].features++;
	}

	// for(int r = 1; r <= MAX_REGION; ++r){
	// 	REGIONS[r].mean /= REGIONS[r].features;
	// }

	for(int i = TRACKING_SPACE->regionCount; i--;){
		// Point2f p = TRACKING_SPACE->regions[i].centroid;
		// p.x /= TRACKING_SPACE->dimensions.width;
		// p.y /= TRACKING_SPACE->dimensions.height;
		// p.x *= WIDTH;
		// p.y *= HEIGHT;
		// rectangle(
		// 	*frame,
		// 	p - Point2f(2, 2),
		// 	p + Point2f(2, 2),
		// 	// Scalar(0, ts->delta[i].y * 16 + 128, ts->delta[i].x * 16 + 128),
		// 	regionColor(i << 4),
		// 	1,
		// 	8
		// );

		// if(TRACKING_SPACE->regions[i].flags == TRK_REGION_ACTIVE
		{
			rectangle(
				*frame,
				TRACKING_SPACE->regions[i].min,
				TRACKING_SPACE->regions[i].max,
				regionColor(i << 4),
				1,
				8
			);
		}
	}

	return wait;
}

static void computeRegionVariances(Mat* frame, trackingState_t* ts)
{
//
}

int main(int argc, char* argv[])
{
	Mat frame, frameGrey, greyProc[2];

	char* hostname = NULL;
	int centerX = 320 / 2, centerY = 240 / 2;
	int isReady = 0, hasVideoFeed = 0;

	// VideoCapture cap(0);
	VideoCapture cap("./SparkFun_AVC_2015.avi");
	// VideoCapture cap("./../test2.mp4");
	// VideoCapture cap("./../test.mov");
	hasVideoFeed = cap.isOpened();

	if(argc >= 3){
		for(int i = 1; i < argc; ++i){
			if(!strncmp(argv[i], "-w", 2)){
				centerX = atoi(argv[i] + 2) / 2;
			}
			if(!strncmp(argv[i], "-h", 2)){
				centerY = atoi(argv[i] + 2) / 2;
			}
			if(!strncmp(argv[i], "--no-video", 10)){
				NO_VIDEO_FRAMES = 1;
				printf("Not transmitting video\n");
			}
		}
	}

	WIDTH  = centerX * 2;
	HEIGHT = centerY * 2;

	if(hasVideoFeed){
		// set capture size
		cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
		cap.set(CV_CAP_PROP_FPS, 2);
		printf("Capture dimensions (%d, %d)\n", WIDTH, HEIGHT);
	}
	else{
		printf("No capture\n");
	}

	errno = 0;

	namedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	errno = 0;

	float side = sqrtf(MAX_FEATURES);
	TRACKING_SPACE = new TrackingMat(Size(side, side)); // TODO: replace the shit wit dis

	for(int i = MAX_FEATURES; i--;){
		float x = (i % (int)side) * WIDTH / side;
		float y = (i / side) * HEIGHT / side;
		ts.features.points[!ts.dblBuff].push_back(Point2f(x, y));
		ts.features.points[ts.dblBuff].push_back(Point2f(x, y));
		ts.features.delta.push_back(Point2f(0, 0));
		ts.features.regionIndex.push_back(0);
		ts.statusVector.push_back(0);
	}

	while(1){
		Mat currFrame;

		float side = sqrtf(MAX_FEATURES);
		for(int i = MAX_FEATURES; i--;){

			float x = (i % (int)side) * WIDTH / side;
			float y = (i / side) * HEIGHT / side;
			ts.features.points[!ts.dblBuff][i] = (Point2f(x, y));
			ts.features.points[ts.dblBuff][i] = (Point2f(x, y));
			ts.features.regionIndex[i] = 0;
		}

		cap >> currFrame;

		if(currFrame.empty()){
			printf("Is empty\n");
			continue;
		}

		resize(currFrame, currFrame, Size(WIDTH, HEIGHT), 0, 0, INTER_CUBIC);

		if(!isReady){
			frame = currFrame.clone();
			cvtColor(frame, frameGrey, CV_BGR2GRAY);
			greyProc[0] = frameGrey.clone();
			greyProc[1] = frameGrey.clone();
		}

		currFrame.copyTo(frame);
		cvtColor(frame, greyProc[ts.dblBuff], CV_BGR2GRAY);


		if(isReady){
			calcOpticalFlowPyrLK(
				greyProc[!ts.dblBuff],
				greyProc[ts.dblBuff],
				ts.features.points[!ts.dblBuff],
				ts.features.points[ts.dblBuff],
				ts.statusVector,      // list of bools which inicate feature matches
				ts.errorVector
			);

			TRACKING_SPACE->update(ts.features.points + ts.dblBuff);
			// usleep(1000 * 1000);//250);

			// computeDepths(&ts);
		}

		// MAX_REGION = 0;

		int wait = computeRegionMeans(&frame, &ts);

		// compute the varience
		computeRegionVariances(&frame, &ts);
		// computeRegionBBoxes(&ts);

		for(int r = 1; r <= MAX_REGION; ++r){
			Point2f o(10, r * 24);
			char buf[32] = {};

			// if(!REGIONS[r].features) continue;

			putText(
				frame,
				buf,
				o - Point2f(1, 1),
				FONT_HERSHEY_PLAIN,
				1,
				Scalar(0, 0, 0)
			);

		}


		imshow("AVC", frame);

		char c = cvWaitKey(16);
		if (c == 27) break;

		sysTimerUpdate();
		ts.dblBuff = !ts.dblBuff;
		isReady = 1;
		++FRAME_NUMBER;

		//if(wait) sleep(1);
	}

	return 0;
}
