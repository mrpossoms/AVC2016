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
#define MAX_REGIONS 256

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

typedef struct{
	Point2f topLeft, bottomRight;
	Point2f lastMean;
	Point2f mean;
	Point2f variance;
	int features;
} trackingRegion_t;

//     ___ _     _          _    
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//                               
int FRAME_NUMBER;
int MAX_REGION;
trackingRegion_t REGIONS[MAX_REGIONS];
uint8_t          REGION_COLORS[MAX_REGIONS][3];
TrackingMat* TRACKING_SPACE;

static txState_t        TRANSMIT_STATE;
static depthWindow_t    DEPTH_WINDOW;
static int              MY_SOCK;
static struct sockaddr_in* PEER;
static int NO_VIDEO_FRAMES;

int WIDTH, HEIGHT;

static Point2f featureDerivative(trackingState_t* ts, int x, int y)
{
	return ts->features.delta[y * 30 + x];
}

static Point2f gradient(trackingState_t* ts, int x, int y)
{
	if(x >= 29){
		x = 28;
	}
	else if(x < 0){
		x = 0;
	}

	if(y >= 29){
		y = 29;
	}
	else if(y < 0){
		y = 0;
	}

	Point2f f0 = featureDerivative(ts, x, y);
	Point2f f1 = featureDerivative(ts, x + 1, y + 1);


	return Point2f(f1.x - f0.x, f1.y - f0.y);
}

//    ___           _   _      ___    _   _            _   _          
//   |   \ ___ _ __| |_| |_   | __|__| |_(_)_ __  __ _| |_(_)___ _ _  
//   | |) / -_) '_ \  _| ' \  | _|(_-<  _| | '  \/ _` |  _| / _ \ ' \
//   |___/\___| .__/\__|_||_| |___/__/\__|_|_|_|_\__,_|\__|_\___/_||_|
//            |_|                                                     
void computeDepths(trackingState_t* tracking)
{
	int bufInd = tracking->dblBuff;
	CvPoint frameCenter = tracking->frameCenter;

	DEPTH_WINDOW.detectedFeatures = tracking->features.points[bufInd].size();

	for(int i = tracking->features.points[bufInd].size(); i--;){
		if(!tracking->statusVector[i]) continue;

		CvPoint2D32f centered[2] = {
			cvPoint2D32f(tracking->features.points[bufInd][i].x  - frameCenter.x, tracking->features.points[bufInd][i].y  - frameCenter.y),
			cvPoint2D32f(tracking->features.points[!bufInd][i].x - frameCenter.x, tracking->features.points[!bufInd][i].y - frameCenter.y),
		};

		float dx  = (centered[0].x - centered[1].x);
		float dy  = (centered[0].y - centered[1].y);
		float dot = dx * dx + dy * dy;
		
		// avoid taking a square root of 0
		if(dot == 0) continue;

		float s   = sqrtf(dot);
		float den = 1.0f - s;

		// no division by zero
		if(den == 0) continue;

		// assert(fabs(centered[1].x * s - centered[0].x) < 0.001);
		// assert(fabs(centered[1].y * s - centered[0].y) < 0.001);

		// use the scale of this feature whose origin has been shifted to the center
		// of the frame. The
		float depth = s * (SYS.body.estimated.velocity.y) / (1.0f - s);

		tracking->featureDepths[bufInd][i] = 100 * depth;

		DEPTH_WINDOW.depth[i].x = (centered[0].x / (float)frameCenter.x) * SHRT_MAX;
		DEPTH_WINDOW.depth[i].y = (centered[0].y / (float)frameCenter.y) * SHRT_MAX;
		DEPTH_WINDOW.depth[i].z = tracking->featureDepths[bufInd][i];
	}
}

static uint8_t regionIndexForDelta(trackingState_t* ts, int i)
{
	float s = sqrtf(ts->features.delta[i].x * ts->features.delta[i].x + ts->features.delta[i].y * ts->features.delta[i].y);
	if(ts->errorVector[i] > 1) s /= ts->errorVector[i];
	return (s / 1000) * MAX_REGIONS;
}

static Point2f featureIndexes(int i)
{
	float side = sqrtf(MAX_FEATURES);
	return Point2f((i % (int)side) * WIDTH / side, (i / side) * HEIGHT / side);
}

static void getAdjRegionIndsAndDeltas(int** inds, float* deltas)
{
	float side = sqrtf(MAX_FEATURES);

	for(int i = -1; i <= 1; ++i)
		for(int j = -1; j <= 1; ++j){
			if(!i && !j) continue;

			float s = sqrtf(ts->features.delta[i].x * ts->features.delta[i].x + ts->features.delta[i].y * ts->features.delta[i].y);
			if(ts->errorVector[i] > 1) s /= ts->errorVector[i];
			
		}


}

static void computeRegionMeans(Mat* frame, trackingState_t* ts)
{
	for(int i = ts->features.points[ts->dblBuff].size(); i--;){
		if(!ts->statusVector[i]) continue;
		
		uint8_t r = regionIndexForDelta(ts, i);

		if(r > MAX_REGION) MAX_REGION = r;

		Point2f pos = featureIndexes(i); //(ts->features.points[ts->dblBuff][i].x, ts->features.points[ts->dblBuff][i].y);
		// float s = sqrtf(ts->features.delta[i].x * ts->features.delta[i].x + ts->features.delta[i].y * ts->features.delta[i].y);
		// if(ts->errorVector[i] > 1) s /= ts->errorVector[i];
		float s = 4;

		if(r)
		rectangle(
			*frame,
			Point(ts->features.points[ts->dblBuff][i].x - s, ts->features.points[ts->dblBuff][i].y - s),
			Point(ts->features.points[ts->dblBuff][i].x + s, ts->features.points[ts->dblBuff][i].y + s),
			// Scalar(0, ts->delta[i].y * 16 + 128, ts->delta[i].x * 16 + 128),
			Scalar(REGION_COLORS[r][0], REGION_COLORS[r][1], REGION_COLORS[r][2], 128),
			-1,
			8
		);

		Point2f delta = ts->features.delta[i];
		line(
			*frame,
			ts->features.points[ts->dblBuff][i],
			ts->features.points[ts->dblBuff][i] + delta,
			Scalar(0, 128 + 255 * delta.x, 128 + 255 * delta.y)
		);

		ts->features.regionIndex[i] = r;
		REGIONS[r].mean += pos;
		REGIONS[r].features++;
	}

	for(int r = 1; r <= MAX_REGION; ++r){
		REGIONS[r].mean /= REGIONS[r].features;
	}
}

static void computeRegionVariances(Mat* frame, trackingState_t* ts)
{
	float side = sqrtf(MAX_FEATURES);
	static float lastTime;

	float dt = 1;//SYS.timeUp - lastTime;

	for(int i = ts->features.points[ts->dblBuff].size(); i--;){
		if(!ts->statusVector[i]) continue;

		Point2f pos = featureIndexes(i);//(ts->features.points[ts->dblBuff][i].x, ts->features.points[ts->dblBuff][i].y);
		uint8_t r = ts->features.regionIndex[i];

		Point2f muDelta = (REGIONS[r].mean - pos) * dt;
		Point2f muDeltaSqr(muDelta.x * muDelta.x, muDelta.y * muDelta.y);
		REGIONS[r].variance += muDeltaSqr / REGIONS[r].features;
	}

	lastTime = SYS.timeUp;

	for(int r = 1; r <= MAX_REGION; ++r){
		ellipse(
			*frame,
			Point2f(REGIONS[r].mean.x, REGIONS[r].mean.y),
			Size2f(sqrt(REGIONS[r].variance.x) , sqrt(REGIONS[r].variance.y)),
			0,
			0,
			360,
			Scalar(REGION_COLORS[r][0], REGION_COLORS[r][1], REGION_COLORS[r][2], 128)
		);
		ellipse(
			*frame,
			Point2f(REGIONS[r].mean.x, REGIONS[r].mean.y),
			Size2f(sqrt(REGIONS[r].variance.x) * 2, sqrt(REGIONS[r].variance.y) * 2),
			0,
			0,
			360,
			Scalar(REGION_COLORS[r][0], REGION_COLORS[r][1], REGION_COLORS[r][2], 128)
		);
	}

}
static float regionDensity(int r)
{
	float WIDTH  = REGIONS[r].bottomRight.x - REGIONS[r].topLeft.x;
	float HEIGHT = REGIONS[r].bottomRight.y - REGIONS[r].topLeft.y;

	return REGIONS[r].features / (WIDTH * HEIGHT);
}

static void computeRegionBBoxes(trackingState_t* ts)
{
	for(int i = ts->features.points[ts->dblBuff].size(); i--;){
		if(!ts->statusVector[i]) continue;
		
		uint8_t r = ts->features.regionIndex[i];
		Point2f inds = featureIndexes(r);
		Point2f pos(ts->features.points[ts->dblBuff][i].x, ts->features.points[ts->dblBuff][i].y);

		Point2f muDelta = REGIONS[r].mean - inds;
		Point2f stdDev(sqrtf(REGIONS[r].variance.x), sqrtf(REGIONS[r].variance.y));

		// if(regionDensity(r) < 0.0001){
		// 	REGIONS[r].features = 0;
		// 	continue;
		// }
		if(fabs(muDelta.x) > stdDev.x * 2 || fabs(muDelta.y) > stdDev.y * 2){
			REGIONS[r].features = 0;
			continue;
		}

		if(pos.x < REGIONS[r].topLeft.x)     REGIONS[r].topLeft.x     = pos.x;
		if(pos.x > REGIONS[r].bottomRight.x) REGIONS[r].bottomRight.x = pos.x;
		if(pos.y < REGIONS[r].topLeft.y)     REGIONS[r].topLeft.y     = pos.y;
		if(pos.y > REGIONS[r].bottomRight.y) REGIONS[r].bottomRight.y = pos.y;

	}
}

static void resetRegion(trackingRegion_t* region)
{
	region->topLeft = Point2f(WIDTH, HEIGHT);
	region->bottomRight = Point2f(0, 0);
	region->lastMean = region->mean;
	region->mean = Point2f(0, 0);
	region->variance = Point2f(0, 0);
	region->features = 0;	
}


int main(int argc, char* argv[])
{
	Mat frame, frameGrey, greyProc[2];

	char* hostname = NULL;
	int centerX = 320, centerY = 240;
	int isReady = 0, hasVideoFeed = 0;

	trackingState_t ts = {
		.frameCenter = cvPoint(320, 240),
	};

	VideoCapture cap(0);//cap("./SparkFun_AVC_2015.avi");
	hasVideoFeed = cap.isOpened();

	TRACKING_SPACE = new TrackingMat(Size(20, 20));

	if(argc >= 3){
		for(int i = 1; i < argc; ++i){
			if(!strncmp(argv[i], "-w", 2)){
				ts.frameCenter.x = atoi(argv[i] + 2) / 2;
			}
			if(!strncmp(argv[i], "-h", 2)){
				ts.frameCenter.y = atoi(argv[i] + 2) / 2;
			}
			if(!strncmp(argv[i], "--no-video", 10)){
				NO_VIDEO_FRAMES = 1;
				printf("Not transmitting video\n");
			}
		}
	}

	WIDTH  = ts.frameCenter.x * 2;
	HEIGHT = ts.frameCenter.y * 2;

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
	for(int i = MAX_FEATURES; i--;){
		float x = (i % (int)side) * WIDTH / side;
		float y = (i / side) * HEIGHT / side;
		ts.features.points[!ts.dblBuff].push_back(Point2f(x, y));
		ts.features.points[ts.dblBuff].push_back(Point2f(x, y));
		ts.features.delta.push_back(Point2f(0, 0));
		ts.features.regionIndex.push_back(0);
		ts.statusVector.push_back(0);
	}

	for(int i = MAX_REGIONS; i--;){
		REGION_COLORS[i][0] = random() % 128;
		REGION_COLORS[i][1] = random() % 255;
		REGION_COLORS[i][2] = random() % 255; 
	}
	REGION_COLORS[0][0] = REGION_COLORS[0][1] = REGION_COLORS[0][2] = 0;


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

			for(int i = ts.features.points[ts.dblBuff].size(); i--;){
				if(!ts.statusVector[i]) continue;
				ts.features.delta[i] = Point2f(
					ts.features.points[ts.dblBuff][i].x - ts.features.points[!ts.dblBuff][i].x,
					ts.features.points[ts.dblBuff][i].y - ts.features.points[!ts.dblBuff][i].y
				);
			}

			// TODO processing here
			computeDepths(&ts);		
		}

		// region reset
		for(int i = MAX_REGIONS; i--;){
			resetRegion(&REGIONS[i]);
		}
		// MAX_REGION = 0;

		computeRegionMeans(&frame, &ts);

		// compute the varience
		computeRegionVariances(&frame, &ts);
		computeRegionBBoxes(&ts);

		for(int r = 1; r <= MAX_REGION; ++r){
			Point2f o(10, r * 24);
			char buf[32] = {};
			
			if(!REGIONS[r].features) continue;
			
			sprintf(buf, "density: %f", regionDensity(r));
			putText(
				frame,
				buf,
				o - Point2f(1, 1),
				FONT_HERSHEY_PLAIN,
				1,
				Scalar(0, 0, 0)
			);
			putText(
				frame,
				buf,
				o,
				FONT_HERSHEY_PLAIN,
				1,
				Scalar(REGION_COLORS[r][0], REGION_COLORS[r][1], REGION_COLORS[r][2], 128)
			);
			sprintf(buf, "stdDev: (%0.3f, %0.3f)", sqrt(REGIONS[r].variance.x), sqrt(REGIONS[r].variance.y));
			putText(
				frame,
				buf,
				o + Point2f(0, 12) - Point2f(1, 1),
				FONT_HERSHEY_PLAIN,
				1,
				Scalar(0, 0, 0)
			);
			putText(
				frame,
				buf,
				o + Point2f(0, 12),
				FONT_HERSHEY_PLAIN,
				1,
				Scalar(REGION_COLORS[r][0], REGION_COLORS[r][1], REGION_COLORS[r][2], 128)
			);

			rectangle(
				frame,
				Point(REGIONS[r].topLeft.x, REGIONS[r].topLeft.y),
				Point(REGIONS[r].bottomRight.x, REGIONS[r].bottomRight.y),
				// Scalar(0, ts.delta[i].y * 16 + 128, ts.delta[i].x * 16 + 128),
				Scalar(REGION_COLORS[r][0], REGION_COLORS[r][1], REGION_COLORS[r][2], 128),
				1,
				8
			);
		}		


		imshow("AVC", frame);

		char c = cvWaitKey(16);
		if (c == 27) break;

		sysTimerUpdate();
		ts.dblBuff = !ts.dblBuff;
		isReady = 1;
		++FRAME_NUMBER;
	}

	return 0;
}