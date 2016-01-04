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

#include "system.h"
#include "stream.h"
#include "timer.h"
#include "comms/protocol.h"
#include "comms/messages.h"
#include "sensors/aggergate.h"

// #define DBG(str){\
// 	if(errno){\
// 		printf("%s %d\n", str, errno);\
// 		assert(!errno);\
// 	}\
// }\

#define MAX_REGIONS 256

#define DBG(str){\
}\

using namespace cv;
using namespace std;

//     ___ _     _          _    
//    / __| |___| |__  __ _| |___
//   | (_ | / _ \ '_ \/ _` | (_-<
//    \___|_\___/_.__/\__,_|_/__/
//                               
int IMU_FD;
pthread_t  IMU_THREAD;
sensorStatei_t IMU_READING;
imuState_t IMU_STATE;
int FRAME_NUMBER;
size_t MTU = 4096;

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
	CvPoint         frameCenter;
	struct {
		vector<Point2f> points[2];
		vector<Point2f> delta;
		vector<uint8_t> regionIndex;
	} features;
	float           featureDepths[2][MAX_FEATURES];
	int             dblBuff;
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

int maxRegion;
trackingRegion_t regions[MAX_REGIONS];
uint8_t regionColors[MAX_REGIONS][3];

static txState_t        TRANSMIT_STATE;
static depthWindow_t    DEPTH_WINDOW;
static int              MY_SOCK;
static struct sockaddr_in* PEER;
static int NO_VIDEO_FRAMES;

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

static int procGreeting(int sock, struct sockaddr_in* addr)
{
	printf("Hello there!\n");
	
	if(!PEER){
		PEER = (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));	
	}

	MY_SOCK = sock;

	memcpy(PEER, addr, sizeof(struct sockaddr));

	uint32_t ip = ((struct sockaddr_in*)PEER)->sin_addr.s_addr;
	printf("%d.%d.%d.%d\n", ip >> 24, (ip & 0x00FFFFFF) >> 16, (ip & 0x0000FFFF) >> 8, ip & 0x000000FF);	
	assert(ip);

	return 0;
}

//    ___ __  __ _   _    ___                      
//   |_ _|  \/  | | | |  / __|___ _ __  _ __  ___  
//    | || |\/| | |_| | | (__/ _ \ '  \| '  \(_-<_ 
//   |___|_|  |_|\___/   \___\___/_|_|_|_|_|_/__(_)
//                                                 
void* imuHandler(void* param)
{
	const int samples = 50;
	int i = 0;

	int readings[3][50] = {{},{},{}};
	int origin[100];

	for(int i = 100; i--; origin[i] = 0)
		bzero(readings[0], sizeof(int) * samples);

	while(1){
		imuUpdateState(IMU_FD, &IMU_STATE);
		sensorStatef_t reading = IMU_STATE.adjReadings;

		int topLeft[2] = { 5, 2 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};	
		
		int center = (bottomRight[1] - topLeft[1]) / 2;

		const int scale = 500;

		readings[0][i] = center + reading.linear.x / scale;
		readings[1][i] = center + reading.linear.y / scale;
		readings[2][i] = center + reading.linear.z / scale;

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

static void computeRegionBBoxes(trackingState_t* ts);
static void computeRegionMeans(Mat* frame, trackingState_t* ts);
static void computeRegionVariances(Mat* frame, trackingState_t* ts);
static void resetRegion(trackingRegion_t* region);
static float regionDensity(int r);
static uint8_t regionIndexForDelta(trackingState_t* ts, int deltaIndex);

int width, height;
int main(int argc, char* argv[])
{
	Mat frame, frameGrey, greyProc[2];

	char* hostname = NULL;
	int centerX = 320, centerY = 240;
	int isReady = 0, hasVideoFeed = 0;

	commRegisterRxProc(MSG_GREETING, procGreeting);
	assert(!commInitHost(1337));

	trackingState_t ts = {
		.frameCenter = cvPoint(320, 240),
	};

	VideoCapture cap("./SparkFun_AVC_2015.avi");
	hasVideoFeed = cap.isOpened();

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

	width  = ts.frameCenter.x * 2;
	height = ts.frameCenter.y * 2;

	if(hasVideoFeed){
		// set capture size
		cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
		cap.set(CV_CAP_PROP_FPS, 2);
		printf("Capture dimensions (%d, %d)\n", width, height);
	}
	else{
		printf("No capture\n");
	}

	errno = 0;
#ifdef __linux__
	int res = senInit("/dev/i2c-1", "/dev/ttyAMA0", "./../imu.cal");
#endif

	DBG("");
	
#ifdef __linux__
	icInit();
	pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);
#elif defined(__APPLE__)
	namedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	errno = 0;
#endif
	
	float side = sqrtf(MAX_FEATURES);
	for(int i = MAX_FEATURES; i--;){
		float x = (i % (int)side) * width / side;
		float y = (i / side) * height / side;
		ts.features.points[!ts.dblBuff].push_back(Point2f(x, y));
		ts.features.points[ts.dblBuff].push_back(Point2f(x, y));
		ts.features.delta.push_back(Point2f(0, 0));
		ts.features.regionIndex.push_back(0);
		ts.statusVector.push_back(0);
	}

	for(int i = MAX_REGIONS; i--;){
		regionColors[i][0] = random() % 128;
		regionColors[i][1] = random() % 255;
		regionColors[i][2] = random() % 255; 
	}
	regionColors[0][0] = regionColors[0][1] = regionColors[0][2] = 0;


	while(1){
		Mat currFrame;

		float side = sqrtf(MAX_FEATURES);
		for(int i = MAX_FEATURES; i--;){

			float x = (i % (int)side) * width / side;
			float y = (i / side) * height / side;
			ts.features.points[!ts.dblBuff][i] = (Point2f(x, y));
			ts.features.points[ts.dblBuff][i] = (Point2f(x, y));
			ts.features.regionIndex[i] = 0;
		}

		cap >> currFrame;

		if(currFrame.empty()){
			printf("Is empty\n");
			continue;
		}

		resize(currFrame, currFrame, Size(width, height), 0, 0, INTER_CUBIC);

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

#ifdef __APPLE__
		// region reset
		for(int i = MAX_REGIONS; i--;){
			resetRegion(&regions[i]);
		}
		// maxRegion = 0;

		computeRegionMeans(&frame, &ts);

		// compute the varience
		computeRegionVariances(&frame, &ts);

		computeRegionBBoxes(&ts);

		for(int r = 1; r <= maxRegion; ++r){
			Point2f o(10, r * 24);
			char buf[32] = {};
			
			if(!regions[r].features) continue;
			
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
				Scalar(regionColors[r][0], regionColors[r][1], regionColors[r][2], 128)
			);
			sprintf(buf, "stdDev: (%0.3f, %0.3f)", sqrt(regions[r].variance.x), sqrt(regions[r].variance.y));
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
				Scalar(regionColors[r][0], regionColors[r][1], regionColors[r][2], 128)
			);

			rectangle(
				frame,
				Point(regions[r].topLeft.x, regions[r].topLeft.y),
				Point(regions[r].bottomRight.x, regions[r].bottomRight.y),
				// Scalar(0, ts.delta[i].y * 16 + 128, ts.delta[i].x * 16 + 128),
				Scalar(regionColors[r][0], regionColors[r][1], regionColors[r][2], 128),
				1,
				8
			);
		}		


		imshow("AVC", frame);
#else
		senUpdate(&SYS.body);
#endif

		// DBG("");
		// if(PEER){
		// 	int res = 0;

		// 	if(!NO_VIDEO_FRAMES){
		// 		res = txFrame(
		// 			MY_SOCK,
		// 			(struct sockaddr_in*)PEER,
		// 			width, height, 
		// 			&TRANSMIT_STATE,
		// 			(const char*)greyProc[ts.dblBuff].data
		// 		);
		// 	}
		// 	DEPTH_WINDOW.detectedFeatures = 400;
		// 	commSend(MSG_TRACKING, &DEPTH_WINDOW, sizeof(DEPTH_WINDOW), PEER);
		// }

		//commListen();

		char c = cvWaitKey(16);
		if (c == 27) break;

		timerUpdate();
		ts.dblBuff = !ts.dblBuff;
		isReady = 1;
		++FRAME_NUMBER;
	}

	return 0;
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
	return Point2f((i % (int)side) * width / side, (i / side) * height / side);
}

static void computeRegionMeans(Mat* frame, trackingState_t* ts)
{
	for(int i = ts->features.points[ts->dblBuff].size(); i--;){
		if(!ts->statusVector[i]) continue;
		
		uint8_t r = regionIndexForDelta(ts, i);

		if(r > maxRegion) maxRegion = r;

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
			Scalar(regionColors[r][0], regionColors[r][1], regionColors[r][2], 128),
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
		regions[r].mean += pos;
		regions[r].features++;
	}

	for(int r = 1; r <= maxRegion; ++r){
		regions[r].mean /= regions[r].features;
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

		Point2f muDelta = (regions[r].mean - pos) * dt;
		Point2f muDeltaSqr(muDelta.x * muDelta.x, muDelta.y * muDelta.y);
		regions[r].variance += muDeltaSqr / regions[r].features;
	}

	lastTime = SYS.timeUp;

	for(int r = 1; r <= maxRegion; ++r){
		ellipse(
			*frame,
			Point2f(regions[r].mean.x, regions[r].mean.y),
			Size2f(sqrt(regions[r].variance.x) , sqrt(regions[r].variance.y)),
			0,
			0,
			360,
			Scalar(regionColors[r][0], regionColors[r][1], regionColors[r][2], 128)
		);
		ellipse(
			*frame,
			Point2f(regions[r].mean.x, regions[r].mean.y),
			Size2f(sqrt(regions[r].variance.x) * 2, sqrt(regions[r].variance.y) * 2),
			0,
			0,
			360,
			Scalar(regionColors[r][0], regionColors[r][1], regionColors[r][2], 128)
		);
	}

}
static float regionDensity(int r)
{
	float width  = regions[r].bottomRight.x - regions[r].topLeft.x;
	float height = regions[r].bottomRight.y - regions[r].topLeft.y;

	return regions[r].features / (width * height);
}

static void computeRegionBBoxes(trackingState_t* ts)
{
	for(int i = ts->features.points[ts->dblBuff].size(); i--;){
		if(!ts->statusVector[i]) continue;
		
		uint8_t r = ts->features.regionIndex[i];
		Point2f inds = featureIndexes(r);
		Point2f pos(ts->features.points[ts->dblBuff][i].x, ts->features.points[ts->dblBuff][i].y);

		Point2f muDelta = regions[r].mean - inds;
		Point2f stdDev(sqrtf(regions[r].variance.x), sqrtf(regions[r].variance.y));

		// if(regionDensity(r) < 0.0001){
		// 	regions[r].features = 0;
		// 	continue;
		// }
		if(fabs(muDelta.x) > stdDev.x * 2 || fabs(muDelta.y) > stdDev.y * 2){
			regions[r].features = 0;
			continue;
		}

		if(pos.x < regions[r].topLeft.x)     regions[r].topLeft.x     = pos.x;
		if(pos.x > regions[r].bottomRight.x) regions[r].bottomRight.x = pos.x;
		if(pos.y < regions[r].topLeft.y)     regions[r].topLeft.y     = pos.y;
		if(pos.y > regions[r].bottomRight.y) regions[r].bottomRight.y = pos.y;

	}
}

static void resetRegion(trackingRegion_t* region)
{
	region->topLeft = Point2f(width, height);
	region->bottomRight = Point2f(0, 0);
	region->lastMean = region->mean;
	region->mean = Point2f(0, 0);
	region->variance = Point2f(0, 0);
	region->features = 0;	
}
