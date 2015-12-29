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
#include "comms/protocol.h"
#include "comms/messages.h"
#include "sensors/imu.h"

// #define DBG(str){\
// 	if(errno){\
// 		printf("%s %d\n", str, errno);\
// 		assert(!errno);\
// 	}\
// }\

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
	vector<Point2f> features[2];
	float           featureDepths[2][MAX_FEATURES];
	int             dblBuff;
	vector<unsigned char> statusVector;
	vector<float>         errorVector;
} trackingState_t;

static txState_t        TRANSMIT_STATE;
static depthWindow_t    DEPTH_WINDOW;
static int              MY_SOCK;
static struct sockaddr_in* PEER;
static int NO_VIDEO_FRAMES;

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

	DEPTH_WINDOW.detectedFeatures = tracking->features[bufInd].size();

	for(int i = tracking->features[bufInd].size(); i--;){
		if(!tracking->statusVector[i]) continue;

		CvPoint2D32f centered[2] = {
			cvPoint2D32f(tracking->features[bufInd][i].x  - frameCenter.x, tracking->features[bufInd][i].y  - frameCenter.y),
			cvPoint2D32f(tracking->features[!bufInd][i].x - frameCenter.x, tracking->features[!bufInd][i].y - frameCenter.y),
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
		static float deepest;
		float depth = s * SYS.body.estimated.velocity.y / (1.0f - s);

		printf("depth %f\n", depth);

		if(fabs(depth) > deepest){
			printf("Deepest %f\n", deepest = depth);
		}

		tracking->featureDepths[bufInd][i] = 10 * depth;

		DEPTH_WINDOW.depth[i].x = (centered[0].x / (float)frameCenter.x) * SHRT_MAX;
		DEPTH_WINDOW.depth[i].y = (centered[0].y / (float)frameCenter.y) * SHRT_MAX;
		DEPTH_WINDOW.depth[i].z = tracking->featureDepths[bufInd][i];
	}
}

int main(int argc, char* argv[])
{
	Mat frame, frameGrey, greyProc[2];

	char* hostname = NULL;
	int centerX = 320, centerY = 240;
	int width = centerX * 2, height = centerY * 2;
	int isReady = 0, hasVideoFeed = 0;

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

	commRegisterRxProc(MSG_GREETING, procGreeting);
	assert(!commInitHost(1337));

	trackingState_t ts = {
		.frameCenter = cvPoint(centerX, centerY),
	};

	VideoCapture cap(0);
	hasVideoFeed = cap.isOpened();

	width  = ts.frameCenter.x * 2;
	height = ts.frameCenter.y * 2;

	if(hasVideoFeed){
		// set capture size
		cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
		cap.set(CV_CAP_PROP_FPS, 12);
		printf("Capture dimensions (%d, %d)\n", width, height);
	}

	errno = 0;
#ifdef __linux__
	// open the I2C device
	IMU_FD = open("/dev/i2c-1", O_RDWR);
#endif

	DBG("");
	
#ifdef __linux__
	//icInit();
	pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);
#elif defined(__APPLE__)
	namedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	errno = 0;
#endif
	int cornerCount = MAX_FEATURES;
	
	DBG("");

	while(1){
		DBG("");

		Mat currFrame;

		if(hasVideoFeed){
			cap >> currFrame;
			if(currFrame.empty()) continue;
		}
		else{
			static int rand_fd;
			if(!rand_fd){
				rand_fd = open("/dev/random", O_RDONLY);
			}	

			currFrame.create(height, width, CV_8UC(3));
			read(rand_fd, currFrame.data, width * height * 3);
		}

		if(!isReady){
			frame = currFrame.clone();
			cvtColor(frame, frameGrey, CV_BGR2GRAY);
			greyProc[0] = frameGrey.clone();
			greyProc[1] = frameGrey.clone();
		}

		currFrame.copyTo(frame);
		cvtColor(frame, greyProc[ts.dblBuff], CV_BGR2GRAY);
		
		if(isReady){
			DBG("");
			calcOpticalFlowPyrLK(
				greyProc[!ts.dblBuff],
				greyProc[ts.dblBuff],
				ts.features[!ts.dblBuff],
				ts.features[ts.dblBuff],
				ts.statusVector,      // list of bools which inicate feature matches
				ts.errorVector
			);

			// TODO processing here
			computeDepths(&ts);		
			DBG("");
		}

#ifdef __APPLE__
		for(int i = ts.features[ts.dblBuff].size(); i--;){
			if(!ts.statusVector[i]) continue;
			circle(
				frame,
				Point(ts.features[ts.dblBuff][i].x, ts.features[ts.dblBuff][i].y),
				3,
				Scalar(255, 0, 0, 255),
				1,
				8,
				0
			);

			float dx = (ts.features[ts.dblBuff][i].x - ts.features[!ts.dblBuff][i].x) * 10;
			float dy = (ts.features[ts.dblBuff][i].y - ts.features[!ts.dblBuff][i].y) * 10;
			float depth = dx * dx + dy * dy;

			depth = pow(depth, 64);

			line(
				frame,
				Point(ts.features[ts.dblBuff][i].x, ts.features[ts.dblBuff][i].y),
				Point((ts.features[ts.dblBuff][i].x + dx) , (ts.features[ts.dblBuff][i].y + dy)),
				Scalar(depth, dy + 128, dx + 128, 255 / (ts.errorVector[i] + 1)),
				1, 
				8,
				0
			);
		}

		imshow("AVC", frame);
#endif
		DBG("");
		if(PEER){
			int res = 0;

			if(!NO_VIDEO_FRAMES){
				res = txFrame(
					MY_SOCK,
					(struct sockaddr_in*)PEER,
					width, height, 
					&TRANSMIT_STATE,
					(const char*)greyProc[ts.dblBuff].data
				);
			}
			commSend(MSG_TRACKING, &DEPTH_WINDOW, sizeof(DEPTH_WINDOW), PEER);
		}

		commListen();
		DBG("");

		// detect features
		goodFeaturesToTrack(
			greyProc[ts.dblBuff],
			ts.features[ts.dblBuff],
			MAX_FEATURES,
			0.01,        // quality
			0.01,        // min distance
			Mat(),       // mask for ROI
			9,           // block size
			0,           // use harris detector
			0.04         // not used (free param of harris)
		);

		DBG("");
		ts.dblBuff = !ts.dblBuff;
		isReady = 1;
		++FRAME_NUMBER;
	}

	return 0;
}
