#include <limits.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>


#include "stream.h"
#include "comms/protocol.h"
#include "comms/messages.h"

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
int FRAME_NUMBER;
static depthWindow_t    DEPTH_WINDOW;

static Mat frame, frameGrey, greyProc[2];
static int centerX = 320, centerY = 240;
static int width, height;
static int isReady, hasVideoFeed;



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
		tracking->featureDepths[bufInd][i] = s * IMU_STATE.velocities.linear.y / (1.0f - s);

		DEPTH_WINDOW.depth[i].x = (centered[0].x / (float)frameCenter.x) * SHRT_MAX;
		DEPTH_WINDOW.depth[i].y = (centered[0].y / (float)frameCenter.y) * SHRT_MAX;
		DEPTH_WINDOW.depth[i].z = tracking->featureDepths[bufInd][i];
	}
}

int main(int argc, char* argv[])
{
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

			if(!(FRAME_NUMBER % 1)){
				res = txFrame(
					MY_SOCK,
					(struct sockaddr_in*)PEER,
					width, height, 
					&TRANSMIT_STATE,
					(const char*)greyProc[ts.dblBuff].data
				);
				commSend(MSG_TRACKING, &DEPTH_WINDOW, sizeof(DEPTH_WINDOW), PEER);
			}

			if(res < 0){
				printf("Error %d\n", errno);
			}
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
			3,           // block size
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

int visionInit(int w, int h)
{
	width = w; height = h;

	centerX = width / 2;
	centerY = height / 2;

	commRegisterRxProc(MSG_GREETING, procGreeting);

	trackingState_t ts = {
		.frameCenter = cvPoint(centerX, centerY),
	};

	VideoCapture cap(0);
	hasVideoFeed = cap.isOpened();

	if(hasVideoFeed){
		// set capture size
		cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
		printf("Capture dimensions (%d, %d)\n", width, height);
	}

	errno = 0;
#if defined(__APPLE__)
	namedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	errno = 0;
#endif

}

void visionUpdate()
{
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

	// detect features
	goodFeaturesToTrack(
		greyProc[ts.dblBuff],
		ts.features[ts.dblBuff],
		MAX_FEATURES,
		0.01,        // quality
		0.01,        // min distance
		Mat(),       // mask for ROI
		3,           // block size
		0,           // use harris detector
		0.04         // not used (free param of harris)
	);

	DBG("");
	ts.dblBuff = !ts.dblBuff;
	isReady = 1;
	++FRAME_NUMBER;

}

