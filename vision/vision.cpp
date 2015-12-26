#include <limits.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

#include "vision.h"

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

static Mat frame, frameGrey, greyProc[2];
static int centerX = 320, centerY = 240;
static int width, height;
static int isReady, hasVideoFeed;
static VideoCapture CAP;


//     ___             _            _      
//    / __|___ _ _  __| |_ __ _ _ _| |_ ___
//   | (__/ _ \ ' \(_-<  _/ _` | ' \  _(_-<
//    \___\___/_||_/__/\__\__,_|_||_\__/__/
//                                         
const float FOCAL_PLANE = 1;

//    ___           _   _      ___    _   _            _   _          
//   |   \ ___ _ __| |_| |_   | __|__| |_(_)_ __  __ _| |_(_)___ _ _  
//   | |) / -_) '_ \  _| ' \  | _|(_-<  _| | '  \/ _` |  _| / _ \ ' \
//   |___/\___| .__/\__|_||_| |___/__/\__|_|_|_|_\__,_|\__|_\___/_||_|
//            |_|                                                     
void computeDepths(trackingState_t* tracking)
{
	int bufInd = tracking->dblBuff;
	CvPoint frameCenter = tracking->frameCenter;

	SYS.window.detectedFeatures = tracking->features[bufInd].size();

	for(int i = tracking->features[bufInd].size(); i--;){
		if(!tracking->statusVector[i]) return;

		CvPoint2D32f centered[2] = {
			cvPoint2D32f(tracking->features[bufInd][i].x  - frameCenter.x, tracking->features[bufInd][i].y  - frameCenter.y),
			cvPoint2D32f(tracking->features[!bufInd][i].x - frameCenter.x, tracking->features[!bufInd][i].y - frameCenter.y),
		};

		float dx  = (centered[0].x - centered[1].x);
		float dy  = (centered[0].y - centered[1].y);
		float dot = dx * dx + dy * dy;
		
		// avoid taking a square root of 0
		if(dot == 0) return;

		float s   = sqrtf(dot);
		float den = 1.0f - s;

		// no division by zero
		if(den == 0) return;

		// use the scale of this feature whose origin has been shifted to the center
		// of the frame. The 
		tracking->featureDepths[bufInd][i] = s * SYS.body.estimated.velocity.y / (1.0f - s);

		SYS.window.depth[i].x = (centered[0].x / (float)frameCenter.x) * SHRT_MAX;
		SYS.window.depth[i].y = (centered[0].y / (float)frameCenter.y) * SHRT_MAX;
		SYS.window.depth[i].z = tracking->featureDepths[bufInd][i];
	}
}

int visionInit(trackingState_t* tracking, int w, int h)
{
	width = w; height = h;

	centerX = width / 2;
	centerY = height / 2;

	tracking->frameCenter = cvPoint(centerX, centerY);

	VideoCapture cap(0);
	hasVideoFeed = cap.isOpened();

	if(hasVideoFeed){
		// set capture size
		cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
		printf("Capture dimensions (%d, %d)\n", width, height);
	}

	CAP = cap;

	errno = 0;
#if defined(__APPLE__)
	namedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	errno = 0;
#endif

	return 0;
}

void visionUpdate(trackingState_t* tracking)
{
	DBG("");

	Mat currFrame;

	if(hasVideoFeed){
		CAP >> currFrame;
		if(currFrame.empty()) return;
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
	cvtColor(frame, greyProc[tracking->dblBuff], CV_BGR2GRAY);
	
	if(isReady){
		DBG("");
		calcOpticalFlowPyrLK(
			greyProc[!tracking->dblBuff],
			greyProc[tracking->dblBuff],
			tracking->features[!tracking->dblBuff],
			tracking->features[tracking->dblBuff],
			tracking->statusVector,      // list of bools which inicate feature matches
			tracking->errorVector
		);

		computeDepths(tracking);		
		DBG("");
	}

#ifdef __APPLE__
	for(int i = tracking->features[tracking->dblBuff].size(); i--;){
		if(!tracking->statusVector[i]) return;
		circle(
			frame,
			Point(tracking->features[tracking->dblBuff][i].x, tracking->features[tracking->dblBuff][i].y),
			3,
			Scalar(255, 0, 0, 255),
			1,
			8,
			0
		);

		float dx = (tracking->features[tracking->dblBuff][i].x - tracking->features[!tracking->dblBuff][i].x) * 10;
		float dy = (tracking->features[tracking->dblBuff][i].y - tracking->features[!tracking->dblBuff][i].y) * 10;
		float depth = dx * dx + dy * dy;

		depth = pow(depth, 64);

		line(
			frame,
			Point(tracking->features[tracking->dblBuff][i].x, tracking->features[tracking->dblBuff][i].y),
			Point((tracking->features[tracking->dblBuff][i].x + dx) , (tracking->features[tracking->dblBuff][i].y + dy)),
			Scalar(depth, dy + 128, dx + 128, 255 / (tracking->errorVector[i] + 1)),
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
		greyProc[tracking->dblBuff],
		tracking->features[tracking->dblBuff],
		MAX_FEATURES,
		0.01,        // quality
		0.01,        // min distance
		Mat(),       // mask for ROI
		3,           // block size
		0,           // use harris detector
		0.04         // not used (free param of harris)
	);

	DBG("");
	tracking->dblBuff = !tracking->dblBuff;
	isReady = 1;
	++FRAME_NUMBER;

}

