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

#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <indicurses.h>
#include <pthread.h>

#include <netdb.h>
#include <arpa/inet.h>
#include <sys/socket.h>


#include "imu.h"
#include "stream.h"

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
const int MAX_FEATURES = 400;

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

static txState_t TRANSMIT_STATE;

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
		IMU_READING = IMU_STATE.lastReadings;

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

//    ___           _   _      ___    _   _            _   _          
//   |   \ ___ _ __| |_| |_   | __|__| |_(_)_ __  __ _| |_(_)___ _ _  
//   | |) / -_) '_ \  _| ' \  | _|(_-<  _| | '  \/ _` |  _| / _ \ ' \ 
//   |___/\___| .__/\__|_||_| |___/__/\__|_|_|_|_\__,_|\__|_\___/_||_|
//            |_|                                                     
void computeDepths(trackingState_t* tracking)
{
	int bufInd = tracking->dblBuff;
	CvPoint frameCenter = tracking->frameCenter;

	for(int i = tracking->features[bufInd].size(); i--;){
		if(!tracking->statusVector[i]) continue;

		CvPoint2D32f centered[2] = {
			cvPoint2D32f(tracking->features[bufInd][i].x  - frameCenter.x, tracking->features[bufInd][i].y  - frameCenter.y),
			cvPoint2D32f(tracking->features[!bufInd][i].x - frameCenter.x, tracking->features[!bufInd][i].y - frameCenter.y),
		};

		float dx = (centered[0].x - centered[1].x);
		float dy = (centered[0].y - centered[1].y);
		float s = sqrtf(dx * dx - dy * dy);

		// assert(fabs(centered[1].x * s - centered[0].x) < 0.001);
		// assert(fabs(centered[1].y * s - centered[0].y) < 0.001);

		// use the scale of this feature whose origin has been shifted to the center
		// of the frame. The 
		tracking->featureDepths[bufInd][i] = s * IMU_STATE.velocities.linear.y / (1.0f - s);
	}
}

int main(int argc, char* argv[])
{
	Mat frame, frameGrey, greyProc[2];

	char* hostname = NULL;
	int centerX = 320, centerY = 240;
	int width = centerX * 2, height = centerY * 2;
	int sock = socket(AF_INET, SOCK_DGRAM, 0);

	if(argc < 2){
		printf("Please provide at least a host\n");
		return -1;
	}

	hostname = argv[1];

	if(argc >= 3){
		for(int i = 2; i < argc; ++i){
			if(!strncmp(argv[i], "-w", 2)){
				centerX = atoi(argv[i] + 2) / 2;
			}
			if(!strncmp(argv[i], "-h", 2)){
				centerY = atoi(argv[i] + 2) / 2;
			}
		}
	}

	struct hostent *hp;
	struct sockaddr_in addr = {};

	addr.sin_family = AF_INET;
	addr.sin_port   = htons(1337);

	trackingState_t ts = {
		.frameCenter = cvPoint(centerX, centerY),
	};

	int isReady = 0;

	VideoCapture cap(0);
	assert(cap.isOpened());

	// set capture size
	cap.set(CV_CAP_PROP_FRAME_WIDTH,  width  = ts.frameCenter.x * 2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height = ts.frameCenter.y * 2);

	printf("Capture dimensions (%d, %d)\n", width, height);

	errno = 0;
#ifdef __linux__
	// open the I2C device
	IMU_FD = open("/dev/i2c-1", O_RDWR);
#endif

	assert(!errno);
	hp = gethostbyname(hostname);
	memcpy((void *)&addr.sin_addr, hp->h_addr_list[0], hp->h_length);
	assert(!errno);

	
#ifdef __linux__
//	icInit();
//	pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);
#elif defined(__APPLE__)
	namedWindow("AVC", CV_WINDOW_AUTOSIZE); //resizable window;
	errno = 0;
#endif
	int cornerCount = MAX_FEATURES;

	assert(!errno);

	while(1){
		assert(!errno);

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
		// cvtColor(frame, frameGrey, CV_BGR2GRAY);
		// medianBlur(frameGrey, greyProc[ts.dblBuff], 3);


		cvtColor(frame, greyProc[ts.dblBuff], CV_BGR2GRAY);

		
		if(isReady){
			assert(!errno);
			calcOpticalFlowPyrLK(
				greyProc[!ts.dblBuff],
				greyProc[ts.dblBuff],
				ts.features[!ts.dblBuff],
				ts.features[ts.dblBuff],
				ts.statusVector,      // list of bools which inicate feature matches
				ts.errorVector
			);

			// TODO processing here
			//computeDepths(&ts);		
			assert(!errno);
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

		assert(!errno);
		if(!(FRAME_NUMBER % 1)){
			int res = txFrame(
				sock,
				(const struct sockaddr*)&addr,
				width, height, 
				&TRANSMIT_STATE,
				(const char*)greyProc[ts.dblBuff].data
			);

			if(res < 0){
				printf("Error %d\n", errno);
			}

		}
		assert(!errno);

		// detect features
		cornerCount = 400;
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

		assert(!errno);
		ts.dblBuff = !ts.dblBuff;
		isReady = 1;
		++FRAME_NUMBER;
	}

	return 0;
}
