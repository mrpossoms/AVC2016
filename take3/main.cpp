
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include <indicurses.h>

#include "imu.h"

#define LERP(start, end, p) ((start) * (1.0 - (p)) + (end) * (p))

using namespace cv;
using namespace std;


const int MAX_CORNERS = 400;
int IMU_FD;
pthread_t  IMU_THREAD;
readings_t IMU_READING;

Mat frame_grey, last_grey_frame, display_frame;
unsigned int TOTAL_FRAMES = 0;

vector<Point2f> corners, lastCorners;
vector<unsigned char> status;
vector<float> err;

//hide the local functions in an anon namespace
namespace {
	void help(char** av) {
		cout << "The program captures frames from a video file, image sequence (01.jpg, 02.jpg ... 10.jpg) or camera connected to your computer." << endl
			 << "Usage:\n" << av[0] << " <video file, image sequence or device number>" << endl
			 << "q,Q,esc -- quit" << endl
			 << "space   -- save frame" << endl << endl
			 << "\tTo capture from a camera pass the device number. To find the device number, try ls /dev/video*" << endl
			 << "\texample: " << av[0] << " 0" << endl
			 << "\tYou may also pass a video file instead of a device number" << endl
			 << "\texample: " << av[0] << " video.avi" << endl
			 << "\tYou can also pass the path to an image sequence and OpenCV will treat the sequence just like a video." << endl
			 << "\texample: " << av[0] << " right%%02d.jpg" << endl;
	}

	RNG rng(12345);

	int process(VideoCapture& capture) {
		int n = 0;
		char filename[200];
		string window_name = "video | q or esc to quit";
		cout << "press space to save a picture. q or esc to quit" << endl;
		namedWindow(window_name, WINDOW_KEEPRATIO); //resizable window;
		Mat frame, frameSmall;

		for (;;) {
			capture >> frame;

			if (frame.empty())
				break;

			cvtColor(frame, frame_grey, COLOR_BGR2GRAY );
			pyrDown( frame_grey, frame_grey, Size( frame.cols/2, frame.rows/2 ));

			/// Parameters for Shi-Tomasi algorithm
			double qualityLevel = 0.01;
			double minDistance = 0.01;
			int blockSize = 3;
			bool useHarrisDetector = false;
			double k = 0.4;

			/// Copy the source image

			if(lastCorners.size()){
				calcOpticalFlowPyrLK(
					last_grey_frame,
					frame_grey,
					lastCorners,
					corners,
					status,
					err
				);
			}

			display_frame = frame.clone();

			display_frame *= 0.1;

			int r = 3;
			for( int i = 0; i < corners.size(); i++ ){
				if(status[i]){
					float dx = (corners[i].x - lastCorners[i].x) * 10;
					float dy = (corners[i].y - lastCorners[i].y) * 10;
					float depth = dx * dx + dy * dy;

					depth = pow(depth, 64);

					line(
						display_frame,
						Point(corners[i].x * 2, corners[i].y * 2),
						Point((corners[i].x + dx) * 2 , (corners[i].y + dy) * 2),
						Scalar(depth, dy + 128, dx + 128),
						1, 
						8,
						0
					);
				}
			}


			char buf[64] = {};
			sprintf(buf, "Frames: %u", TOTAL_FRAMES++);
			putText(
				display_frame,
				buf,
				Point(10, 20),
				FONT_HERSHEY_SIMPLEX,
				0.5,
				Scalar(255, 255, 255), 
				1,
				8
			);

			sprintf(buf, "acc = (%hd, %hd, %hd) ", 
				IMU_READING.accLinear.v[0],
				IMU_READING.accLinear.v[1],
				IMU_READING.accLinear.v[2]
			);
			putText(
				display_frame,
				buf,
				Point(10, 40),
				FONT_HERSHEY_SIMPLEX,
				0.5,
				Scalar(255, 255, 255), 
				1,
				8
			);

			/// Apply corner detection
			goodFeaturesToTrack(
				frame_grey,
				corners,
				MAX_CORNERS,
				qualityLevel,
				minDistance,
				Mat(),
				blockSize,
				useHarrisDetector,
				k 
			);

			imshow(window_name, display_frame);

			last_grey_frame = frame_grey.clone();
			lastCorners = corners;

			char key = (char)waitKey(10); //delay N millis, usually long enough to display and capture input

			switch (key) {
			case 'q':
			case 'Q':
			case 27: //escape key
				return 0;
			case ' ': //Save an image
			   break;
			default:
				break;
			}
		}
		return 0;
	}
}

long variance(int* data, int offset, int length)
{
	long var = 0;

	if(offset - 10 < 0){
		offset = length + (offset - 10);
	}

	for(int i = 10; i--;){
		int j = (i + 1) % length;
		int delta = data[i] - data[j];

		var += delta * delta;		
	}

	return var;
}

int isOutOfSync(int* data, int offset, int length){
	long var = 0;

	if(offset - 10 < 0){
		offset = length + (offset - 10);
	}

	for(int i = 10; i--;){
		int j = (i + 1) % length;
		int delta = data[i] - data[j];

		var += delta * delta;		
	}

	if(var > 100){
		return 1;
	}

	return 0;
}

void* imuHandler(void* param)
{
	const int samples = 50;
	int i = 0;

	int readings[3][samples] = {{},{},{}};
	int origin[100];
	int center = (IC_TERM_HEIGHT / 2) - 4;

	for(int i = 100; i--; origin[i] = center)
		bzero(readings[0], sizeof(int) * samples);

	while(1){
		long var = variance(readings[0], i, samples);

		if(isOutOfSync(readings[0], i, samples)){
			// bzero(readings[0], sizeof(int) * samples);
			// bzero(readings[1], sizeof(int) * samples);
			// bzero(readings[2], sizeof(int) * samples);
			// imuSynch(IMU_FD);
		}

		IMU_READING = imuGetReadings(IMU_FD);
		
		const int scale = 500;

		readings[0][i] = center + IMU_READING.accLinear.x / scale;
		readings[1][i] = center + IMU_READING.accLinear.y / scale;
		readings[2][i] = center + IMU_READING.accLinear.z / scale;

		++i;
		i %= samples;

		int topLeft[2] = { 25, 2 };
		int bottomRight[2] = { IC_TERM_WIDTH - 5, IC_TERM_HEIGHT - 2};
		clear();
		icLineGraph(topLeft, bottomRight, '-', origin, 100);
		icLineGraph(topLeft, bottomRight, 'x', readings[0], samples);
		icLineGraph(topLeft, bottomRight, 'y', readings[1], samples);
		icLineGraph(topLeft, bottomRight, 'z', readings[2], samples);

		char count[32] = {};
		sprintf(count, "%d", var);
		icText(2, 2, count);

		icPresent();
	}

	return NULL;
}

int main(int ac, char** av) {

	if (ac != 3) {
		help(av);
		return 1;
	}

	std::string arg = av[1];
	VideoCapture capture(arg); //try to open string, this will attempt to open it as a video file or image sequence

	IMU_FD = open(av[2], O_RDWR);

	imuConfigSerial(IMU_FD, 9600);

	if (!capture.isOpened()) //if this fails, try to open as a video camera, through the use of an integer param
		capture.open(atoi(arg.c_str()));
	if (!capture.isOpened()) {
		cerr << "Failed to open the video device, video file or image sequence!\n" << endl;
		help(av);
		return 1;
	}

	int w, h;

	cout << "Width  " << (w = capture.get(CV_CAP_PROP_FRAME_WIDTH)) << "\n";
	cout << "Height " << (h = capture.get(CV_CAP_PROP_FRAME_HEIGHT)) << "\n";

//	pthread_create(&IMU_THREAD, NULL, imuHandler, NULL);
	icInit();

	return process(capture);
}
