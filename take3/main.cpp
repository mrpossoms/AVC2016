
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define LERP(start, end, p) ((start) * (1.0 - (p)) + (end) * (p))

using namespace cv;
using namespace std;


const int MAX_CORNERS = 400;

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

			char buf[16] = {};
			sprintf(buf, "Frames: %u", TOTAL_FRAMES++);

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

int main(int ac, char** av) {

	if (ac != 2) {
		help(av);
		return 1;
	}
	std::string arg = av[1];
	VideoCapture capture(arg); //try to open string, this will attempt to open it as a video file or image sequence

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

	return process(capture);
}
