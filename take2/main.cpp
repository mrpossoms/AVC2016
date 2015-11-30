
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define LERP(start, end, p) ((start) * (1.0 - (p)) + (end) * (p))

using namespace cv;
using namespace std;

Mat lastFrame, display;

unsigned int TOTAL_FRAMES = 0;

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

	int setup;

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

			cvtColor( frame, frame, COLOR_BGR2GRAY );
			// pyrDown( frame_grey, frameSmall, Size( frame.cols/2, frame.rows/2 ));

			if(!setup){
				setup = 1;
				display = frame.clone();
			}

			/// Copy the source image
			Mat delta = frame - lastFrame;
			// copy += 128;

			display += delta;
			display *= 0.9;


			imshow(window_name, display);
			char key = (char)waitKey(10); //delay N millis, usually long enough to display and capture input

			lastFrame = frame.clone();


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
