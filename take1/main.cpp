
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/videoio/videoio.hpp>

#include <opencv2/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#define LERP(start, end, p) ((start) * (1.0 - (p)) + (end) * (p))

using namespace cv;
using namespace std;

typedef struct{
	int          featureId;
	unsigned int born;
	unsigned int tracked;
} trackingCell_t;

typedef struct {
	int featureId;
	unsigned int lastUpdated;

	int key[2];

	float strength;
	float pos[2], dispPos[2];
	float depth;
	float velocity[2];

} trackingFeature_t;

const int MAX_CORNERS = 100;

trackingCell_t** FEATURE_MAP;
int FEATURE_MAP_DIMS[2];

trackingFeature_t* FEATURE_LIST_SORTED[MAX_CORNERS];
trackingFeature_t FEATURE_LIST[MAX_CORNERS];

Mat frame_grey, last_grey_frame;

unsigned int TOTAL_FRAMES = 0;
CvFont FONT;

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

	int compareFeatures(trackingFeature_t** a, trackingFeature_t** b)
	{
		return (*b)->lastUpdated - (*a)->lastUpdated; 
	}

	void reorderFeatures()
	{
		qsort(FEATURE_LIST_SORTED, MAX_CORNERS, sizeof(trackingFeature_t*), (int(*)(const void*, const void*))compareFeatures);
	}

	int recordFeature(float x, float y)
	{
		trackingFeature_t* closest = FEATURE_LIST + 0;

		float dist = sqrtf(pow(closest->pos[0] - x, 2) + pow(closest->pos[1] - y, 2));
		for(int i = MAX_CORNERS; i--;){
			trackingFeature_t* f = FEATURE_LIST + i;

			float d = sqrtf(pow(f->pos[0] - x, 2) + pow(f->pos[1] - y, 2));
			if(d < dist && f->lastUpdated != TOTAL_FRAMES){
				dist = d;
				closest = f;
			}
		}

		if(dist < 40){
			closest->strength += (1.0 - closest->strength) * 0.01;
		}
		else{
			closest->key[0] = rng.uniform(64,255);
			closest->key[1] = rng.uniform(64,255);
			closest->strength += (0.0 - closest->strength) * 0.001;
		}

		closest->velocity[0] = LERP((x - closest->pos[0]), closest->velocity[0], 0.9);
		closest->velocity[1] = LERP((y - closest->pos[1]), closest->velocity[1], 0.9);			

		float interp = 0.1;//1 / (dist + 0.1);
		closest->pos[0] = x;//LERP(closest->pos[0], x, interp);
		closest->pos[1] = y;//LERP(closest->pos[1], y, interp);

		closest->dispPos[0] = LERP(x, closest->dispPos[0], interp);
		closest->dispPos[1] = LERP(y, closest->dispPos[1], interp);

		closest->lastUpdated = TOTAL_FRAMES;

		return closest->featureId;
	}

	int process(VideoCapture& capture) {
		int n = 0;
		char filename[200];
		string window_name = "video | q or esc to quit";
		cout << "press space to save a picture. q or esc to quit" << endl;
		namedWindow(window_name); //resizable window;
		Mat frame, frameSmall;

		for (;;) {
			capture >> frame;

			if (frame.empty())
				break;

			cvtColor( frame, frame_grey, COLOR_BGR2GRAY );
			pyrDown( frame_grey, frameSmall, Size( frame.cols/2, frame.rows/2 ));

			/// Parameters for Shi-Tomasi algorithm
			vector<Point2f> corners;
			double qualityLevel = 0.01;
			double minDistance = 1;
			int blockSize = 3;
			bool useHarrisDetector = false;
			double k = 0.4;

			/// Copy the source image
			Mat copy = frame.clone();


			/// Apply corner detection
			goodFeaturesToTrack( frameSmall,
			           corners,
			           MAX_CORNERS,
			           qualityLevel,
			           minDistance,
			           Mat(),
			           blockSize,
			           useHarrisDetector,
			           k );


			/// Draw corners detected
			int r = 5;
			int idIndex = 0;
		 	static float mdx;

			for( int i = 0; i < corners.size(); i++ ){
				recordFeature(corners[i].x, corners[i].y);
			}

			for(int i = MAX_CORNERS; i--;){
				trackingFeature_t* feature =  FEATURE_LIST + i;
				float* pos = feature->dispPos;
				float str = feature->strength;

				circle(
					copy,
						Point(pos[0] * 2, pos[1] * 2),
						r * str * str,
						Scalar(0, str * feature->key[0], str * feature->key[1]),
						-1,
						8, 
						0 
				);

				line(
					copy,
					Point(pos[0] * 2, pos[1] * 2),
					Point((pos[0] + feature->velocity[0] * 10 * str) * 2, (pos[1] + feature->velocity[1] * 10 * str) * 2),
					Scalar(0, str * feature->key[0], str * feature->key[1]),
					1, 
					8,
					0
				);

					if(feature->velocity[0] > mdx){
						mdx = feature->velocity[0];
						cout << "Max " << mdx << "\n";
					}

					char buf[8];
					sprintf(buf, "id %d", feature->featureId);

					// if(str * str > 0.8)
					putText(
						copy,
						buf,
						Point(pos[0] * 2 - 5, pos[1] * 2 - 5),
						FONT_HERSHEY_SIMPLEX,
						0.25 * str * str,
						Scalar(0, str * feature->key[0], str * feature->key[1]),
						1,
						8
					);

			}

			char buf[128] = {};
			sprintf(buf, "Frames: %u", TOTAL_FRAMES++);

			putText(
				copy,
				buf,
				Point(10, 20),
				FONT_HERSHEY_SIMPLEX,
				0.5,
				Scalar(255, 255, 255), 
				1,
				8
			);

			done: imshow(window_name, copy);
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

	cvInitFont(&FONT, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1, 4);

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

	w /= 2;
	h /= 2;

	FEATURE_MAP_DIMS[0] = w;
	FEATURE_MAP_DIMS[1] = h;

	FEATURE_MAP = (trackingCell_t**)calloc(sizeof(trackingCell_t*), w);
	for(int i = w; i--;){
		FEATURE_MAP[i] = (trackingCell_t*)calloc(sizeof(trackingCell_t), h);
	}

	for(int i = MAX_CORNERS; i--;){
		FEATURE_LIST[i].featureId = i;
		FEATURE_LIST_SORTED[i] = FEATURE_LIST + i;
	}

	return process(capture);
}
