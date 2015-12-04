#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <unistd.h>

int main()
{
	CvCapture* cap = NULL;
	IplImage *frame;

	cap = cvCreateCameraCapture(0);	
	assert(cap);

	// set capture size
	cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH,  320);
	cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, 240);
	
	while(1){
		frame = cvQueryFrame(cap);
		
		if(!frame) continue;

		
	}

	cvReleaseCapture(&cap);
	return 0;
}
