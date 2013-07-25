#include <iostream>	// for standard I/O
#include <fstream>
#include <string>   // for strings

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraRemoteProxy.h"
#include "CameraLocalProxy.h"

using namespace std;
using namespace cv;

Rect maskRect;
Point lastMouseClickLocation;

void mouse_callback(int eventtype, int x, int y, int flags, void *param)
{
	if (eventtype == CV_EVENT_LBUTTONDOWN)
	{
		lastMouseClickLocation = Point(x,y);
		cout << "Click location saved: " << x << ";" << y << endl;
	}
}

void updateMask(Mat &srcOriginal, Mat &srcAnnotated, Mat &mask)
{
	srcOriginal.copyTo(srcAnnotated);
	mask.setTo(Scalar(0));
	rectangle(srcAnnotated,maskRect,Scalar(0,255,0));
	rectangle(mask,maskRect,Scalar(255),CV_FILLED);
}


// Meant to record video from multiple cameras. Later, should be able to use CameraProxy and save images with timestamp for proper re-playing.
// Will support even multiple local and/or multiple remote cameras.
// Later todo: make a component like logging where the measurement host can push the just retrieved images and their timestamps.
// Record to multiple files by first capturing into memory and then saving into AVI at once. (Pre-allocate many buffer Mat-s)
int main(int argc, char *argv[], char *window_name)
{
	CameraProxy *camProxy = new CameraLocalProxy(VIDEOINPUTTYPE_GENERIC,0);

	Size S = Size(640,480);

	namedWindow("Image", CV_WINDOW_AUTOSIZE);
	namedWindow("Mask", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("Image", mouse_callback);
	cvSetMouseCallback("Mask", mouse_callback);

	Mat srcOriginal(480,640,CV_8UC4);
	Mat srcAnnotated(480,640,CV_8UC4);
	Mat mask(480,640,CV_8UC1);
	srcOriginal.setTo(Scalar(100,100,100));
	srcAnnotated.setTo(Scalar(100,100,100));
	mask.setTo(Scalar(0));

	int fileCounter=0;
	bool running = true;
	while(running)
	{
		imshow("Image",srcAnnotated);
		imshow("Mask",mask);

		char ch = waitKey(25);
		switch(ch)
		{
		case 27:
			running=false;
			break;
		case 'c':	// Capture
			camProxy->CaptureImage(0,&srcOriginal);
			srcOriginal.copyTo(srcAnnotated);
			mask.setTo(Scalar(0));
			cout << "Image captured, mask reset." << endl;
			break;
		case '1':	// upper left corner
			maskRect.x = lastMouseClickLocation.x;
			maskRect.y = lastMouseClickLocation.y;
			updateMask(srcOriginal,srcAnnotated,mask);
			break;
		case '2':	// lower right corner
			maskRect.width = lastMouseClickLocation.x - maskRect.x;
			maskRect.height = lastMouseClickLocation.y - maskRect.y;
			updateMask(srcOriginal,srcAnnotated,mask);
			break;
		case 's':	// save image and mask
			char filename[128];
			sprintf(filename,"image%d.jpg",fileCounter);
			imwrite(filename,srcOriginal);
			sprintf(filename,"mask%d.jpg",fileCounter);
			imwrite(filename,mask);
			fileCounter++;
			cout << "Image and mask saved." << endl;
			break;
		}
	}
	return 0;
}
