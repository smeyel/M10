#include <iostream>	// for standard I/O
#include <string>   // for strings

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraRemoteProxy.h"
#include "CameraLocalProxy.h"

using namespace std;
using namespace cv;

void help()
{
	cout
		<< "\n--------------------------------------------------------------------------" << endl
		<< "This program captures video from local camera to file."
		<< "--------------------------------------------------------------------------"   << endl
		<< endl;
}

// Meant to record video from multiple cameras. Later, should be able to use CameraProxy and save images with timestamp for proper re-playing.
// Will support even multiple local and/or multiple remote cameras.
// Later todo: make a component like logging where the measurement host can push the just retrieved images and their timestamps.
// Record to multiple files by first capturing into memory and then saving into AVI at once. (Pre-allocate many buffer Mat-s)
int main(int argc, char *argv[], char *window_name)
{
	CameraProxy *camProxy0 = new CameraLocalProxy(VIDEOINPUTTYPE_PS3EYE,0);
	CameraProxy *camProxy1 = new CameraLocalProxy(VIDEOINPUTTYPE_PS3EYE,1);

/*	cout << "Waiting 3s..." << endl;
#ifdef WIN32
	Sleep(3000);
#else
#error TODO: Sleep not implemented for non-Win32.
#endif*/

	/*CameraRemoteProxy *camProxy0 = new CameraRemoteProxy();
	camProxy0->Connect("127.0.0.1",6000);*/
	/*CameraRemoteProxy *camProxy1 = new CameraRemoteProxy();
	camProxy1->Connect("127.0.0.1",6001);*/

	/*Size S = Size((int) inputVideo.get(CV_CAP_PROP_FRAME_WIDTH),    //Acquire input size
				  (int) inputVideo.get(CV_CAP_PROP_FRAME_HEIGHT));*/
	Size S = Size(640,480);

	const string target = "test.avi";//argv[2];            // the target file name
	VideoWriter outputVideo;                                        // Open the output
	outputVideo.open(string(target), CV_FOURCC('M','J','P','G'), 25.0 ,S, true);

	if (!outputVideo.isOpened())
	{
		cout  << "Could not open the output video for write: " << target << endl;
		return -1;
	}

//	union { int v; char c[5];} uEx ;

/*	cout << "Input frame resolution: Width=" << S.width << "  Height=" << S.height
		<< " of nr#: " << inputVideo.get(CV_CAP_PROP_FRAME_COUNT) << endl;*/

	namedWindow("Video 0", CV_WINDOW_AUTOSIZE);
	//namedWindow("Video 1", CV_WINDOW_AUTOSIZE);

	Mat src0(480,640,CV_8UC4);
	Mat src1(480,640,CV_8UC4);
	int framecounter = 0;	// 0-based frame counter
	while(true) //Show the image captured in the window and repeat
	{
		camProxy0->CaptureImage(0,&src0);
		camProxy1->CaptureImage(0,&src1);

		imshow("Video 0",src0);
		imshow("Video 1",src1);

		cout << "Frame " << framecounter << endl;

		//outputVideo << src;

		framecounter++;

		char ch = waitKey(25);
		if (ch==27)
		{
			break;
		}
	}

	//camProxy1->Disconnect();

	cout << endl << "Capture complete." << endl;
	return 0;
}
