#include <iostream>	// for standard I/O
#include <fstream>

#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
 
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraLocalProxy.h"

#include "DefaultLutColorFilter.h"
#include "StdoutLogger.h"
#include "FileLogger.h"

#include "myconfigmanager.h"

#include "ImageTransitionStat.h"
#include "PixelPrecisionCalculator.h"
#include "PixelScoreImageTransform.h"

#include "LutCalibrationPattern.h"

#include "DetectionBoundingBoxCollector.h"
#include "SimpleBoundingBoxValidator.h"

// BEGIN for test_CvBlobLib
#include <iostream>
#include <iomanip>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cvblob.h>
using namespace cvb;
// END for test_CvBlobLib

using namespace cv;
using namespace LogConfigTime;
using namespace smeyel;

char *configfilename = "default.ini";
MyConfigManager configmanager;
CameraLocalProxy *camProxy = NULL;
Mat *src;

// Globals used by mouse event handler
Point lastMouseClickLocation;
unsigned int lastLutIdx;
DefaultLutColorFilter *lutColorFilter;


// Mouse callback to retrieve debug information at pixel locations
void mouse_callback(int eventtype, int x, int y, int flags, void *param)
{
	if (eventtype == CV_EVENT_LBUTTONDOWN)
	{
		lastMouseClickLocation = Point(x,y);
		cout << "Click location saved: " << x << ";" << y << endl;

		Vec3b intensity;
		uchar bOrig, gOrig, rOrig;
		uchar bNew,gNew,rNew;
		uchar colorCode;

		intensity = src->at<Vec3b>(lastMouseClickLocation.y, lastMouseClickLocation.x);
		bOrig = intensity.val[0];
		gOrig = intensity.val[1];
		rOrig = intensity.val[2];
		lutColorFilter->quantizeRgb(rOrig,gOrig,bOrig,rNew,gNew,bNew);
		colorCode = lutColorFilter->rgb2lutValue(rOrig,gOrig,bOrig);
		cout << "Pixel data:" << endl;
		cout << "   real RGB: " << (int)rOrig << "," << (int)gOrig << "," << (int)bOrig << endl;
		cout << "   quantizedRGB: " << (int)rNew << "," << (int)gNew << "," << (int)bNew << endl;
		cout << "   LUT value: (" << (int)colorCode << ") " << lutColorFilter->GetColorcodeName(colorCode) << endl;
		lastLutIdx = lutColorFilter->rgb2idx(rOrig,gOrig,bOrig);
	}
}

void init_defaults(const char *overrideConfigFileName = NULL)
{
	if (overrideConfigFileName != NULL)
	{
		// INI file is given as command line parameter
		strcpy(configfilename,overrideConfigFileName);
	}
	// Setup config management
	configmanager.init(configfilename);

	Logger *logger = new StdoutLogger();
	logger->SetLogLevel(Logger::LOGLEVEL_WARNING);

	if (configmanager.videoInputFileOverride)
	{
		camProxy = new CameraLocalProxy(configmanager.videoInputFilename.c_str());
	}
	else
	{
		camProxy = new CameraLocalProxy(VIDEOINPUTTYPE_PS3EYE,0);
	}
	camProxy->getVideoInput()->SetNormalizedExposure(-1);
	camProxy->getVideoInput()->SetNormalizedGain(-1);
	camProxy->getVideoInput()->SetNormalizedWhiteBalance(-1,-1,-1);

	src = new Mat(480,640,CV_8UC3);
}

void test_BackgroundSubtractor(const char *overrideConfigFileName = NULL)
{
	init_defaults(overrideConfigFileName);

	namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	namedWindow("BACK", CV_WINDOW_AUTOSIZE);
	namedWindow("FORE", CV_WINDOW_AUTOSIZE);

	src = new Mat(480,640,CV_8UC3);
	Mat *backgroundFrame = new Mat(480,640,CV_8UC1);
	Mat *foregroundFrame = new Mat(480,640,CV_8UC1);

    BackgroundSubtractorMOG2 *backgroundSubtractor = new BackgroundSubtractorMOG2(10,16,false);
 
    std::vector<std::vector<cv::Point> > contours;

	bool finished = false;
	while (!finished)
	{
		if (!camProxy->CaptureImage(0,src))
		{
			finished=true;
			break;
		}

		backgroundSubtractor->operator()(*src,*foregroundFrame);
		backgroundSubtractor->getBackgroundImage(*backgroundFrame);
        erode(*foregroundFrame,*foregroundFrame,cv::Mat());
        dilate(*foregroundFrame,*foregroundFrame,cv::Mat());
        findContours(*foregroundFrame,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        drawContours(*src,contours,-1,cv::Scalar(0,0,255),2);
        imshow("SRC",*src);
        imshow("BACK",*backgroundFrame);
        imshow("FORE",*foregroundFrame);

		char ch = waitKey(25);
		switch (ch)
		{
		case 27:
			finished=true;
			break;
		}
	}
}

void test_CvBlobLib()
{
	CvTracks tracks;

	cvNamedWindow("red_object_tracking", CV_WINDOW_AUTOSIZE);

	CvCapture *capture = cvCaptureFromCAM(0);
	cvGrabFrame(capture);
	IplImage *img = cvRetrieveFrame(capture);

	CvSize imgSize = cvGetSize(img);

	IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);

	IplConvKernel* morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);

	//unsigned int frameNumber = 0;
	unsigned int blobNumber = 0;

	bool quit = false;
	while (!quit&&cvGrabFrame(capture))
	{
		IplImage *img = cvRetrieveFrame(capture);

		cvConvertScale(img, frame, 1, 0);

		IplImage *segmentated = cvCreateImage(imgSize, 8, 1);
    
		// Detecting red pixels:
		// (This is very slow, use direct access better...)
		for (unsigned int j=0; j<imgSize.height; j++)
			for (unsigned int i=0; i<imgSize.width; i++)
			{
				CvScalar c = cvGet2D(frame, j, i);

				double b = ((double)c.val[0])/255.;
				double g = ((double)c.val[1])/255.;
				double r = ((double)c.val[2])/255.;
				unsigned char f = 255*((r>0.2+g)&&(r>0.2+b));

				cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
			}

		cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN, 1);

		cvShowImage("segmentated", segmentated);

		IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

		CvBlobs blobs;
		unsigned int result = cvLabel(segmentated, labelImg, blobs);
		cvFilterByArea(blobs, 500, 1000000);
		cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
		cvUpdateTracks(blobs, tracks, 200., 5);
		cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

		cvShowImage("red_object_tracking", frame);

		/*std::stringstream filename;
		filename << "redobject_" << std::setw(5) << std::setfill('0') << frameNumber << ".png";
		cvSaveImage(filename.str().c_str(), frame);*/

		cvReleaseImage(&labelImg);
		cvReleaseImage(&segmentated);

		char k = cvWaitKey(10)&0xff;
		switch (k)
		{
			case 27:
			case 'q':
			case 'Q':
			quit = true;
			break;
			case 's':
			case 'S':
			for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
			{
				std::stringstream filename;
				filename << "redobject_blob_" << std::setw(5) << std::setfill('0') << blobNumber << ".png";
				cvSaveImageBlob(filename.str().c_str(), img, it->second);
				blobNumber++;

				std::cout << filename.str() << " saved!" << std::endl;
			}
			break;
		}

		cvReleaseBlobs(blobs);

		//frameNumber++;
	}

	cvReleaseStructuringElement(&morphKernel);
	cvReleaseImage(&frame);

	cvDestroyWindow("red_object_tracking");
}

int main(int argc, char *argv[], char *window_name)
{
	//test_BackgroundSubtractor();
	test_CvBlobLib();
}
