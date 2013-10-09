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

// Entry point of this module, called from main()
void test_bkgndRemove(const int firstFileIndex, const int lastFileIndex, const char *overrideConfigFileName = NULL)
{
	init_defaults(overrideConfigFileName);

	int markovChainOrder = 100;

	// Create LUT Color filter, load from file
	lutColorFilter = new DefaultLutColorFilter();
	if (configmanager.loadLutAtStartup)
	{
		cout << "Loading LUT from " << configmanager.lutFile << endl;
		lutColorFilter->load(configmanager.lutFile.c_str());
	}

	namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	namedWindow("LUT", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("SRC", mouse_callback);
	cvSetMouseCallback("LUT", mouse_callback);

	src = new Mat(480,640,CV_8UC3);
	Mat *lut = new Mat(480,640,CV_8UC1);
	Mat *foreground = new Mat(480,640,CV_8UC1);
	Mat *visLut = new Mat(480,640,CV_8UC3);

	DetectionBoundingBoxCollector collector(new SimpleBoundingBoxValidator());

	// ------------------- Main loop (using keyboard commands) -----------------------
	bool running=true;
	bool capture=true;
	while(running) //Show the image captured in the window and repeat
	{
		if (capture)
		{
			camProxy->CaptureImage(0,src);
		}
		lutColorFilter->Filter(src,lut,NULL);	// LUT may be changed...
		lutColorFilter->InverseLut(*lut,*visLut);	// May be changed at mouse clicks

		imshow("SRC",*src);
		imshow("LUT",*visLut);

		char ch = waitKey(25);
		unsigned char newLutValue;
		switch(ch)
		{
		case 27:
			running=false;
			break;
		case 'c':	// Lut change	
			cout << "LUT modification for idx " << lastLutIdx << endl;
			lutColorFilter->ShowColorcodeNames(cout);
			cout << "Choose by pressing the corresponding number." << endl;
			newLutValue = waitKey(0) - '0';
			lutColorFilter->setLutItemByIdx(lastLutIdx,newLutValue);
			cout << "LUT[" << lastLutIdx << "] = " << (int)newLutValue << endl;
			break;
		case 'w':	// Write LUT to file
			lutColorFilter->save(configmanager.lutFile.c_str());
			cout << "LUT saved to " << configmanager.lutFile << endl;
			break;
		case 'r':	// Read LUT from file
			lutColorFilter->load(configmanager.lutFile.c_str());
			cout << "LUT loaded from " << configmanager.lutFile << endl;
			break;
		}
	}
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

int main(int argc, char *argv[], char *window_name)
{
	//test_bkgndRemove(0,14,NULL);
	test_BackgroundSubtractor();
}
