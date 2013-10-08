#include <iostream>	// for standard I/O
#include <fstream>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraLocalProxy.h"

#include "DefaultLutColorFilter.h"
#include "StdoutLogger.h"
//#include "FileLogger.h"

#include "myconfigmanager.h"

//#include "ImageTransitionStat.h"
//#include "PixelPrecisionCalculator.h"
//#include "PixelScoreImageTransform.h"

//#include "LutCalibrationPattern.h"

#include "LutDiffColorFilter.h"

using namespace cv;
using namespace LogConfigTime;
using namespace smeyel;

extern char *configfilename;
extern MyConfigManager configmanager;

extern Mat *src;
extern Mat *lut;
extern Mat *visLut;

extern Point lastMouseClickLocation;
extern unsigned int lastLutIdx;
extern DefaultLutColorFilter *lutColorFilter;

// Mouse callback to retrieve debug information at pixel locations
void mouse_callback(int eventtype, int x, int y, int flags, void *param);

// Calibration of the LitColorFilter based on the image of a calibration pattern.
void CalibrateLut(Mat &src);

// Entry point of this module, called from main()
void test_foregroundDetector(const char *configFileName = NULL)
{
	if (configFileName != NULL)
	{
		// INI file is given as command line parameter
		strcpy(configfilename,configFileName);
	}
	// Setup config management
	configmanager.init(configfilename);

	Logger *logger = new StdoutLogger();
	logger->SetLogLevel(Logger::LOGLEVEL_WARNING);

	// Create LUT Color filter, load from file
	LutDiffColorFilter *lutDiffColorFilter = new LutDiffColorFilter();
	lutColorFilter = new DefaultLutColorFilter();
	if (configmanager.loadLutAtStartup)
	{
		cout << "Loading LUT from " << configmanager.lutFile << endl;
		lutColorFilter->load(configmanager.lutFile.c_str());
		lutDiffColorFilter->InitInverseLut(0,0,0);
		lutDiffColorFilter->load(configmanager.lutFile.c_str());
	}

	// ------------------- Now start camera and apply statistics (auxScore mask) to the frames -----------------------
	CameraLocalProxy *camProxy0 = NULL;
	if (configmanager.videoInputFileOverride)
	{
		camProxy0 = new CameraLocalProxy(configmanager.videoInputFilename.c_str());
	}
	else
	{
		camProxy0 = new CameraLocalProxy(VIDEOINPUTTYPE_PS3EYE,0);
	}
	
	camProxy0->getVideoInput()->SetNormalizedExposure(-1);
	camProxy0->getVideoInput()->SetNormalizedGain(-1);
	camProxy0->getVideoInput()->SetNormalizedWhiteBalance(-1,-1,-1);

	namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	namedWindow("LUT", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("LUT", mouse_callback);
	cvSetMouseCallback("Score", mouse_callback);

	src = new Mat(480,640,CV_8UC3);
	lut = new Mat(480,640,CV_8UC1);
	visLut = new Mat(480,640,CV_8UC3);
	Mat *background = new Mat(480,640,CV_8UC3);
	Mat *resultMask = new Mat(480,640,CV_8UC1);

	// ------------------- Main loop (using keyboard commands) -----------------------
	bool running=true;
	bool capture=true;
	while(running) //Show the image captured in the window and repeat
	{
		if (capture)
		{
			if (!camProxy0->CaptureImage(0,src))
			{
				cout << "No more images to capture..." << endl;
				running=false;
				break;
			}
		}
		lutColorFilter->Filter(src,lut,NULL);	// LUT may be changed...
		lutColorFilter->InverseLut(*lut,*visLut);	// May be changed at mouse clicks

		lutDiffColorFilter->Filter(background,src,resultMask);

		imshow("SRC",*src);
		imshow("LUT",*visLut);
		imshow("ResultMask",*resultMask);

		char ch = waitKey(25);
		unsigned char newLutValue;
		switch(ch)
		{
		case 27:
			running=false;
			break;
		case 'p':
			capture = !capture;
			break;
		case 'b':
			cout << "Copy current frame to background..." << endl;
			src->copyTo(*background);
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
			cout << "LUT written to " << configmanager.lutFile << endl;
			break;
		case 'r':	// Read LUT from file
			lutColorFilter->load(configmanager.lutFile.c_str());
			cout << "LUT read from " << configmanager.lutFile << endl;
			break;
		}
	}
}
