#include <iostream>	// for standard I/O
#include <fstream>

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

using namespace cv;
using namespace LogConfigTime;
using namespace smeyel;

//#include "FsmLearner.h"

char *configfilename = "default.ini";
MyConfigManager configmanager;

vector<string> inputValueNames(7);

Mat *src;
Mat *lut;
Mat *score;
Mat *visLut;

void processImage(const char *imageFileName, const char *maskFileName, LutColorFilter *filter, ImageTransitionStat *stat, bool verboseAllImages=false)
{
	Mat image = imread(imageFileName);
	Mat mask = imread(maskFileName);
	
	if (mask.channels()==3)
	{
		cvtColor(mask, mask, CV_BGR2GRAY);
	}

	cout << "Processing file: " << imageFileName << " and " << maskFileName << ", size=" << image.cols << " x " << image.rows << endl;

	Mat lut(image.rows,image.cols,CV_8UC1);

	filter->Filter(&image,&lut,NULL);

	if (verboseAllImages)
	{
		Mat visLut(image.rows,image.cols,CV_8UC3);
		filter->InverseLut(lut,visLut);
		imshow("Teaching Image",image);
		imshow("Teaching LUT Image",visLut);
		imshow("Teaching Mask",mask);
		waitKey(0);
	}

	stat->addImageWithMask(lut,mask);
}


Point lastMouseClickLocation;
unsigned int lastLutIdx;
DefaultLutColorFilter *lutColorFilter;

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
		cout << "   LUT value: (" << (int)colorCode << ") " << inputValueNames[colorCode] << endl;
		lastLutIdx = lutColorFilter->rgb2idx(rOrig,gOrig,bOrig);
	}
}


void CalibrateLut(Mat &src)
{
	LutCalibrationPattern area;
	if (!area.findInImage(src))
	{
		return;	// Not found...
	}
	//area.draw(src,Scalar(100,255,100));

	Mat normalizedImage;
	area.compensatePerspectiveTransform(src,normalizedImage);

	// Debug visualization
	Mat verboseImage;
	normalizedImage.copyTo(verboseImage);
	Tetragon subarea;
	for(int row=0; row<2; row++)
	{
		for(int col=0; col<3; col++)
		{
			area.getSubArea(row,col,subarea);
			subarea.draw(verboseImage,Scalar(255,100,100));
		}
	}
	namedWindow( "Normalized (verbose)", 1 );
    imshow( "Normalized (verbose)", verboseImage );

	area.updateLUT(*lutColorFilter,normalizedImage);
}


void test_learnFromImagesAndMasks(const int firstFileIndex, const int lastFileIndex, const char *configFileName = NULL)
{
	if (configFileName != NULL)
	{
		// INI file is given as command line parameter
		strcpy(configfilename,configFileName);
	}
	// Setup config management
	configmanager.init(configfilename);

	int markovChainOrder = 100;

	inputValueNames[COLORCODE_BLK]=string("BLK");
	inputValueNames[COLORCODE_WHT]=string("WHT");
	inputValueNames[COLORCODE_RED]=string("RED");
	inputValueNames[COLORCODE_GRN]=string("GRN");
	inputValueNames[COLORCODE_BLU]=string("BLU");
	inputValueNames[COLORCODE_NONE]=string("NON");

	Logger *logger = new StdoutLogger();
	//Logger *logger = new FileLogger("log.txt");
	logger->SetLogLevel(Logger::LOGLEVEL_WARNING);
	//logger->SetLogLevel(Logger::LOGLEVEL_VERBOSE);

	lutColorFilter = new DefaultLutColorFilter();
	if (configmanager.loadLutAtStartup)
	{
		cout << "Loading LUT from " << configmanager.lutFile << endl;
		lutColorFilter->load(configmanager.lutFile.c_str());
	}

	ImageTransitionStat *stat = new ImageTransitionStat(8,markovChainOrder, configmanager.runLengthTransformFile.c_str());

	for(int fileIndex=firstFileIndex; fileIndex<=lastFileIndex; fileIndex++)
	{
		char imageFileName[128];
		char maskFileName[128];
		sprintf(imageFileName,"image%d.jpg",fileIndex);
		sprintf(maskFileName,"mask%d.jpg",fileIndex);
		processImage(imageFileName,maskFileName,lutColorFilter,stat,false);
	}

	// Fix dataset imbalances in the counter values
	stat->balanceCounter(COUNTERIDX_ON, COUNTERIDX_OFF, false);

	// Set precisions
	PixelPrecisionCalculator precisionCalculator(COUNTERIDX_ON,COUNTERIDX_OFF);
	precisionCalculator.setPrecisionStatus(stat->counterTreeRoot,0.9F);	// TODO 0.9F !!!

	cout << "Current created SequenceCounterTreeNode number: " << SequenceCounterTreeNode::getSumCreatedNodeNumber() << endl;

	PixelScoreImageTransform scoreTransform(stat);

	// -------------- Now start camera and apply statistics (auxScore mask) to the frames
	CameraLocalProxy *camProxy0 = new CameraLocalProxy(VIDEOINPUTTYPE_PS3EYE,0);
	camProxy0->getVideoInput()->SetNormalizedExposure(-1);
	camProxy0->getVideoInput()->SetNormalizedGain(-1);
	camProxy0->getVideoInput()->SetNormalizedWhiteBalance(-1,-1,-1);

	namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	namedWindow("LUT", CV_WINDOW_AUTOSIZE);
	namedWindow("Score", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("LUT", mouse_callback);
	cvSetMouseCallback("Score", mouse_callback);

	src = new Mat(480,640,CV_8UC3);
	lut = new Mat(480,640,CV_8UC1);
	score = new Mat(480,640,CV_8UC1);
	visLut = new Mat(480,640,CV_8UC3);

	bool running=true;
	bool capture=true;
	while(running) //Show the image captured in the window and repeat
	{
		if (capture)
		{
			camProxy0->CaptureImage(0,src);
		}
		lutColorFilter->Filter(src,lut,NULL);	// LUT may be changed...
		scoreTransform.TransformImage(*lut,*score);
		lutColorFilter->InverseLut(*lut,*visLut);	// May be changed at mouse clicks

		imshow("SRC",*src);
		imshow("LUT",*visLut);
		imshow("Score",*score);

		char ch = waitKey(25);
		unsigned char newLutValue;
		switch(ch)
		{
		case 27:
			running=false;
			break;
		case 't':	// TEST LUT calibration
			CalibrateLut(*src);
			break;
		case 's':	// stop
			capture=false;
			break;
		case 'r':	// run
			capture=true;
			break;
		case 'v':	// verbose
			stat->verboseScoreForImageLocation(*lut,lastMouseClickLocation);
			break;
		case 'c':	// Lut change	
			cout << "LUT modification for idx " << lastLutIdx << endl;
			for(unsigned int i=0; i<inputValueNames.size(); i++)
			{
				cout << "   code " << i << ": " << inputValueNames[i] << endl;
			}
			cout << "Choose by pressing the corresponding number." << endl;
			newLutValue = waitKey(0) - '0';
			lutColorFilter->setLutItemByIdx(lastLutIdx,newLutValue);
			cout << "LUT[" << lastLutIdx << "] = " << (int)newLutValue << endl;
			break;
		case 'w':	// Write LUT to file
			lutColorFilter->save(configmanager.lutFile.c_str());
			cout << "LUT saved to " << configmanager.lutFile << endl;
			break;
		case 'l':	// Load LUT from file
			lutColorFilter->load(configmanager.lutFile.c_str());
			cout << "LUT loaded from " << configmanager.lutFile << endl;
			break;
		}
	}
}
