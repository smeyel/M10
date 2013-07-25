#include <iostream>	// for standard I/O
#include <fstream>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraLocalProxy.h"

#include "MyLutColorFilter.h"
#include "StdoutLogger.h"
#include "FileLogger.h"

#include "myconfigmanager.h"

using namespace cv;
using namespace LogConfigTime;
using namespace smeyel;

#include "FsmLearner.h"

char *configfilename = "default.ini";
MyConfigManager configmanager;

vector<string> inputValueNames(7);

Mat *src;
Mat *lut;
Mat *score;
Mat *visLut;

void test_graphOpt()
{
	Logger *logger = new StdoutLogger();
	logger->SetLogLevel(Logger::LOGLEVEL_WARNING);
	MyLutColorFilter *lutColorFilter = new MyLutColorFilter();
	FsmLearner *fsmlearner = new FsmLearner(8,3,COLORCODE_NONE);

	vector<string> inputValueNames(7);
	inputValueNames[COLORCODE_BLK]=string("BLK");
	inputValueNames[COLORCODE_WHT]=string("WHT");
	inputValueNames[COLORCODE_RED]=string("RED");
	inputValueNames[COLORCODE_GRN]=string("GRN");
	inputValueNames[COLORCODE_BLU]=string("BLU");
	inputValueNames[COLORCODE_NONE]=string("NON");

	Mat src(50,50,CV_8UC3);
	Mat lut(50,50,CV_8UC1);
	Mat lutVis(50,50,CV_8UC3);
	src.setTo(Scalar(0,0,0));
	rectangle(src,Point2d(25,0),Point2d(30,25),Scalar(255,0,0));
	rectangle(src,Point2d(25,26),Point2d(30,49),Scalar(0,255,0));
	lutColorFilter->Filter(&src,&lut,NULL);
	lutColorFilter->InverseLut(lut,lutVis);
	imshow("TestImage",src);
	imshow("LUT",lutVis);
	//waitKey(0);
	fsmlearner->addImage(lut, true);

	//cout << "------------- ON train -------------" << endl;
	//stat->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);

	src.setTo(Scalar(0,0,0));
	rectangle(src,Point2d(25,0),Point2d(30,49),Scalar(0,0,255));
	lutColorFilter->Filter(&src,&lut,NULL);
	lutColorFilter->InverseLut(lut,lutVis);
	imshow("TestImage",src);
	imshow("LUT",lutVis);
	//waitKey(0);
	fsmlearner->addImage(lut, false);

	//cout << "------------- OFF train -------------" << endl;
	//stat->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);

	// calculateSubtreeCounters
	fsmlearner->counterTreeRoot->calculateSubtreeCounters(COUNTERIDX_ON);
	fsmlearner->counterTreeRoot->calculateSubtreeCounters(COUNTERIDX_OFF);

	// Set precisions
	fsmlearner->setPrecisionStatus(fsmlearner->counterTreeRoot,0.7F);

	fsmlearner->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);

	// cut
	cout << "------------- cut -------------" << endl;
	fsmlearner->counterTreeRoot->cut(0);

	fsmlearner->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);

	cout << "------------- merge -------------" << endl;
	fsmlearner->mergeNodesForPrecision(&inputValueNames);

	cout << "------------- combining nodes... -------------" << endl;
	fsmlearner->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);



	cout << "done" << endl;

}

void processImage(const char *imageFileName, const char *maskFileName, LutColorFilter *filter, TransitionStat *stat)
{
	Mat image = imread(imageFileName);
	Mat mask = imread(maskFileName);
	
	if (mask.channels()==3)
	{
		cvtColor(mask, mask, CV_BGR2GRAY);
	}

	cout << "Processing file: " << imageFileName << " and " << maskFileName << ", size=" << image.cols << " x " << image.rows << endl;

	Mat lut(image.rows,image.cols,CV_8UC1);
//	Mat visLut(image.rows,image.cols,CV_8UC3);

	filter->Filter(&image,&lut,NULL);
//	filter->InverseLut(lut,visLut);

/*	imshow("Teaching Image",image);
	imshow("Teaching LUT Image",visLut);
	imshow("Teaching Mask",mask);
	waitKey(0);*/

	stat->addImageWithMask(lut,mask);
}


Point lastMouseClickLocation;
unsigned int lastLutIdx;
MyLutColorFilter *lutColorFilter;

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
		cout << "   LUT value: " << (int)colorCode << ", meaning:" << inputValueNames[colorCode] << endl;
		lastLutIdx = lutColorFilter->rgb2idx(rOrig,gOrig,bOrig);
	}
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

	lutColorFilter = new MyLutColorFilter();
	if (configmanager.loadLutAtStartup)
	{
		cout << "Loading LUT from " << configmanager.lutFile << endl;
		lutColorFilter->load(configmanager.lutFile.c_str());
	}

	FsmLearner *fsmlearner = new FsmLearner(8,markovChainOrder,COLORCODE_NONE);

	for(int fileIndex=firstFileIndex; fileIndex<=lastFileIndex; fileIndex++)
	{
		char imageFileName[128];
		char maskFileName[128];
		sprintf(imageFileName,"image%d.jpg",fileIndex);
		sprintf(maskFileName,"mask%d.jpg",fileIndex);
		processImage(imageFileName,maskFileName,lutColorFilter,fsmlearner);
	}

	fsmlearner->counterTreeRoot->calculateSubtreeCounters(COUNTERIDX_OFF);
	fsmlearner->counterTreeRoot->calculateSubtreeCounters(COUNTERIDX_ON);

	// Fix dataset imbalances in the counter values
	float onSum = fsmlearner->counterTreeRoot->getCounter(COUNTERIDX_ON);
	float offSum = fsmlearner->counterTreeRoot->getCounter(COUNTERIDX_OFF);
	float multiplier = offSum / onSum;
	fsmlearner->counterTreeRoot->multiplySubtreeCounters(COUNTERIDX_ON, multiplier);

	// Optimize tree
	// Set precisions
	fsmlearner->setPrecisionStatus(fsmlearner->counterTreeRoot,0.7F);

	//fsmlearner->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);

	// cut
	cout << "------------- cut -------------" << endl;
	// Warning: a node look-up nem kezeli rendesen, ha CUT-olva van a fa...
	//fsmlearner->counterTreeRoot->cut(0);
	//fsmlearner->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);

	cout << "------------- merge -------------" << endl;
	//fsmlearner->mergeNodesForPrecision(&inputValueNames);
	fsmlearner->setPrecisionStatus(fsmlearner->counterTreeRoot,0.7F);	// re-set precision status and AUX!
	//fsmlearner->counterTreeRoot->showCompactRecursive(0,1,&inputValueNames);

	SequenceCounterTreeNode::showNodeNumber();

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
		fsmlearner->getScoreMaskForImage(*lut,*score);
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
		case 's':	// stop
			capture=false;
			break;
		case 'r':	// run
			capture=true;
			break;
		case 'v':	// verbose
			fsmlearner->verboseScoreForImageLocation(*lut,lastMouseClickLocation);
			break;
		case 'c':	// Lut change	
			cout << "LUT modification for idx " << lastLutIdx << endl;
			for(int i=0; i<inputValueNames.size(); i++)
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
