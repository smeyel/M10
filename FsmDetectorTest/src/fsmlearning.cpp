#include <iostream>	// for standard I/O
#include <fstream>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraLocalProxy.h"

#include "MyLutColorFilter.h"
#include "StdoutLogger.h"
#include "FileLogger.h"

#include "myconfigmanager.h"

#include "ImageTransitionStat.h"
#include "PixelPrecisionCalculator.h"
#include "PixelScoreImageTransform.h"

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

void processImage(const char *imageFileName, const char *maskFileName, LutColorFilter *filter, ImageTransitionStat *stat)
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

int findMinIdx(vector<Vec3f> marks, float coeffX, float coeffY, float offset)
{
	int minIdx = 0;
	float minValue = coeffX*marks[0][0] + coeffY*marks[0][1] + offset;
	for(int i=1; i<marks.size(); i++)
	{
		float currentValue = coeffX*marks[i][0] + coeffY*marks[i][1] + offset;
		if (currentValue < minValue)
		{
			minIdx = i;
			minValue = currentValue;
		}
	}
	return minIdx;
}

class RectangularArea
{
public:
	Point2f corners[4];	// Upper left, upper right, lower left, lower right

	void draw(Mat &img, Scalar color)
	{
		line(img, corners[0], corners[1], color,3);
		line(img, corners[1], corners[3], color,3);
		line(img, corners[3], corners[2], color,3);
		line(img, corners[2], corners[0], color,3);
	}

	void getPolyline(Point *targetOf4Points)
	{
		for(int i=0; i<4; i++)
		{
			targetOf4Points[i].x = cvRound(this->corners[i].x);
			targetOf4Points[i].y = cvRound(this->corners[i].y);
		}
	}

/*	// Modifies corners!
	void compensatePerspectiveTransform(Mat &img)
	{

	}*/
};

class CalibrationArea : public RectangularArea
{
public:
	// Unit vectors from upper left corner
	Point2f unitHorizontal;
	Point2f unitVertical;

	void setFromCircleVector(vector<Vec3f> circles)
	{
		Vec3f marks[4];
		// Reoder the circles in predefined order (upper left, upper right, lower left, lower right)
		int idx = findMinIdx(circles,1,1,0);	// arg min ( x + y )
		marks[0] = circles[idx];
		idx = findMinIdx(circles,-1,1,640);	// arg min ( 640-x + y )
		marks[1] = circles[idx];
		idx = findMinIdx(circles,1,-1,480);	// arg min ( x + 480-y )
		marks[2] = circles[idx];
		idx = findMinIdx(circles,-1,-1,1120);	// arg min ( 640-x + 480-y )
		marks[3] = circles[idx];

		corners[0] = Point2f( marks[0][0]+marks[0][2],marks[0][1]+marks[0][2] );
		corners[1] = Point2f( marks[1][0]-marks[1][2],marks[1][1]+marks[1][2] );
		corners[2] = Point2f( marks[2][0]+marks[2][2],marks[2][1]-marks[2][2] );
		corners[3] = Point2f( marks[3][0]-marks[3][2],marks[3][1]-marks[3][2] );

		unitHorizontal.x = (corners[1].x - corners[0].x) / 21.0F;
		unitHorizontal.y = (corners[1].y - corners[0].y) / 21.0F;

		unitVertical.x = (corners[2].x - corners[0].x) / 18.0F;
		unitVertical.y = (corners[2].y - corners[0].y) / 18.0F;
	}

	void getSubArea(int row, int col, RectangularArea &subArea)
	{
		// Get upper left corner
		Point2f UL = this->corners[0] + this->unitHorizontal + this->unitVertical
			+ col*7*this->unitHorizontal + row*10*this->unitVertical;
		Point2f UR = UL + 5*this->unitHorizontal;
		Point2f LL = UL + 6*this->unitVertical;
		Point2f LR = LL + 5*this->unitHorizontal;

		subArea.corners[0] = UL;
		subArea.corners[1] = UR;
		subArea.corners[2] = LL;
		subArea.corners[3] = LR;
	}

	void getPolylineForSubArea(int row, int col, Point *targetOf4Points)
	{
		RectangularArea subarea;
		getSubArea(row,col,subarea);
		for(int i=0; i<4; i++)
		{
			targetOf4Points[i].x = cvRound(subarea.corners[i].x);
			targetOf4Points[i].y = cvRound(subarea.corners[i].y);
		}
	}
};


void CalibrateLut(Mat &src)
{
	OPENCV_ASSERT(src.cols==640 && src.rows==480,"CalibrateLut", "Source image is not 640x480...");
	OPENCV_ASSERT(src.type() == CV_8UC3,"CalibrateLut", "Source image is not CV_8UC3...");
	// Find circles
	Mat grayImage;
	cvtColor(src, grayImage, CV_BGR2GRAY);
    GaussianBlur( grayImage, grayImage, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;
    HoughCircles(grayImage, circles, CV_HOUGH_GRADIENT,
                 1,	// full recolution accumulator
				 grayImage.rows/4,	// minimal distance between the centers of detected circles
				 100,25);
    for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
         // draw the circle center
         circle( src, center, 3, Scalar(0,255,0), -1, 8, 0 );
         // draw the circle outline
         circle( src, center, radius, Scalar(0,0,255), 3, 8, 0 );
    }

//	namedWindow( "Circles", 1 );
//    imshow( "Circles", src );
	if (circles.size() != 4)
	{
		cout << "Not 4 circles were found... (" << circles.size() << ")" << endl;
		return;
	}

	CalibrationArea area;
	area.setFromCircleVector(circles);
	area.draw(src,Scalar(100,255,100));
	RectangularArea subarea;
	for(int row=0; row<2; row++)
	{
		for(int col=0; col<3; col++)
		{
			area.getSubArea(row,col,subarea);
			subarea.draw(src,Scalar(255,100,100));
		}
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

	ImageTransitionStat *stat = new ImageTransitionStat(8,markovChainOrder, configmanager.runLengthTransformFile.c_str());

	for(int fileIndex=firstFileIndex; fileIndex<=lastFileIndex; fileIndex++)
	{
		char imageFileName[128];
		char maskFileName[128];
		sprintf(imageFileName,"image%d.jpg",fileIndex);
		sprintf(maskFileName,"mask%d.jpg",fileIndex);
		processImage(imageFileName,maskFileName,lutColorFilter,stat);
	}

	// Fix dataset imbalances in the counter values
	stat->balanceCounter(COUNTERIDX_ON, COUNTERIDX_OFF, false);

	// Set precisions
	PixelPrecisionCalculator precisionCalculator(COUNTERIDX_ON,COUNTERIDX_OFF);
	precisionCalculator.setPrecisionStatus(stat->counterTreeRoot,0.7F);	// TODO 0.9F !!!

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
