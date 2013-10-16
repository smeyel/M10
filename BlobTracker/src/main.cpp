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

#include "cvblobwrapper.h"

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

void test_CvBlobLib(const char *overrideConfigFileName = NULL)
{
	init_defaults(overrideConfigFileName);

	CvBlobWrapper *cvblob = new CvBlobWrapper();

	src = new Mat(480,640,CV_8UC3);
	Mat *result = new Mat(480,640,CV_8UC3);

	namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	namedWindow("RES", CV_WINDOW_AUTOSIZE);

	bool finish = false;
	while (!finish && camProxy->CaptureImage(0,src))
	{
		cvblob->findBlobsInRgb(src,result);

		imshow("SRC", *src);
		imshow("RES", *result);

		char k = cvWaitKey(25);
		switch (k)
		{
			case 27:
				finish = true;
			break;
			break;
		}
	}
}

void drawContourAsPolygon(Mat *img, vector<Point> contour, Scalar color)
{
	// create a pointer to the data as an array of points (via a conversion to 
	// a Mat() object)
	const cv::Point *pts = (const cv::Point*) Mat(contour).data;
	int npts = Mat(contour).rows;

	//std::cout << "Number of polygon vertices: " << npts << std::endl;
	
	// draw the polygon 
	//polylines(*img, &pts,&npts, 1, true, Scalar(0,255,0), 3);
	fillPoly(*img, &pts,&npts, 1, color);
}

class MyBackgroundSubtractor : public BackgroundSubtractorMOG2
{
	public:
		MyBackgroundSubtractor() : BackgroundSubtractorMOG2(100,16,true) 	// Originally hist=10, thres=16
		{
			fTau = 0.8;	// Not shadow if darker than background*fTau (?)
		}

};


void test_BlobOnForeground(const char *overrideConfigFileName = NULL)
{
	init_defaults(overrideConfigFileName);

	CvBlobWrapper *cvblob = new CvBlobWrapper();

	src = new Mat(480,640,CV_8UC3);
	Mat *backgroundFrame = new Mat(480,640,CV_8UC1);
	Mat *foregroundFrame = new Mat(480,640,CV_8UC1);
	Mat *result = new Mat(480,640,CV_8UC3);
	Mat *blurredSrc = new Mat(480,640,CV_8UC3);

	namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	namedWindow("BACK", CV_WINDOW_AUTOSIZE);
	namedWindow("FORE", CV_WINDOW_AUTOSIZE);
	namedWindow("RES", CV_WINDOW_AUTOSIZE);

	MyBackgroundSubtractor *backgroundSubtractor = new MyBackgroundSubtractor();

    std::vector<std::vector<cv::Point> > contours;

	Mat openKernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));

	bool finish = false;
	while (!finish && camProxy->CaptureImage(0,src))
	{
		//cv::blur(*src,*blurredSrc,cv::Size(10,10));
		src->copyTo(*blurredSrc);

		backgroundSubtractor->operator()(*blurredSrc,*foregroundFrame);


		morphologyEx(*foregroundFrame,*foregroundFrame,MORPH_OPEN,openKernel,Point(-1,-1),1);
        //erode(*foregroundFrame,*foregroundFrame,cv::Mat());
        //dilate(*foregroundFrame,*foregroundFrame,cv::Mat());
        imshow("FORE",*foregroundFrame);

//		findContours(*foregroundFrame,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);

//        drawContours(*src,contours,-1,cv::Scalar(0,0,255),2);
		imshow("SRC",*src);

/*		if (contours.size()>0)
		{
			int colorIncrement = 255 / contours.size();
			for(int contourIdx=0; contourIdx<contours.size(); contourIdx++)
			{
				unsigned char color = (contourIdx+1)*colorIncrement;
				drawContourAsPolygon(foregroundFrame, contours[contourIdx], Scalar(color));
			}
		} */


//        imshow("FORE",*foregroundFrame);

		// Just for couriosity...
		backgroundSubtractor->getBackgroundImage(*backgroundFrame);
        imshow("BACK",*backgroundFrame);

		// Tracking blobs
		src->copyTo(*result);
		cvblob->findWhiteBlobs(foregroundFrame,result);
		imshow("RES", *result);

		char k = cvWaitKey(25);
		switch (k)
		{
			case 27:
				finish = true;
			break;
			break;
		}
	}
}

int main(int argc, char *argv[], char *window_name)
{
	//test_BackgroundSubtractor();
	//test_CvBlobLib();
	test_BlobOnForeground();
}
