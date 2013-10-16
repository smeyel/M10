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

#include "area.h"

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
			fTau = 0.6F;	// Not shadow if darker than background*fTau (?)
		}
};

bool isIntersecting(cvb::CvTrack *track, Area *area)
{
	Rect rect(track->minx, track->miny, track->maxx - track->minx, track->maxy - track->miny);
	return area->isRectangleIntersecting(rect);
}

void processTracks(cvb::CvTracks *tracks, std::vector<Area> *areas)
{
	cout << "#CvTracks: " << tracks->size() << endl;

	cvb::CvTracks::const_iterator it;
	for (it = tracks->begin(); it != tracks->end(); ++it)
	{
		for(unsigned int areaIdx=0; areaIdx<areas->size(); areaIdx++)
		{
			if (isIntersecting(it->second, &(*areas)[areaIdx]))
			{
				cout << (it->second->inactive ? "inactive " : "  active ");
				cout << "CAR " << it->second->id << " in AREA " << (*areas)[areaIdx].id << endl;
			}
		}
	}
}

void test_BlobOnForeground(const char *overrideConfigFileName = NULL)
{
	init_defaults(overrideConfigFileName);

	std::vector<Area> areas;
	Area::loadAreaList(configmanager.areaInputFilename.c_str(), &areas);

	vector<cvb::CvTracks> currentTrackList;

	CvBlobWrapper *cvblob = new CvBlobWrapper();
	cvblob->minBlobArea = configmanager.minBlobArea;
	cvblob->maxBlobArea = configmanager.maxBlobArea;

	src = new Mat(480,640,CV_8UC3);
	Mat *backgroundFrame = new Mat(480,640,CV_8UC1);
	Mat *foregroundFrame = new Mat(480,640,CV_8UC1);
	Mat *result = new Mat(480,640,CV_8UC3);
	Mat *blurredSrc = new Mat(480,640,CV_8UC3);

	if (configmanager.showSRC)
	{
		namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	}
	if (configmanager.showBACK)
	{
		namedWindow("BACK", CV_WINDOW_AUTOSIZE);
	}
	if (configmanager.showFORE)
	{
		namedWindow("FORE", CV_WINDOW_AUTOSIZE);
	}
	namedWindow("RES", CV_WINDOW_AUTOSIZE);

	MyBackgroundSubtractor *backgroundSubtractor = new MyBackgroundSubtractor();

    std::vector<std::vector<cv::Point> > contours;

	Mat openKernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));

	bool finish = false;
	while (!finish && camProxy->CaptureImage(0,src))
	{
		cv::blur(*src,*blurredSrc,cv::Size(10,10));
		src->copyTo(*blurredSrc);

		backgroundSubtractor->operator()(*blurredSrc,*foregroundFrame);

		morphologyEx(*foregroundFrame,*foregroundFrame,MORPH_OPEN,openKernel,Point(-1,-1),1);

		morphologyEx(*foregroundFrame,*foregroundFrame,MORPH_CLOSE,openKernel,Point(-1,-1),1);

		if (configmanager.showFORE)
		{
			imshow("FORE",*foregroundFrame);
		}

		for(unsigned int i=0; i<areas.size(); i++)
		{
			areas[i].draw(src,Scalar(0,255,0),false);
		}

		if (configmanager.showSRC)
		{
			imshow("SRC",*src);
		}

		if (configmanager.showBACK)
		{
			// Just for couriosity...
			backgroundSubtractor->getBackgroundImage(*backgroundFrame);
			imshow("BACK",*backgroundFrame);
		}

		// Tracking blobs
		src->copyTo(*result);
		cvblob->findWhiteBlobs(foregroundFrame,result);

		processTracks(cvblob->getCvTracks(),&areas);

		imshow("RES", *result);

		char k = cvWaitKey(25);
		switch (k)
		{
		case -1:	// No keypress
			break;
		case 27:
			finish = true;
			break;
		default:
			cout << "Press ESC to exit." << endl;
			break;
		}
	}
}

int main(int argc, char *argv[], char *window_name)
{
	test_BlobOnForeground();
}
