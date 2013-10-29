#ifndef __SIMPLETRACKINGVIEW_H
#define __SIMPLETRACKINGVIEW_H

#include "SimpleIniConfigReader.h"
#include "TrackingViewBase.h"
#include "SimpleTracker.h"

class SimpleTrackingView : public TrackingViewBase
{
	void drawContourAsPolygon(Mat *img, vector<Point> contour, Scalar color);

	class MyConfigManager
	{
		virtual bool readConfiguration(const char *filename)
		{
			LogConfigTime::SimpleIniConfigReader *SIreader = new LogConfigTime::SimpleIniConfigReader(filename);
			LogConfigTime::ConfigReader *reader = SIreader;

			showSRC = reader->getBoolValue("show","showSRC");
			showFORE = reader->getBoolValue("show","showFORE");
			showBACK = reader->getBoolValue("show","showBACK");
			showLocationPredictions = reader->getBoolValue("show","showLocationPredictions");
			showAllMotionVectors = reader->getBoolValue("show","showAllMotionVectors");
			showPath = reader->getBoolValue("show","showPath");
			showVectorsAsPath = reader->getBoolValue("show","showVectorsAsPath");

			recordVideo = reader->getBoolValue("output","recordVideo");
			return true;
		}

	public:
		void init(const char *filename)
		{
			readConfiguration(filename);
		}

		// --- Settings
		std::string outputVideoName;
		bool showSRC;
		bool showFORE;
		bool showBACK;
		bool showLocationPredictions;
		bool showAllMotionVectors;
		bool showPath;
		bool showVectorsAsPath;

		bool recordVideo;
	};

	cv::Mat result;
	cv::Mat imageToRecord;

	VideoWriter *outputVideo;

public:
	MyConfigManager configmanager;

	Mat *src;
	Mat *verbose;

	static Point lastMouseClickLocation;

	SimpleTracker *tracker;

	SimpleTrackingView(const char *configFileName)
	{
		init(configFileName);
		result = Mat(480,640,CV_8UC3);
		imageToRecord = Mat(480,2*640,CV_8UC3);
	}
	void init(const char *configFileName);

	~SimpleTrackingView()
	{
		delete outputVideo;
		outputVideo = NULL;
	}

	virtual void show(int frameIdx);

	void verboseMeanSizeAtLastClickLocation();
};

#endif
