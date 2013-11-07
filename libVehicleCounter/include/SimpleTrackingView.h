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
			showAllMotionVectors = reader->getBoolValue("show","showAllMotionVectors");
			showPath = reader->getBoolValue("show","showPath");
			showVectorsAsPath = reader->getBoolValue("show","showVectorsAsPath");

			showMotionVectorPredictionCloud = reader->getBoolValue("show","showMotionVectorPredictionCloud");
			showSizeRatioBars = reader->getBoolValue("show","showSizeRatioBars");
			showMeanSizeAtLocation = reader->getBoolValue("show","showMeanSizeAtLocation");
			showBoundingBox = reader->getBoolValue("show","showBoundingBox");
			showTrackId = reader->getBoolValue("show","showTrackId");

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
		bool showAllMotionVectors;
		bool showPath;
		bool showVectorsAsPath;

		bool showMotionVectorPredictionCloud;
		bool showSizeRatioBars;
		bool showMeanSizeAtLocation;
		bool showBoundingBox;
		bool showTrackId;

		bool recordVideo;
	};

	cv::Mat result;
	cv::Mat imageToRecord;

	VideoWriter *outputVideo;

	// Temp store used in show() to retrieve these registrations from the context
	vector<LocationRegistration*> locationRegistrationsOfCurrentFrame;
	void drawLocationRegistration(LocationRegistration *locReg, Mat &verbose);

public:
	MyConfigManager configmanager;

	Mat *src;
	Mat *verbose;

	static Point lastMouseClickLocation;

	TrackerBase *tracker;

	SimpleTrackingView(const char *configFileName);

	void init(const char *configFileName);

	~SimpleTrackingView();

	virtual void show(int frameIdx);

	void verboseMeanSizeAtLastClickLocation();
};

#endif
