#ifndef __SIMPLETRACKER_H
#define __SIMPLETRACKER_H
#include "SimpleIniConfigReader.h"

#include "TrackerBase.h"

class SimpleTracker : public TrackerBase
{
	class MyConfigManager
	{
		virtual bool readConfiguration(const char *filename)
		{
			LogConfigTime::SimpleIniConfigReader *SIreader = new LogConfigTime::SimpleIniConfigReader(filename);
			LogConfigTime::ConfigReader *reader = SIreader;

			minBlobArea = reader->getIntValue("blob","minBlobArea");
			maxBlobArea = reader->getIntValue("blob","maxBlobArea");

			return true;
		}

	public:
		void init(const char *filename)
		{
			readConfiguration(filename);
		}

		// --- Settings
		unsigned int minBlobArea;
		unsigned int maxBlobArea;
	};



	class MyBackgroundSubtractor : public BackgroundSubtractorMOG2
	{
		public:
			MyBackgroundSubtractor() : BackgroundSubtractorMOG2(100,16,true) 	// Originally hist=10, thres=16
			{
				fTau = 0.6F;	// Not shadow if darker than background*fTau (?)
			}
	};

	CvBlobWrapper cvblob;

	MyConfigManager configmanager;

	MyBackgroundSubtractor backgroundSubtractor;
	Mat openKernel;

	Mat *backgroundFrame;
	Mat *foregroundFrame;
	Mat *blurredSrc;
public:

	SimpleTracker(const char *configfilename);

	void init();

	virtual void processFrame(Mat &src, int frameIdx, Mat *verbose);

	Mat *getCurrentForegroundImage();	// for debug
};

#endif