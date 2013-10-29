#ifndef __MYCONFIGMANAGER_H
#define __MYCONFIGMANAGER_H
#include "stdlib.h"
#include "SimpleIniConfigReader.h"

using namespace LogConfigTime;

class MyConfigManager
{
	virtual bool readConfiguration(char *filename)
	{
		SimpleIniConfigReader *SIreader = new SimpleIniConfigReader(filename);
		ConfigReader *reader = SIreader;

		videoInputFileOverride = reader->getBoolValue("input","videoInputFileOverride");
		videoInputFilename = reader->getStringValue("input","videoInputFilename");
		trackedAreaInputFilename = reader->getStringValue("input","trackedAreaInputFilename");
		backgroundAreaInputFilename = reader->getStringValue("input","backgroundAreaInputFilename");
		motionVectorInputFilename = reader->getStringValue("input","motionVectorInputFilename");

		detectionOutputFilename = reader->getStringValue("output","detectionOutputFilename");
		areaHitOutputFilename = reader->getStringValue("output","areaHitOutputFilename");
		imageOutputDirectory = reader->getStringValue("output","imageOutputDirectory");
		doSaveImages = reader->getBoolValue("output","doSaveImages");
		outputVideoName = reader->getStringValue("output","outputVideoName");

		showSRC = reader->getBoolValue("show","showSRC");
		showFORE = reader->getBoolValue("show","showFORE");
		showBACK = reader->getBoolValue("show","showBACK");
		showLocationPredictions = reader->getBoolValue("show","showLocationPredictions");
		showAllMotionVectors = reader->getBoolValue("show","showAllMotionVectors");
		showPath = reader->getBoolValue("show","showPath");
		showVectorsAsPath = reader->getBoolValue("show","showVectorsAsPath");

		minBlobArea = reader->getIntValue("blob","minBlobArea");
		maxBlobArea = reader->getIntValue("blob","maxBlobArea");


		return true;
	}

public:
	void init(char *filename)
	{
		readConfiguration(filename);
	}

	// --- Settings
	bool videoInputFileOverride;
	std::string videoInputFilename;

	std::string trackedAreaInputFilename;
	std::string backgroundAreaInputFilename;

	std::string detectionOutputFilename;
	std::string areaHitOutputFilename;
	std::string imageOutputDirectory;
	bool doSaveImages;
	std::string motionVectorInputFilename;
	std::string outputVideoName;

	bool showSRC;
	bool showFORE;
	bool showBACK;
	bool showLocationPredictions;
	bool showAllMotionVectors;
	bool showPath;
	bool showVectorsAsPath;

	unsigned int minBlobArea;
	unsigned int maxBlobArea;
};

#endif
