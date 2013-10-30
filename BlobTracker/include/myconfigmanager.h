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
		validPathInputFilename = reader->getStringValue("input","validPathInputFilename");

		registrationsFilename = reader->getStringValue("output","registrationsFilename");

		detectionOutputFilename = reader->getStringValue("output","detectionOutputFilename");
		areaHitOutputFilename = reader->getStringValue("output","areaHitOutputFilename");
		imageOutputDirectory = reader->getStringValue("output","imageOutputDirectory");
		doSaveImages = reader->getBoolValue("output","doSaveImages");
		outputVideoName = reader->getStringValue("output","outputVideoName");

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

	std::string registrationsFilename;

	std::string detectionOutputFilename;
	std::string areaHitOutputFilename;
	std::string imageOutputDirectory;
	bool doSaveImages;
	std::string motionVectorInputFilename;
	std::string validPathInputFilename;
	std::string outputVideoName;
};

#endif
