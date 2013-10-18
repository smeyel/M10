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

		loadLutAtStartup = reader->getBoolValue("main","loadLutAtStartup");
		lutFile = reader->getStringValue("main","lutFile");

		videoInputFileOverride = reader->getBoolValue("input","videoInputFileOverride");
		videoInputFilename = reader->getStringValue("input","videoInputFilename");
		trackedAreaInputFilename = reader->getStringValue("input","trackedAreaInputFilename");
		backgroundAreaInputFilename = reader->getStringValue("input","backgroundAreaInputFilename");

		detectionOutputFilename = reader->getStringValue("output","detectionOutputFilename");
		areaHitOutputFilename = reader->getStringValue("output","areaHitOutputFilename");
		imageOutputDirectory = reader->getStringValue("output","imageOutputDirectory");
		doSaveImages = reader->getBoolValue("output","doSaveImages");

		showSRC = reader->getBoolValue("show","showSRC");
		showFORE = reader->getBoolValue("show","showFORE");
		showBACK = reader->getBoolValue("show","showBACK");

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
	bool loadLutAtStartup;
	std::string lutFile;
	bool videoInputFileOverride;
	std::string videoInputFilename;

	std::string trackedAreaInputFilename;
	std::string backgroundAreaInputFilename;

	std::string detectionOutputFilename;
	std::string areaHitOutputFilename;
	std::string imageOutputDirectory;
	bool doSaveImages;

	bool showSRC;
	bool showFORE;
	bool showBACK;

	unsigned int minBlobArea;
	unsigned int maxBlobArea;
};

#endif
