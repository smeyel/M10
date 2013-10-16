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
		areaInputFilename = reader->getStringValue("input","areaInputFilename");

		showSRC = reader->getBoolValue("show","showSRC");
		showFORE = reader->getBoolValue("show","showFORE");
		showBACK = reader->getBoolValue("show","showBACK");

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

	std::string areaInputFilename;

	bool showSRC;
	bool showFORE;
	bool showBACK;
};

#endif
