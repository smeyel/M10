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
		areaOutputFilename = reader->getStringValue("output","areaOutputFilename");
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

	std::string areaOutputFilename;

};

#endif
