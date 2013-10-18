#ifndef __TRACKEDVEHICLEMANAGER_H
#define __TRACKEDVEHICLEMANAGER_H

#include <iostream>
#include <fstream>
#include <vector>

#include "cvblobwrapper.h"
#include "area.h"
#include "TrackedVehicle.h"

#include "MeasurementExport.h"


using namespace std;

class TrackedVehicleManager
{
	map<unsigned int,TrackedVehicle*> trackedVehicles;

public:
	MeasurementExport *measurementExport;

	TrackedVehicle *getTrackedVehicleOrCreate(unsigned int trackId);
	void processTracks(unsigned int frameIdx, cvb::CvTracks *tracks, std::vector<Area> *areas, Mat *verboseImg = NULL, Mat *srcImage = NULL, Mat *foregroundMask = NULL);
	void exportAreaHits(bool onStdout, bool onExportfile);
};

#endif
