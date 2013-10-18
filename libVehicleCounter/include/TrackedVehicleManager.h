#ifndef __TRACKEDVEHICLEMANAGER_H
#define __TRACKEDVEHICLEMANAGER_H

#include <iostream>
#include <vector>

#include "cvblobwrapper.h"
#include "area.h"
#include "TrackedVehicle.h"

using namespace std;

class TrackedVehicleManager
{
	map<unsigned int,TrackedVehicle*> trackedVehicles;
public:
	TrackedVehicle *getTrackedVehicleOrCreate(unsigned int trackId);
	void processTracks(unsigned int frameIdx, cvb::CvTracks *tracks, std::vector<Area> *areas, Mat *verboseImg = NULL);
	void showDetections();
};

#endif
