#ifndef __TRACKEDVEHICLEMANAGER_H
#define __TRACKEDVEHICLEMANAGER_H

#include <iostream>
#include <fstream>
#include <vector>

#include "cvblobwrapper.h"
#include "area.h"
#include "TrackedVehicle.h"

#include "MeasurementExport.h"
#include "VehicleSizeStorage.h"

using namespace std;

class TrackedVehicleManager
{
	map<unsigned int,TrackedVehicle*> trackedVehicles;
	TrackedVehicle *getTrackedVehicleOrCreate(unsigned int trackId);

public:
	Mat *currentVerboseImg;
	Mat *currentSourceImage;
	Mat *currentForegroundMask;
	Mat *currentVerboseImage;

	std::vector<Area> *trackedAreas;

	bool showLocationPredictions;
	bool showPath;

	MeasurementExport *measurementExport;
	MotionVectorStorage *motionVectorStorage;	// Used only to set for new TrackedVehicles
	VehicleSizeStorage *vehicleSizeStorage;

	void processTracks(unsigned int frameIdx, cvb::CvTracks *tracks);
	void exportAllDetections(float minConfidence);
	void collectMotionVectors(float minConfidence=0.);

	void showAllPath(Mat &img);
	void recalculateLocationConfidences();

	void clear();
};

#endif
