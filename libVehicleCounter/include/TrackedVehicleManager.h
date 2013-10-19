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
	TrackedVehicle *getTrackedVehicleOrCreate(unsigned int trackId);

public:
	Mat *currentVerboseImg;
	Mat *currentSourceImage;
	Mat *currentForegroundMask;
	Mat *currentVerboseImage;

	std::vector<Area> *trackedAreas;

	bool showLocationPredictions;

	MeasurementExport *measurementExport;
	MotionVectorStorage *motionVectorStorage;	// Used only to set for new TrackedVehicles

	void processTracks(unsigned int frameIdx, cvb::CvTracks *tracks);
	void exportAreaHits(bool onStdout, bool onExportfile);
	void collectMotionVectors();

	void showAllPath(Mat *img);
	void recalculateLocationConfidences();
};

#endif
