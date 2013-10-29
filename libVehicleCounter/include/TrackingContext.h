#ifndef __TRACKINGCONTEXT_H
#define __TRACKINGCONTEXT_H

#include <vector>
#include "TrackedVehicle.h"
#include "VehicleSizeStorage.h"
#include "MotionVectorStorage.h"

class TrackingContext
{

public:
	//std::vector<TrackedVehicle> vehicles;
	VehicleSizeStorage sizeStorage;
	MotionVectorStorage motionVectorStorage;

	/** Measurement export target */
	MeasurementExport *measurementExport;


	bool showLocationPredictions;

	/** Vehicles */
	map<unsigned int,TrackedVehicle*> trackedVehicles;
	TrackedVehicle *getTrackedVehicleOrCreate(unsigned int trackId);

	void clear();


	/** Tracked areas */
	std::vector<Area> trackedAreas;
	void loadTrackedAreas(string filename)
	{
		Area::loadAreaList(filename.c_str(), &trackedAreas);
	}

	std::vector<Area> backgroundAreas;
	void loadBackgroundAreas(string filename)
	{
		Area::loadAreaList(filename.c_str(), &backgroundAreas);
	}
	void clearBackgroundAreasInImage(Mat &img)
	{
		for(unsigned int i=0; i<backgroundAreas.size(); i++)
		{
			backgroundAreas[i].draw(&img,Scalar(30,30,30),true);
		}
	}


	/** Data export functions */
	void exportAllDetections(float minConfidence);

	/** Rebuild list of motion vectors based on all tracked vehicles */
	void recollectMotionVectors(float minConfidence=0.);

	/** Recalculate detection confidences */

	void recalculateLocationConfidences();

	void showPath(TrackedVehicle &vehicle, Mat &img, bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox);

	void exportPathOfAllVehicle(bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox);

	void exportLocationRegistrations(int frameIdx, vector<LocationRegistration*> *targetVector, bool clearVector=true)
	{
		if (clearVector)
		{
			targetVector->clear();
		}
		for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
		{
			(*it).second->exportLocationRegistrations(frameIdx, targetVector);
		}
	}
};

#endif
