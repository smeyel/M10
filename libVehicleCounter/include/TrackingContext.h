#ifndef __TRACKINGCONTEXT_H
#define __TRACKINGCONTEXT_H

#include <vector>
#include "TrackedVehicle.h"
#include "VehicleSizeStorage.h"
#include "MotionVectorStorage.h"
#include "PathValidator.h"

class TrackingContext
{

public:
	VehicleSizeStorage sizeStorage;
	MotionVectorStorage motionVectorStorage;
	PathValidator pathValidator;

	/** Settings */
	bool showLocationPredictions;

	/** Vehicles */
	map<unsigned int,TrackedVehicle*> trackedVehicles;
	TrackedVehicle *getTrackedVehicleOrCreate(unsigned int trackId);
	TrackedVehicle *getTrackedVehicleOrNull(unsigned int trackId);
	void clear();

	/** Measurement export target */
	MeasurementExport *measurementExport;

	/** Tracked areas, background areas */
	std::vector<Area> trackedAreas;
	void loadTrackedAreas(string filename);

	std::vector<Area> backgroundAreas;
	void loadBackgroundAreas(string filename);
	void clearBackgroundAreasInImage(Mat &img);

	/** Data export functions */
	void exportAllDetections(float minConfidence);

	/** Rebuild list of motion vectors based on all tracked vehicles */
	void recollectMotionVectors(float minConfidence=0.);

	/** Recalculate detection confidences */

	void recalculateLocationConfidences();

	void showPath(TrackedVehicle &vehicle, Mat &img, bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox);

	void exportPathOfAllVehicle(bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox);

	void exportLocationRegistrations(int frameIdx, vector<LocationRegistration*> *targetVector, bool clearVector=true);

	void validatePath(float minConfidence);

	void saveVehicles(const char *filename);
	void loadVehicles(const char *filename);
};

#endif
