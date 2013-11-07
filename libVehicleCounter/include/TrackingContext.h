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
	int nextUnusedTrackedVehicleId;
	TrackedVehicle *getTrackedVehicleOrCreate(unsigned int trackId);
	TrackedVehicle *getTrackedVehicleOrNull(unsigned int trackId);
	TrackedVehicle *createTrackedVehicle(Blob blob);

	TrackingContext()
	{
		clear();
	}

	void clear();

	/** Measurement export target */
	MeasurementExport *measurementExport;

	/** Tracked areas, background areas */
	std::vector<Area> trackedAreas;
	void loadTrackedAreas(string filename);

	std::vector<Area> backgroundAreas;
	void loadBackgroundAreas(string filename);

	/** Clears the area covered by backgroundAreas inside the given image.
		Used for image preprocessing.
		Called by: TODO
	*/
	void clearBackgroundAreasInImage(Mat &img);

	/** Exports detection data
		Calls TrackedVehicle->exportAllDetections
		which calls
		- TrackedVehicle::recalculateLocationConfidences
		- TrackedVehicle::validatePath
			- calls recalculateLocationConfidences (!) TODO
		Exports to
			context->measurementExport->areaHitOutput
			context->measurementExport->detectionOutput
	*/
	void exportAllDetections(float minConfidence);

	/** Rebuild list of motion vectors based on all tracked vehicles */
	//void recollectMotionVectors(float minConfidence=0.);

	/** Recalculate detection confidences
		calls all TrackedVehicle::recalculateLocationConfidences
	*/
	//void recalculateLocationConfidences();

	/** Uses sizeStorage to export all detected locations of a vehicle. Draws them into an image. */
	void showPath(TrackedVehicle &vehicle, Mat &img, bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox);

	/** Uses TrackingContext::showPath for all vehicles to write them into images.
		Calls TrackedVehicle::recalculateLocationConfidences after that for each vehicle! (TODO)
	*/
	void exportPathOfAllVehicle(bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox);

	/** Uses all TrackedVehicle::exportLocationRegistrations to write all LocationRegistrations into the targetVector.
		Used by: TODO
	*/
	void exportLocationRegistrations(int frameIdx, vector<LocationRegistration*> *targetVector, bool clearVector=true);

	/** Calls all TrackedVehicle::validatePath */
	void validatePath(float minConfidence);

	/** Saves trackedVehicles */
	void saveVehicles(const char *filename);

	/** Loads trackedVehicles */
	void loadVehicles(const char *filename);

	/** Calculates and saves vehicle number of every path.
		Exports into MeasurementExport::pathCountersOutput
	*/
	void savePathCounts();
};

#endif
