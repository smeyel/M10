#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include <iostream>
#include <vector>

#include "cvblobwrapper.h"
#include "area.h"
#include "MeasurementExport.h"
#include "MotionVectorStorage.h"

using namespace std;

class TrackedVehicleManager;	// Against circular header includes.

/** For every timeframe: TrackID, location, visual properties (size etc for clustering), intersecting detection Areas
*/
class TrackedVehicle
{
	struct LocationRegistration
	{
		unsigned int frameIdx;
		float confidence;
		Point location;
		Rect boundingBox;
	};

	unsigned int trackID;	// May not reference CvTrack, that is removed after getting useless!
	vector<LocationRegistration> locationRegistrations;

	bool isIntersecting(cvb::CvTrack *track, Area *area);
	bool isIntersecting(LocationRegistration &registration, Area *area);

	void checkForAreaIntersections(LocationRegistration &registration, vector<unsigned int> &areaHitList, float minConfidence);

	TrackedVehicleManager *manager;

public:
	TrackedVehicle(unsigned int iTrackID, TrackedVehicleManager *manager);

	void registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack);

	void exportMotionVectors(float minConfidence=0.);
	
	void showPath(Mat *img);

	void recalculateLocationConfidences();


	// call this after all detections
	vector<unsigned int> exportAllAreaHits(float minConfidence = 0.1);

	friend std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle);
};

std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle);

#endif
