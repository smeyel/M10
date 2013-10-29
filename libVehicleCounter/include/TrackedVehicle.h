#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include <iostream>
#include <vector>

#include "cvblobwrapper.h"
#include "area.h"
#include "MeasurementExport.h"
#include "MotionVectorStorage.h"

using namespace std;

class TrackingContext;	// Against circular header includes.

struct LocationRegistration
{
	unsigned int frameIdx;
	float confidence;
	Point centroid;
	Rect bigBoundingBox;	// As returned by cvBlob
	Rect boundingBox;		// Narrowed using 10% and intergal image method (TrackedVehicle::getNarrowBoundingBox)
	float sizeRatioToMean;
	string srcImageFilename;
	string maskImageFilename;
	Point lastSpeedVector;	// Speed vector from the previous location if there was a detection. Otherwise, 0;0.
	int areaHitIdx;	// ID of the hit area (now assuming non-overlapping areas)
};

/** For every timeframe: TrackID, location, visual properties (size etc for clustering), intersecting detection Areas
*/
class TrackedVehicle
{
	bool isIntersecting(cvb::CvTrack *track, Area *area);
	bool isIntersecting(LocationRegistration &registration, Area *area);

	void checkForAreaIntersections(LocationRegistration &registration, vector<unsigned int> &areaHitList, float minConfidence);

	// call this after all detections
	//	Used by exportAllDetections()
	// DEPRECATED
	vector<unsigned int> exportAllAreaHits(float minConfidence = 0.1);

	Rect getNarrowBoundingBox(Mat &foreground, Rect originalRect);

public:
	TrackingContext *context;

	unsigned int trackID;	// May not reference CvTrack, that is removed after getting useless!
	vector<LocationRegistration> locationRegistrations;

	static Size fullImageSize;

	TrackedVehicle(unsigned int iTrackID, TrackingContext *context);

	/** Returns currently created LocationRegistration */
	void registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *srcImg, Mat *foregroundImg, Mat *verboseImage);

	// Called by MotionVectorStorage
	void feedMotionVectorsIntoMotionVectorStorage(float minConfidence=0.);
	
	// LocationRegistration contains confidence estimated at detection time.
	//	This function may be used to re-estimate them in a later time, based on much
	//	more information collected after the original detection.
	void recalculateLocationConfidences();

	// call this after all detections
	void exportAllDetections(float minConfidence);


};

//std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle);

#endif
