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
	int frameIdx;
	float confidence;
	Point centroid;
	Rect bigBoundingBox;	// As returned by cvBlob
	Rect boundingBox;		// Narrowed using 10% and intergal image method (TrackedVehicle::getNarrowBoundingBox)
	float sizeRatioToMean;
	string srcImageFilename;
	string maskImageFilename;
	Point lastSpeedVector;	// Speed vector from the previous location if there was a detection. Otherwise, 0;0.
	int areaHitIdx;	// ID of the hit area (now assuming non-overlapping areas), -2 means not checked yet
};

/** For every timeframe: TrackID, location, visual properties (size etc for clustering), intersecting detection Areas
*/
class TrackedVehicle
{
	bool isIntersecting(cvb::CvTrack *track, Area *area);
	bool isIntersecting(LocationRegistration &registration, Area *area);

	void checkForAreaIntersections(LocationRegistration &registration, vector<int> &areaHitList, float minConfidence);

	Rect getNarrowBoundingBox(Mat &foreground, Rect originalRect);

public:
	TrackingContext *context;

	int trackID;	// May not reference CvTrack, that is removed after getting useless!
	vector<LocationRegistration> locationRegistrations;

	static Size fullImageSize;

	TrackedVehicle(int iTrackID, TrackingContext *context);

	TrackedVehicle(FileNode *node, TrackingContext *context)
	{
		this->context = context;
		load(node);
	}

	/** Returns currently created LocationRegistration */
	void registerDetection(int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *srcImg, Mat *foregroundImg, Mat *verboseImage);

	// Called by MotionVectorStorage
	void feedMotionVectorsIntoMotionVectorStorage(float minConfidence=0.);
	
	// LocationRegistration contains confidence estimated at detection time.
	//	This function may be used to re-estimate them in a later time, based on much
	//	more information collected after the original detection.
	void recalculateLocationConfidences();

	// call this after all detections
	void exportAllDetections(float minConfidence);

	void exportLocationRegistrations(int frameIdx, vector<LocationRegistration*> *targetVector);

	void save(FileStorage *fs);

	void load(FileNode *node);

private:
	void loadPoint(cv::FileNode& node, cv::Point &point);
	void loadRect(cv::FileNode& node, cv::Rect &rect);
};

//cv::FileNode& operator>>(cv::FileNode& node, cv::Point &point);

#endif
