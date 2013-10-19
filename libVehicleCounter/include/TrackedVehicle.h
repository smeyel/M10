#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include <iostream>
#include <vector>

#include "cvblobwrapper.h"
#include "area.h"
#include "MeasurementExport.h"
#include "MotionVectorStorage.h"

using namespace std;

/** For every timeframe: TrackID, location, visual properties (size etc for clustering), intersecting detection Areas
*/
class TrackedVehicle
{
	unsigned int trackID;	// May not reference CvTrack, that is removed after getting useless!
	vector<unsigned int> areaHits;

	// Last known location and its frame index
	//	Used by registerDetection to add motion vectors to motionVectorStorage.
	Point lastKnownLocation;
	unsigned int lastKnownLocationFrameIdx;

	void registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *sourceImage=NULL, Mat *foregroundMask=NULL);
	bool isIntersecting(cvb::CvTrack *track, Area *area);
	void checkForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, std::vector<Area> *areas);

public:
	MeasurementExport *measurementExport;
	MotionVectorStorage *motionVectorStorage;

	TrackedVehicle(unsigned int iTrackID);
	void registerDetectionAndCheckForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, std::vector<Area> *areas, Mat *sourceImage=NULL, Mat *foregroundMask=NULL);
	
	// call this after all detections
	void exportAllAreaHits();

	friend std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle);
};

std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle);

#endif
