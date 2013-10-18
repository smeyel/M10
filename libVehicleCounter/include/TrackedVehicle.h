#ifndef __TRACKEDVEHICLE_H
#define __TRACKEDVEHICLE_H

#include <iostream>
#include <vector>

#include "cvblobwrapper.h"
#include "area.h"

using namespace std;

/** For every timeframe: TrackID, location, visual properties (size etc for clustering), intersecting detection Areas
*/
class TrackedVehicle
{
	unsigned int trackID;	// May not reference CvTrack, that is removed after getting useless!
	vector<unsigned int> areaHits;

	void registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *sourceImage=NULL, Mat *foregroundMask=NULL);
	bool isIntersecting(cvb::CvTrack *track, Area *area);
	void checkForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, std::vector<Area> *areas);

public:
	TrackedVehicle(unsigned int iTrackID);
	void registerDetectionAndCheckForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, std::vector<Area> *areas, Mat *sourceImage=NULL, Mat *foregroundMask=NULL);

	friend std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle);
};

std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle);

#endif
