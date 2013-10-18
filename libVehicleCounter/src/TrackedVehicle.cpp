#include "TrackedVehicle.h"

void TrackedVehicle::registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack)
{
	// Save coordinates, image, moments etc.
}

bool TrackedVehicle::isIntersecting(cvb::CvTrack *track, Area *area)
{
	Rect rect(track->minx, track->miny, track->maxx - track->minx, track->maxy - track->miny);
	return area->isRectangleIntersecting(rect);
}

void TrackedVehicle::checkForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, std::vector<Area> *areas)
{
	// Register area hits
	for(unsigned int areaIdx=0; areaIdx<areas->size(); areaIdx++)
	{
		if (isIntersecting(currentDetectingCvTrack, &(*areas)[areaIdx]))
		{
			cout << (currentDetectingCvTrack->inactive ? "inactive " : "  active ");
			cout << "CAR " << trackID << " in AREA " << (*areas)[areaIdx].id << endl;

			areaHits.push_back(areaIdx);
		}
	}
}

TrackedVehicle::TrackedVehicle(unsigned int iTrackID)
{
	trackID = iTrackID;
}

void TrackedVehicle::registerDetectionAndCheckForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, std::vector<Area> *areas)
{
	registerDetection(frameIdx, currentDetectingCvTrack);
	checkForAreaIntersections(frameIdx, currentDetectingCvTrack, areas);
}

std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle)
{
	output << "Detections for trackID=" << trackedVehicle.trackID << ":" << endl << "\t";
	for(vector<unsigned int>::iterator it=trackedVehicle.areaHits.begin(); it!=trackedVehicle.areaHits.end(); it++)
	{
		output << *it << " ";
	}
	output << endl;
	return output;
}