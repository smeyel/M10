#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "TrackedVehicle.h"

void TrackedVehicle::registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *sourceImage, Mat *foregroundMask)
{
	// Save coordinates, image, moments etc.

	Point centroid(currentDetectingCvTrack->centroid.x,currentDetectingCvTrack->centroid.y);
	Point upperLeft(currentDetectingCvTrack->minx, currentDetectingCvTrack->miny);
	Size size(currentDetectingCvTrack->maxx - currentDetectingCvTrack->minx, currentDetectingCvTrack->maxy - currentDetectingCvTrack->miny);
	Rect rect(upperLeft,size);

//	Mat srcImgRoI(*sourceImage,rect);
/*	if (sourceImage)
	{
		cv::Mat roi = (*sourceImage)(rect);
		imwrite("dummy.png",roi);
	}*/


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

void TrackedVehicle::registerDetectionAndCheckForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, std::vector<Area> *areas, Mat *sourceImage, Mat *foregroundMask)
{
	registerDetection(frameIdx, currentDetectingCvTrack, sourceImage, foregroundMask);
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