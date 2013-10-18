#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "TrackedVehicle.h"

void TrackedVehicle::registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *sourceImage, Mat *foregroundMask)
{
	// Save coordinates, image, moments etc.

	Point centroid(currentDetectingCvTrack->centroid.x,currentDetectingCvTrack->centroid.y);
	Point upperLeft(currentDetectingCvTrack->minx, currentDetectingCvTrack->miny);
	Size size(currentDetectingCvTrack->maxx - currentDetectingCvTrack->minx, currentDetectingCvTrack->maxy - currentDetectingCvTrack->miny);
	Rect rect(upperLeft,size);

	string srcImgRoiFilename;
	string foreImgRoiFilename;

	// Save images
	if (sourceImage)
	{
		srcImgRoiFilename = this->measurementExport->saveimage(sourceImage,rect);
	}
	if (foregroundMask)
	{
		foreImgRoiFilename = this->measurementExport->saveimage(foregroundMask,rect);
	}

	// Save detection data
	measurementExport->detectionOutput
		<< this->trackID << ";"
		<< frameIdx << ";"
		<< upperLeft.x << ";" << upperLeft.y << ";"
		<< size.width << ";" << size.height << ";"
		<< srcImgRoiFilename << ";"
		<< foreImgRoiFilename << endl;
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

void TrackedVehicle::exportAllAreaHits()
{
	measurementExport->areaHitOutput
		<< this->trackID << ";";
	for(vector<unsigned int>::iterator it=areaHits.begin(); it!=areaHits.end(); it++)
	{
		measurementExport->areaHitOutput
			<< *it << ";";
	}
	measurementExport->areaHitOutput
		<< endl;
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
