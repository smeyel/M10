#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "TrackedVehicle.h"
#include "TrackedVehicleManager.h"

void TrackedVehicle::registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack)
{
	// Save coordinates, image, moments etc.

	Point centroid(currentDetectingCvTrack->centroid.x,currentDetectingCvTrack->centroid.y);
	Point upperLeft(currentDetectingCvTrack->minx, currentDetectingCvTrack->miny);
	Size size(currentDetectingCvTrack->maxx - currentDetectingCvTrack->minx, currentDetectingCvTrack->maxy - currentDetectingCvTrack->miny);
	Rect rect(upperLeft,size);

	string srcImgRoiFilename;
	string foreImgRoiFilename;

	// Save images
	if (manager->currentSourceImage)
	{
		srcImgRoiFilename = this->manager->measurementExport->saveimage(manager->currentSourceImage,rect);
	}
	if (manager->currentForegroundMask)
	{
		foreImgRoiFilename = this->manager->measurementExport->saveimage(manager->currentForegroundMask,rect);
	}

	// Save detection data
	manager->measurementExport->detectionOutput
		<< this->trackID << ";"
		<< frameIdx << ";"
		<< upperLeft.x << ";" << upperLeft.y << ";"
		<< size.width << ";" << size.height << ";"
		<< srcImgRoiFilename << ";"
		<< foreImgRoiFilename << endl;

	// Store motion vector
	if (lastKnownLocationFrameIdx == frameIdx-1 && manager->motionVectorStorage!=NULL)
	{
		manager->motionVectorStorage->addMotionVector(lastKnownLocation,centroid);
	}
	lastKnownLocation = centroid;
	lastKnownLocationFrameIdx = frameIdx;

	// Show motion vector prediction cloud for next location
	manager->motionVectorStorage->showMotionVectorPredictionCloud(centroid,manager->currentVerboseImage);
}

bool TrackedVehicle::isIntersecting(cvb::CvTrack *track, Area *area)
{
	Rect rect(track->minx, track->miny, track->maxx - track->minx, track->maxy - track->miny);
	return area->isRectangleIntersecting(rect);
}

void TrackedVehicle::checkForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack)
{
	// Register area hits
	for(unsigned int areaIdx=0; areaIdx<manager->trackedAreas->size(); areaIdx++)
	{
		if (isIntersecting(currentDetectingCvTrack, &(*manager->trackedAreas)[areaIdx]))
		{
			cout << (currentDetectingCvTrack->inactive ? "inactive " : "  active ");
			cout << "CAR " << trackID << " in AREA " << (*manager->trackedAreas)[areaIdx].id << endl;

			areaHits.push_back(areaIdx);
		}
	}
}

TrackedVehicle::TrackedVehicle(unsigned int iTrackID, TrackedVehicleManager *manager)
{
	this->manager = manager;
	trackID = iTrackID;
}

void TrackedVehicle::registerDetectionAndCheckForAreaIntersections(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack)
{
	registerDetection(frameIdx, currentDetectingCvTrack);

	checkForAreaIntersections(frameIdx, currentDetectingCvTrack);
}

void TrackedVehicle::exportAllAreaHits()
{
	manager->measurementExport->areaHitOutput
		<< this->trackID << ";";
	for(vector<unsigned int>::iterator it=areaHits.begin(); it!=areaHits.end(); it++)
	{
		manager->measurementExport->areaHitOutput
			<< *it << ";";
	}
	manager->measurementExport->areaHitOutput
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
