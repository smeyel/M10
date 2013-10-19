#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "TrackedVehicle.h"
#include "TrackedVehicleManager.h"

void TrackedVehicle::registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack)
{
	// Save coordinates, image, moments etc.

	Point centroid((int)currentDetectingCvTrack->centroid.x,(int)currentDetectingCvTrack->centroid.y);
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
	LocationRegistration registration;
	registration.frameIdx = frameIdx;
	registration.confidence = 1.;
	registration.location = centroid;
	locationRegistrations.push_back(registration);

	// Show motion vector prediction cloud for next location
	if (manager->showLocationPredictions)
	{
		manager->motionVectorStorage->showMotionVectorPredictionCloud(centroid,manager->currentVerboseImage, 0.5);
	}
}

void TrackedVehicle::exportMotionVectors()
{
	OPENCV_ASSERT(manager->motionVectorStorage,"TrackedVehicle::exportMotionVectors","motionVectorStorage not set prior to export");
	// use locationRegistrations
	unsigned int lastFrameIdx = -2;
	Point lastLocation;
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != locationRegistrations.end(); it++)
	{
		if ((*it).frameIdx - lastFrameIdx == 1)
		{
			// For consecutive locations only
			manager->motionVectorStorage->addMotionVector(lastLocation,(*it).location);
		}
		lastLocation = (*it).location;
		lastFrameIdx = (*it).frameIdx;
	}
}


bool TrackedVehicle::isIntersecting(cvb::CvTrack *track, Area *area)
{
	Rect rect(track->minx, track->miny, track->maxx - track->minx, track->maxy - track->miny);
	return area->isRectangleIntersecting(rect);
}

bool TrackedVehicle::isIntersecting(LocationRegistration &registration, Area *area)
{
	return area->isPointInside(registration.location);
}

void TrackedVehicle::checkForAreaIntersections(LocationRegistration &registration, vector<unsigned int> &areaHitList, float minConfidence)
{
	// Register area hits
	for(unsigned int areaIdx=0; areaIdx<manager->trackedAreas->size(); areaIdx++)
	{
		if (isIntersecting(registration, &(*manager->trackedAreas)[areaIdx]))
		{
/*			cout << (currentDetectingCvTrack->inactive ? "inactive " : "  active ");
			cout << "CAR " << trackID << " in AREA " << (*manager->trackedAreas)[areaIdx].id << endl;*/

			areaHitList.push_back(areaIdx);
		}
	}
}

TrackedVehicle::TrackedVehicle(unsigned int iTrackID, TrackedVehicleManager *manager)
{
	this->manager = manager;
	trackID = iTrackID;
}

vector<unsigned int> TrackedVehicle::exportAllAreaHits(float minConfidence)
{
	vector<unsigned int> resultList;
	// Go along all locations and check for area hits
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		if (it->confidence >= minConfidence)
		{
			checkForAreaIntersections(*it,resultList,minConfidence);
		}
	}
	return resultList;
}

std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle)
{
/*	output << "Detections for trackID=" << trackedVehicle.trackID << ":" << endl << "\t";
	for(vector<unsigned int>::iterator it=trackedVehicle.areaHits.begin(); it!=trackedVehicle.areaHits.end(); it++)
	{
		output << *it << " ";
	}
	output << endl; */
	return output;
}
