#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "TrackedVehicle.h"
#include "TrackedVehicleManager.h"

#define DEFAULTCONFIDENCE 0.1

bool TrackedVehicle::showVectorsAsPath;

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
	// TODO export with confidence value!
	manager->measurementExport->detectionOutput
		<< this->trackID << ";"
		<< frameIdx << ";"
		<< upperLeft.x << ";" << upperLeft.y << ";"
		<< size.width << ";" << size.height << ";"
		<< srcImgRoiFilename << ";"
		<< foreImgRoiFilename << endl;

	// Estimate detection confidence
	//	Last location: last element of locationRegistrations
	//	Current location: centroid
	float confidence = DEFAULTCONFIDENCE;	// Default confidence
	if (locationRegistrations.size() > 0)
	{
		Point prevLocation = (locationRegistrations.end()-1)->location;
		confidence = manager->motionVectorStorage->getConfidence(prevLocation,centroid);
		cout << "Confidence of new location: " << confidence << endl;
	}

	LocationRegistration registration;
	registration.frameIdx = frameIdx;
	registration.confidence = confidence;
	registration.location = centroid;
	registration.boundingBox = rect;
	locationRegistrations.push_back(registration);

	// Show motion vector prediction cloud for next location
	if (manager->showLocationPredictions)
	{
		manager->motionVectorStorage->showMotionVectorPredictionCloud(centroid,manager->currentVerboseImage, 0.5);
	}
}

void TrackedVehicle::exportMotionVectors(float minConfidence)
{
	OPENCV_ASSERT(manager->motionVectorStorage,"TrackedVehicle::exportMotionVectors","motionVectorStorage not set prior to export");
	// use locationRegistrations
	unsigned int lastFrameIdx = -2;
	Point lastLocation;
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != locationRegistrations.end(); it++)
	{
		if ((*it).frameIdx - lastFrameIdx == 1)	// For consecutive locations only
		{
			if ((*it).confidence >= minConfidence)	// With sufficient confidence
			{
				manager->motionVectorStorage->addMotionVector(lastLocation,(*it).location);
			}
		}
		lastLocation = (*it).location;
		lastFrameIdx = (*it).frameIdx;
	}
}

void TrackedVehicle::showPath(Mat *img)
{
	if (showVectorsAsPath)
	{
		for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != (locationRegistrations.end()-1); it++)
		{
			int color = (int)(255. * (*it).confidence);
			if ( (*(it+1)).frameIdx > (*it).frameIdx+1)
			{
				// Not continuous detection sequence
				continue;
			}
			Point p1 = (*(it)).location;
			Point p2 = (*(it+1)).location;
			line(*img,p1,p2,Scalar(color,color,color));
		}
	}
	else
	{
		for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != locationRegistrations.end(); it++)
		{
			int color = (int)(255. * (*it).confidence);
			Point p = (*(it)).location;
			circle(*img,p,2,Scalar(color,color,color));
		}
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

void TrackedVehicle::recalculateLocationConfidences()
{
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != (locationRegistrations.end()-1); it++)
	{
		Point p1 = (*(it)).location;
		Point p2 = (*(it+1)).location;
		(*(it+1)).confidence = manager->motionVectorStorage->getConfidence(p1,p2);
	}
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
