#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "TrackedVehicle.h"
#include "TrackedVehicleManager.h"

#define DEFAULTCONFIDENCE 0.1

Size TrackedVehicle::fullImageSize;	// static field

void TrackedVehicle::registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack)
{
	// ---------- Calculate current results
	Point centroid((int)currentDetectingCvTrack->centroid.x,(int)currentDetectingCvTrack->centroid.y);
	Point upperLeft(currentDetectingCvTrack->minx, currentDetectingCvTrack->miny);
	Size size(currentDetectingCvTrack->maxx - currentDetectingCvTrack->minx, currentDetectingCvTrack->maxy - currentDetectingCvTrack->miny);
	Rect rect(upperLeft,size);

	// Estimate detection confidence
	//	Last location: last element of locationRegistrations
	//	Current location: centroid
	float confidence = DEFAULTCONFIDENCE;	// Default confidence
	if (locationRegistrations.size() > 0)
	{
		Point prevLocation = (locationRegistrations.end()-1)->centroid;
		confidence = manager->motionVectorStorage->getConfidence(prevLocation,centroid);
		cout << "Confidence of new location: " << confidence << endl;
	}

	// Get size information
	manager->vehicleSizeStorage->add(centroid,size);
	float sizeRatio = manager->vehicleSizeStorage->getAreaRatioToMean(centroid,size);
	//cout << "CarID " << this->trackID << ": size ratio to mean: " << sizeRatio << endl;

	// ---------- Store/export current results
	// Save images
	string srcImgRoiFilename;
	string foreImgRoiFilename;
	if (manager->currentSourceImage)
	{
		srcImgRoiFilename = this->manager->measurementExport->saveimage(this->trackID,"S",frameIdx,*manager->currentSourceImage,rect);
	}
	if (manager->currentForegroundMask)
	{
		foreImgRoiFilename = this->manager->measurementExport->saveimage(this->trackID,"M",frameIdx,*manager->currentForegroundMask,rect);
	}

	// Store registration data
	LocationRegistration registration;
	registration.frameIdx = frameIdx;
	registration.confidence = confidence;
	registration.centroid = centroid;
	registration.boundingBox = rect;
	registration.sizeRatioToMean = sizeRatio;
	registration.srcImageFilename = srcImgRoiFilename;
	registration.maskImageFilename = foreImgRoiFilename;
	locationRegistrations.push_back(registration);

	// ---------- Visualize current results
	// Show motion vector prediction cloud for next location
	if (manager->showLocationPredictions)
	{
		manager->motionVectorStorage->showMotionVectorPredictionCloud(centroid,manager->currentVerboseImage, 0.5);
	}

	// Show size ratio
	line(*manager->currentVerboseImage,
		Point(upperLeft.x, upperLeft.y-5),
		Point(upperLeft.x + 50, upperLeft.y-7),
		Scalar(255,0,0), 3);
	Scalar color(0,255,255);	// yellow be default
	if (sizeRatio<0.8)
	{
		color = Scalar(0,255,0);	// Green (definitely small)
	}
	else if (sizeRatio>1.2)
	{
		color = Scalar(0,0,255);	// Red (definitely big)
	}

	line(*manager->currentVerboseImage,
		Point(upperLeft.x, upperLeft.y-5),
		Point(upperLeft.x + (int)(sizeRatio*50.0F), upperLeft.y-5),
		color, 3);
}

void TrackedVehicle::exportAllDetections(float minConfidence)
{
	recalculateLocationConfidences();

	// Go along all locations and check for area hits
	vector<unsigned int> trackedAreaHits;
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		if (it->confidence >= minConfidence)
		{
			checkForAreaIntersections(*it,trackedAreaHits,minConfidence);
		}
	}
	// TODO: Tidy-up this list and create a final "which direction did it go to" description.
	// TODO: Export area hits

	// Export all LocationRegistrations
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		manager->measurementExport->detectionOutput
			<< this->trackID << ";"
			<< it->frameIdx << ";"
			<< it->boundingBox.x << ";" << it->boundingBox.y << ";"
			<< it->boundingBox.width << ";" << it->boundingBox.height << ";"
			<< it->confidence << ";"	// new field!
			<< it->sizeRatioToMean << ";"	// new field!
			<< it->srcImageFilename << ";"
			<< it->maskImageFilename << endl;
	}

	// Draw path on image with sizeRatio information and save it
	Mat pathImage(TrackedVehicle::fullImageSize.height,TrackedVehicle::fullImageSize.width,CV_8UC3);
	showPath(pathImage,true,true,true);
	manager->measurementExport->saveimage(this->trackID,"P", 0, pathImage, Rect(0,0,pathImage.cols,pathImage.rows));

}


void TrackedVehicle::feedMotionVectorsIntoMotionVectorStorage(float minConfidence)
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
				manager->motionVectorStorage->addMotionVector(lastLocation,(*it).centroid);
			}
		}
		lastLocation = (*it).centroid;
		lastFrameIdx = (*it).frameIdx;
	}
}

void TrackedVehicle::showPath(Mat &img, bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox)
{
	// Show bounding boxes (measured and mean)
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != (locationRegistrations.end()-1); it++)
	{
		if (showBoundingBox)
		{
			rectangle(img,it->boundingBox,Scalar(255,255,255));
		}

		if (showMeanBoundingBox)
		{
			Size meanSize = manager->vehicleSizeStorage->getMeanSize(it->centroid);
			Rect meanRect(
				it->centroid.x - meanSize.width/2, it->centroid.y - meanSize.height/2, 
				it->centroid.x - meanSize.width/2, it->centroid.y - meanSize.height/2);
			rectangle(img,meanRect,Scalar(100,100,100));
		}
	}
	
	// Show locations
	if (showContinuousPath)
	{
		for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != (locationRegistrations.end()-1); it++)
		{
			int color = (int)(255. * (*it).confidence);
			if ( (*(it+1)).frameIdx > (*it).frameIdx+1)
			{
				// Not continuous detection sequence
				continue;
			}
			Point p1 = (*(it)).centroid;
			Point p2 = (*(it+1)).centroid;
			line(img,p1,p2,Scalar(color,color,color));
		}
	}
	else
	{
		for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != locationRegistrations.end(); it++)
		{
			int color = (int)(255. * (*it).confidence);
			Point p = (*(it)).centroid;
			circle(img,p,2,Scalar(color,color,color));
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
	return area->isPointInside(registration.centroid);
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

// DEPRECATED
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
		Point p1 = (*(it)).centroid;
		Point p2 = (*(it+1)).centroid;
		(*(it+1)).confidence = manager->motionVectorStorage->getConfidence(p1,p2);
	}
}

/*std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle)
{
	output << "Detections for trackID=" << trackedVehicle.trackID << ":" << endl << "\t";
	for(vector<unsigned int>::iterator it=trackedVehicle.areaHits.begin(); it!=trackedVehicle.areaHits.end(); it++)
	{
		output << *it << " ";
	}
	output << endl;
	return output;
} */
