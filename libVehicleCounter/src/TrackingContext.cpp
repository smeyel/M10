#include "TrackingContext.h"

TrackedVehicle *TrackingContext::getTrackedVehicleOrCreate(unsigned int trackId)
{
	if (trackedVehicles.count(trackId) == 0)
	{
		TrackedVehicle *trackedVehicle = new TrackedVehicle(trackId, this);
		trackedVehicles.insert(std::make_pair(trackId, trackedVehicle));
	}
	return trackedVehicles[trackId];
}


void TrackingContext::exportAllDetections(float minConfidence)
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		it->second->exportAllDetections(minConfidence);
	}
}

void TrackingContext::recollectMotionVectors(float minConfidence)
{
	this->motionVectorStorage.clear();

	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->feedMotionVectorsIntoMotionVectorStorage(minConfidence);
	}
}

void TrackingContext::recalculateLocationConfidences()
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->recalculateLocationConfidences();
	}
}

void TrackingContext::clear()
{
	trackedVehicles.clear();
}

void TrackingContext::exportPathOfAllVehicle(bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox)
{
	// Draw path on image with sizeRatio information and save it
	for(map<unsigned int,TrackedVehicle*>::iterator it = this->trackedVehicles.begin(); it != this->trackedVehicles.end(); it++)
	{
		Mat pathImage(TrackedVehicle::fullImageSize.height,TrackedVehicle::fullImageSize.width,CV_8UC3);
		showPath(*(*it).second,pathImage,true,true,true);
		measurementExport->saveimage(it->second->trackID,"P", 0, pathImage, Rect(0,0,pathImage.cols,pathImage.rows),true);

		(*it).second->recalculateLocationConfidences();
	}
}

void TrackingContext::showPath(TrackedVehicle &vehicle, Mat &img, bool showContinuousPath, bool showBoundingBox, bool showMeanBoundingBox)
{
	// Show bounding boxes (measured and mean)
	for(vector<LocationRegistration>::iterator it=vehicle.locationRegistrations.begin(); it != (vehicle.locationRegistrations.end()-1); it++)
	{
		if (showBoundingBox)
		{
			rectangle(img,it->boundingBox,Scalar(255,255,255));
		}

		if (showMeanBoundingBox)
		{
			Size meanSize = sizeStorage.getMeanSize(it->centroid);
			Rect meanRect(
				it->centroid.x - meanSize.width/2, it->centroid.y - meanSize.height/2, 
				meanSize.width, meanSize.height);
			rectangle(img,meanRect,Scalar(100,100,100));
		}
	}
	
	// Show locations
	if (showContinuousPath)
	{
		for(vector<LocationRegistration>::iterator it=vehicle.locationRegistrations.begin(); it != (vehicle.locationRegistrations.end()-1); it++)
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
		for(vector<LocationRegistration>::iterator it=vehicle.locationRegistrations.begin(); it != vehicle.locationRegistrations.end(); it++)
		{
			int color = (int)(255. * (*it).confidence);
			Point p = (*(it)).centroid;
			circle(img,p,2,Scalar(color,color,color));
		}
	}
}
