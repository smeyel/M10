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

TrackedVehicle *TrackingContext::getTrackedVehicleOrNull(unsigned int trackId)
{
	if (trackedVehicles.count(trackId) == 0)
	{
		return NULL;
	}
	return trackedVehicles[trackId];
}

void TrackingContext::loadTrackedAreas(string filename)
{
	Area::loadAreaList(filename.c_str(), &trackedAreas);
}

void TrackingContext::loadBackgroundAreas(string filename)
{
	Area::loadAreaList(filename.c_str(), &backgroundAreas);
}

void TrackingContext::clearBackgroundAreasInImage(Mat &img)
{
	for(unsigned int i=0; i<backgroundAreas.size(); i++)
	{
		backgroundAreas[i].draw(&img,Scalar(30,30,30),true);
	}
}


void TrackingContext::exportAllDetections(float minConfidence)
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		it->second->exportAllDetections(minConfidence);
	}
}

/*void TrackingContext::recollectMotionVectors(float minConfidence)
{
	this->motionVectorStorage.clear();

	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->feedMotionVectorsIntoMotionVectorStorage(minConfidence);
	}
} */

/*void TrackingContext::recalculateLocationConfidences()
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->recalculateLocationConfidences();
	}
} */

void TrackingContext::clear()
{
	for(map<unsigned int,TrackedVehicle*>::iterator vehicleIterator=trackedVehicles.begin(); vehicleIterator!=trackedVehicles.end(); vehicleIterator++)
	{
		delete vehicleIterator->second;
	}
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

void TrackingContext::saveVehicles(const char *filename)
{
	FileStorage fs(filename,FileStorage::WRITE);

	fs << "trackedvehicles" << "[";
	for(map<unsigned int,TrackedVehicle*>::iterator vehicleIterator=trackedVehicles.begin(); vehicleIterator!=trackedVehicles.end(); vehicleIterator++)
	{
		vehicleIterator->second->save(&fs);
	}

	fs << "]";
	fs.release();
}

void TrackingContext::loadVehicles(const char *filename)
{
	clear();

	FileStorage fs(filename,FileStorage::READ);
	ostringstream oss;

	FileNode vehicleNode = fs["trackedvehicles"];
	cout << "Loading trackedvehicles (size=" << vehicleNode.size() << ")" << endl;

	FileNodeIterator it = vehicleNode.begin();
	FileNodeIterator it_end = vehicleNode.end();

	for( ; it != it_end; ++it)
	{
		TrackedVehicle *trackedVehicle = new TrackedVehicle(&(*it), this);
		trackedVehicles.insert(std::make_pair(trackedVehicle->trackID, trackedVehicle));
	}

	fs.release();
}

void TrackingContext::exportLocationRegistrations(int frameIdx, vector<LocationRegistration*> *targetVector, bool clearVector)
{
	if (clearVector)
	{
		targetVector->clear();
	}
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->exportLocationRegistrations(frameIdx, targetVector);
	}
}

void TrackingContext::validatePath(float minConfidence)
{
	for(map<unsigned int,TrackedVehicle*>::iterator vehicleIterator=trackedVehicles.begin(); vehicleIterator!=trackedVehicles.end(); vehicleIterator++)
	{
		vehicleIterator->second->validatePath(minConfidence,NULL,NULL);
	}
}

void TrackingContext::savePathCounts()
{
	map<int,int> counters;	// PathID;Counter

	for(map<unsigned int,TrackedVehicle*>::iterator vehicleIterator=trackedVehicles.begin(); vehicleIterator!=trackedVehicles.end(); vehicleIterator++)
	{
		if (counters.count(vehicleIterator->second->pathID) == 0)
		{
			counters.insert(std::make_pair(vehicleIterator->second->pathID, 1));
		}
		else
		{
			counters[vehicleIterator->second->pathID]++;
		}
	}

	for(map<int,int>::iterator counterIterator=counters.begin(); counterIterator!=counters.end(); counterIterator++)
	{
		cout << "Counter for path " << counterIterator->first << ": " << counterIterator->second << endl;
		// TODO: make CSV with form src;dst;cnt
		measurementExport->pathCountersOutput << counterIterator->first << ";" << counterIterator->second << endl;
	}
}
