#include "TrackedVehicleManager.h"

TrackedVehicle *TrackedVehicleManager::getTrackedVehicleOrCreate(unsigned int trackId)
{
	if (trackedVehicles.count(trackId) == 0)
	{
		TrackedVehicle *trackedVehicle = new TrackedVehicle(trackId, this);
		trackedVehicles.insert(std::make_pair(trackId, trackedVehicle));
	}
	return trackedVehicles[trackId];
}

void TrackedVehicleManager::processTracks(unsigned int frameIdx, cvb::CvTracks *tracks)
{
	cvb::CvTracks::const_iterator it;
	for (it = tracks->begin(); it != tracks->end(); ++it)
	{
		TrackedVehicle *vehicle = getTrackedVehicleOrCreate(it->second->id);

		vehicle->registerDetection(frameIdx, it->second);
	}
}

void TrackedVehicleManager::exportAllDetections()
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		it->second->exportAllDetections();
	}
}

void TrackedVehicleManager::collectMotionVectors(float minConfidence)
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->feedMotionVectorsIntoMotionVectorStorage(minConfidence);
	}
}

void TrackedVehicleManager::showAllPath(Mat &img)
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->showPath(img,true,false,false);
	}
}

void TrackedVehicleManager::recalculateLocationConfidences()
{
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		(*it).second->recalculateLocationConfidences();
	}
}