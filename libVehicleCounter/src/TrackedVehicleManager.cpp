#include "TrackedVehicleManager.h"

TrackedVehicle *TrackedVehicleManager::getTrackedVehicleOrCreate(unsigned int trackId)
{
	if (trackedVehicles.count(trackId) == 0)
	{
		TrackedVehicle *trackedVehicle = new TrackedVehicle(trackId);
		trackedVehicle->measurementExport = this->measurementExport;
		trackedVehicle->motionVectorStorage = this->motionVectorStorage;
		trackedVehicles.insert(std::make_pair(trackId, trackedVehicle));
	}
	return trackedVehicles[trackId];
}

void TrackedVehicleManager::processTracks(unsigned int frameIdx, cvb::CvTracks *tracks, std::vector<Area> *areas, Mat *verboseImg, Mat *srcImage, Mat *foregroundMask)
{
	cvb::CvTracks::const_iterator it;
	for (it = tracks->begin(); it != tracks->end(); ++it)
	{
		TrackedVehicle *vehicle = getTrackedVehicleOrCreate(it->second->id);

		vehicle->registerDetectionAndCheckForAreaIntersections(frameIdx, it->second, areas, srcImage, foregroundMask);
	}
}

void TrackedVehicleManager::exportAreaHits(bool onStdout, bool onExportfile)
{
	cout << "List of detections:" << endl;
	for(map<unsigned int,TrackedVehicle*>::iterator it = trackedVehicles.begin(); it != trackedVehicles.end(); it++)
	{
		if (onStdout)
			cout << *(it->second);
		if (onExportfile)
			it->second->exportAllAreaHits();
	}
}
