#include "AdvancedTracker.h"
#include "Blob.h"

void AdvancedTracker::init()
{
	cvblob.minBlobArea = configmanager.minBlobArea;
	cvblob.maxBlobArea = configmanager.maxBlobArea;
	MyBackgroundSubtractor *backgroundSubtractor = new MyBackgroundSubtractor();
	Mat openKernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));
}

void AdvancedTracker::processFrame(Mat &src, int frameIdx, Mat *verbose)
{
	cout << "Tracking: New frame" << endl;
	cv::blur(src,*blurredSrc,cv::Size(10,10));
	src.copyTo(*blurredSrc);

	backgroundSubtractor.operator()(*blurredSrc,*foregroundFrame);

	morphologyEx(*foregroundFrame,*foregroundFrame,MORPH_OPEN,openKernel,Point(-1,-1),1);
	morphologyEx(*foregroundFrame,*foregroundFrame,MORPH_CLOSE,openKernel,Point(-1,-1),1);

	vector<Blob> blobs;
	cvblob.findBlobs(foregroundFrame,verbose,blobs);
	for(vector<Blob>::iterator blobIt = blobs.begin(); blobIt!=blobs.end(); blobIt++)
	{
		blobIt->id = blobIt - blobs.begin();
	}

	// Ask every tracked vehicle to find its blob
	for(map<unsigned int,TrackedVehicle*>::iterator it = context->trackedVehicles.begin(); it != context->trackedVehicles.end(); it++)
	{
		if (!it->second->isActive())
		{
			continue;
		}

		TrackedVehicle *currentVehicle = it->second;
		if (currentVehicle->isActive())
		{
			int choosen = findBlob(*currentVehicle,blobs, frameIdx);
			if (choosen >= 0)
			{
				blobs[choosen].assignedVehicleCounter++;
				currentVehicle->registerBlob(blobs[choosen], frameIdx, &src, this->foregroundFrame, verbose);
				currentVehicle->lastAssociatedBlobIdx = choosen;
				cout << "Tracking: Vehicle " << currentVehicle->trackID << " re-found, blob idx: " << choosen << endl;
			}
		}
	}

	// Unassigned blobs are considered new vehicles
	for(vector<Blob>::iterator blobIt = blobs.begin(); blobIt!=blobs.end(); blobIt++)
	{
		if (blobIt->assignedVehicleCounter == 0)
		{
			TrackedVehicle *newVehicle = context->createTrackedVehicle(*blobIt);
			newVehicle->registerBlob(*blobIt, frameIdx, &src, foregroundFrame, verbose);
			newVehicle->lastAssociatedBlobIdx = blobIt->id;
			cout << "Tracking: New vehicle from blob " << blobIt->id << endl;
		}
	}

	// Killing overlapping vehicles (for every blob, only the (continually) oldest vehicle remains)
	vector<TrackedVehicle*> overlappingVehicles;
	for(vector<Blob>::iterator blobIt = blobs.begin(); blobIt!=blobs.end(); blobIt++)
	{
		if (blobIt->assignedVehicleCounter>=2)
		{
			cout << "Overlapping on blob " << blobIt->id << endl;

			// Find the vehicles choosing this blob
			overlappingVehicles.clear();
			int maxActivityFrameNumber = 0;
			TrackedVehicle *vehicleWithMaxActivityFrameNumber = NULL;

			for(map<unsigned int,TrackedVehicle*>::iterator it = context->trackedVehicles.begin(); it != context->trackedVehicles.end(); it++)
			{
				if (!it->second->isActive())
				{
					continue;
				}

				if (it->second->lastAssociatedBlobIdx == blobIt->id)
				{
					overlappingVehicles.push_back(it->second);
					if (it->second->continuousActiveFrameNumber > maxActivityFrameNumber)
					{
						maxActivityFrameNumber = it->second->continuousActiveFrameNumber;
						vehicleWithMaxActivityFrameNumber = it->second;
					}
				}
			}

			// Deactivate other vehicles
			for(vector<TrackedVehicle*>::iterator it=overlappingVehicles.begin(); it!=overlappingVehicles.end(); it++)
			{
				if ((*it) != vehicleWithMaxActivityFrameNumber)
				{
					(*it)->deactivate();
					cout << "Overlapping vehicle " << (*it)->trackID << " deactivated." << endl;
				}
			}
		}
	}

	// Aging vehicles
	for(map<unsigned int,TrackedVehicle*>::iterator it = context->trackedVehicles.begin(); it != context->trackedVehicles.end(); it++)
	{
		if (!it->second->isActive())
		{
			continue;
		}

		if (it->second->frameIdxOfLastRegistration == frameIdx)
		{
			// Detected in this frame
			it->second->continuousActiveFrameNumber++;
		}
		else
		{
			// Not detected in this frame
			it->second->continuousActiveFrameNumber = 0;
			it->second->inactiveFrameNumber++;
			int inactiveFrameNumber = it->second->inactiveFrameNumber;
			if (inactiveFrameNumber > configmanager.maxInactivityBeforeDeactivation )
			{
				cout << "Tracking: DEACTIVATED vehicle " << it->second->trackID << endl;
				it->second->deactivate();
			}
		}

	}
}

AdvancedTracker::AdvancedTracker(const char *configfilename)
{
	configmanager.init(configfilename);
	backgroundFrame = new Mat(480,640,CV_8UC1);
	foregroundFrame = new Mat(480,640,CV_8UC1);
	blurredSrc = new Mat(480,640,CV_8UC3);
	init();
}

Mat *AdvancedTracker::getCurrentForegroundImage()	// for debug
{
	return foregroundFrame;
}

Point operator+(Point &a, Point &b)
{
	return Point(a.x+b.x, a.y+b.y);
}

Point operator/(Point &a, int b)
{
	return Point(a.x/b, a.y/b);
}

Point AdvancedTracker::estimateNextLocation(TrackedVehicle &vehicle)
{
	Point estimatedLocation(0,0);

	int entryNumber = vehicle.locationRegistrations.size();

	// How long can we go back in time to still have continuous detections?
	int continuousDetectionLength = 0;	// default, will be checked below
	for(int i=0; i<3; i++)
	{
		if (entryNumber-i-2 >= 0)	// Do not overrun the vector size
		{
			if (vehicle.locationRegistrations[entryNumber-i-1].frameIdx == vehicle.locationRegistrations[entryNumber-i-2].frameIdx+1)
			{
				continuousDetectionLength = i+1;
			}
			else
			{
				break;
			}
		}
		else
		{
			break;
		}
	}

	Point speed(0,0);
	Point acceleration(0,0);

	if (continuousDetectionLength == 3)	// Use speed and acceleation
	{
		// Continuous detection in last 3 frames
		speed = vehicle.locationRegistrations[entryNumber-0-1].centroid - vehicle.locationRegistrations[entryNumber-1-1].centroid;
		Point speedPrevious = vehicle.locationRegistrations[entryNumber-1-1].centroid - vehicle.locationRegistrations[entryNumber-2-1].centroid;
		acceleration = speed - speedPrevious;
	}
	else if (continuousDetectionLength == 2)	// Use speed only
	{
		// Continuous detection in last 2 frames
		speed = vehicle.locationRegistrations[entryNumber-0-1].centroid - vehicle.locationRegistrations[entryNumber-1-1].centroid;
	}
	// TODO: check equation
	estimatedLocation = vehicle.locationRegistrations[entryNumber-0-1].centroid;
	estimatedLocation = estimatedLocation + speed;
	estimatedLocation = estimatedLocation + acceleration / 2;
	return estimatedLocation;
}

float AdvancedTracker::distance(Point a, Point b)
{
	return sqrt( (double)((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y)) );
}

int AdvancedTracker::findBlob(TrackedVehicle &vehicle, vector<Blob> &blobs, int frameIdx)
{
	int choosenBlobIdx = -1;
	float nearestDistance = 1000.0;

	if (blobs.size()==0)
	{
		// No blobs to choose from...
		return -1;
	}

	Point estimatedLocation = estimateNextLocation(vehicle);

	/* Choose the largets blob within distance limit "maxNearDistance".
		If there is none, but the nearest one is nearer than "maxAssociationDistance",
		choose that one. */

	int idxOfNearestBlob = -1;
	int distanceOfNearestBlob = 1000;
	int idxOflargetsBlobAmongNearOnes = -1;
	int areaOfLargetsBlobAmongNearOnes = 0;
	// Find nearest, largest blob to the estimated location
	for(vector<Blob>::iterator blobIterator = blobs.begin(); blobIterator != blobs.end(); blobIterator++)
	{
		// Calculate distance to estimatedLocation
		float dist = distance(estimatedLocation, blobIterator->centroid);

		// Find nearest
		if (distanceOfNearestBlob > dist)
		{
			// Nearer
			distanceOfNearestBlob = dist;
			idxOfNearestBlob = blobIterator - blobs.begin();
		}

		// If inside range, find largest among these
		if (dist <= configmanager.maxNearDistance)
		{
			// Find largest
			if (blobIterator->area > areaOfLargetsBlobAmongNearOnes)
			{
				// Nearer
				areaOfLargetsBlobAmongNearOnes = blobIterator->area;
				idxOflargetsBlobAmongNearOnes = blobIterator - blobs.begin();
			}

		}
	}

	if (idxOflargetsBlobAmongNearOnes != -1)
	{
		// Found one inside distance range
		choosenBlobIdx = idxOflargetsBlobAmongNearOnes;
	}
	else
	{
		// No near blobs. If the nearest is not too far away, choose that one
		if (distanceOfNearestBlob <= configmanager.maxAssociationDistance)
		{
			choosenBlobIdx = idxOfNearestBlob;
		}
	}

	return choosenBlobIdx;
}
