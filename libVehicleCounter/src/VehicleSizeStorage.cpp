#include <iostream>	// for debug only
#include "VehicleSizeStorage.h"

float VehicleSizeStorage::distance(Point p1, Point p2)
{
	return (float)sqrt( (double)((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) ));
}

void VehicleSizeStorage::add(Point centroid, Point speed, Size size)
{
	VehicleSizeEntry entry;
	entry.centroid = centroid;
	entry.speed = speed;
	entry.size = size;
	sizes.push_back(entry);
}

void VehicleSizeStorage::clear()
{
	sizes.clear();
}

float VehicleSizeStorage::getDirectionAbsDifferenceDeg(Point v1, Point v2)
{
	float deg1 = atan2((float)v1.y,(float)v1.x);
	float deg2 = atan2((float)v2.y,(float)v2.x);
	float absDiffDeg = abs(deg2-deg1) / 3.142592654 * 180.0F;
	return (absDiffDeg < 180.0F) ? absDiffDeg : (360.0F-absDiffDeg);
}

Size VehicleSizeStorage::getMeanSize(Point p, Point speed)
{
	float resultW=0;
	float resultH=0;
	float sumWeight = 0.0F;
	int num=0;
	bool usingDirection = !(speed.x == 0 && speed.y == 0);	// if speed is not null vector

	int debug_idx = -1;
	for(vector<VehicleSizeEntry>::iterator it=sizes.begin(); it!=sizes.end(); it++)
	{
		debug_idx++;
		bool isNullVector = (it->speed.x == 0 && it->speed.y == 0);
		if (usingDirection && isNullVector)
		{
			// Speed is given, but this entry does not have it...
			continue;
		}

		// Calculate distance-dependent weighted average
		float dist = distance(p,it->centroid);
		float weight = 0.0F;
		if (dist > 1.0F)
		{
			weight = 1.0F / dist;
		}
		else
		{
			weight = 1.0F;
		}

		// If the speed is given, only entries with <=90deg direction differences are considered
		bool showWeight = false;
		if (usingDirection && !isNullVector)
		{
			float dirDiffDeg = getDirectionAbsDifferenceDeg(speed, it->speed);
			if (dirDiffDeg > 45.0F)
			{
//				cout << "getMeanSize: Idx" << debug_idx << " dir " << dirDiffDeg << " degs, NO. W=" << weight << endl;
				continue;
			}
			cout << "getMeanSize: Idx" << debug_idx << " dirDiff " << dirDiffDeg << " degs, OK. W=" << weight << endl;
		}


		resultW += weight * (float)(it->size.width);
		resultH += weight * (float)(it->size.height);
		sumWeight += weight;
		num++;
	}

	if (num==0 || sumWeight < 0.0001F)
	{
		return Size(0,0);
	}

	resultW /= sumWeight;
	resultH /= sumWeight;

	//std::cout << "Meansize: " << resultW << "/" << resultH << endl;

	return Size((int)floor(resultW),(int)floor(resultH));
}

int VehicleSizeStorage::getArea(Size s)
{
	return s.width * s.height;
}

float VehicleSizeStorage::getMeanArea(Point p, Point speed)
{
	Size meanSize = getMeanSize(p,speed);
	return (float)(getArea(meanSize));
}

float VehicleSizeStorage::getAreaRatioToMean(Point currentLocation, Point speed, Size currentSize)
{
	float meanArea = getMeanArea(currentLocation,speed);
	float currentArea = getArea(currentSize);
	if (meanArea < 0.001F)	// Not enough sample (that returns 0.0F)
	{
		return 0.0F;
	}
	return currentArea / meanArea;
}

void VehicleSizeStorage::verboseMeanSizeAtLocation(Point p)
{
	Point speed;
	cout << "Mean size dump at location " << p.x << "/" << p.y << endl;
	for(float dirDeg=0.0F; dirDeg<360.0F; dirDeg+=45.0F)
	{
		speed = Point( (int)(10.0F * sin( dirDeg / 180.0 * 3.1416 )), (int)(10.0F * cos( dirDeg / 180.0 * 3.1416 )) );
		Size meanSize = getMeanSize(p,speed);
		cout << "  dir " << dirDeg << " deg, speed vector " << speed.x << "/" << speed.y << ": size=" << meanSize.width << "/" << meanSize.height << "=" << ((float)meanSize.width/(float)meanSize.height) << endl;
	}
}
