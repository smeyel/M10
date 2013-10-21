#include <iostream>	// for debug only
#include "VehicleSizeStorage.h"

float VehicleSizeStorage::distance(Point p1, Point p2)
{
	return (float)sqrt( (double)((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) ));
}

void VehicleSizeStorage::add(Point p, Size s)
{
	measurements.push_back(pair<Point, Size>(p,s));
}

void VehicleSizeStorage::clear()
{
	measurements.clear();
}

Size VehicleSizeStorage::getMeanSize(Point p)
{
	float resultW=0;
	float resultH=0;
	float sumWeight = 0.0F;
	int num=0;

	for(vector<pair<Point, Size>>::iterator it=measurements.begin(); it!=measurements.end(); it++)
	{
		float dist = distance(p,it->first);
		float weight = 0.0F;
		if (dist > 1.0F)
		{
			weight = 1.0F / dist;
		}
		else
		{
			weight = 1.0F;
		}

		resultW += weight * (float)(it->second.width);
		resultH += weight * (float)(it->second.height);
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

float VehicleSizeStorage::getMeanArea(Point p)
{
	Size meanSize = getMeanSize(p);
	return (float)(getArea(meanSize));
}

float VehicleSizeStorage::getAreaRatioToMean(Point currentLocation, Size currentSize)
{
	float meanArea = getMeanArea(currentLocation);
	float currentArea = getArea(currentSize);
	if (meanArea < 0.001F)	// Not enough sample
	{
		return 0.0F;
	}
	return currentArea / meanArea;
}
