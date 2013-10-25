#ifndef __VEHICLESIZESTORAGE_H
#define __VEHICLESIZESTORAGE_H

#include <vector>
//#include <cmath>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class VehicleSizeStorage
{
	struct VehicleSizeEntry
	{
		Point centroid;
		Point speed;
		Size size;
	};

	vector<VehicleSizeEntry> sizes;

	float distance(Point p1, Point p2);
	float getDirectionAbsDifferenceDeg(Point v1, Point v2);
	int getArea(Size s);

	float getMeanArea(Point p, Point speed = Point(0,0));

public:
	void add(Point centroid, Point speed, Size size);
	void clear();

	Size getMeanSize(Point p, Point speed = Point(0,0));

	// 0.0F if there is no sufficient data
	float getAreaRatioToMean(Point currentLocation, Point currentSpeed, Size currentSize);

	// Debug function
	void verboseMeanSizeAtLocation(Point p);
};

#endif
