#ifndef __VEHICLESIZESTORAGE_H
#define __VEHICLESIZESTORAGE_H

#include <vector>
//#include <cmath>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class VehicleSizeStorage
{
	vector<pair<Point, Size>> measurements;

	float distance(Point p1, Point p2);
	Size getMeanSize(Point p);
	int getArea(Size s);
	float getMeanArea(Point p);

public:
	void add(Point p, Size s);
	void clear();
	// 0.0F if there is no sufficient data
	float getAreaRatioToMean(Point currentLocation, Size currentSize);
};

#endif
