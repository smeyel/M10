#ifndef __BLOB_H
#define __BLOB_H
#include <opencv/cv.h>

class Blob
{
public:
	Blob()
	{
		assignedVehicleCounter = 0;
	}

	int id;
	cv::Point centroid;
	unsigned int area;
	cv::Rect rect;

	int assignedVehicleCounter;
};

#endif