#ifndef __MOTIONVECTORSTORAGE_H
#define __MOTIONVECTORSTORAGE_H

#include <cmath>
#include <vector>
#include <opencv2/core/core.hpp>

#include <cvblob.h>

#include "MotionVector.h"

using namespace cv;
using namespace std;

class MotionVectorStorage
{
	vector<MotionVector *> motionVectors;
public:
	void addMotionVector(Point src, Point dst);

	float getConfidence(Point prevLocation, Point currentLocation);

	~MotionVectorStorage();

	void showMotionVectorPredictionCloud(Point sourceLocation, Mat *img, double minimalWeight=0.01);
	
	void showAllMotionVectors(Mat *img, Scalar color);

	void clear();

	void save(string filename);

	void load(string filename);
};

#endif
