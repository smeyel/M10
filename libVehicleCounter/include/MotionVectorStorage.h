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
	float minConfidenceToSkipAddingNewMotionVector;
	bool collectNewMotionVectors;

	~MotionVectorStorage();
	void addMotionVector(Point src, Point dst);

	float getConfidence(Point prevLocation, Point currentLocation);

	/** Consolidate internal representation to accelerate confidence queries, remove outliers */
	void consolidate(float outlierMaxConfidence, float overlapMinConfidence);

	// ------------- Visualization

	void showMotionVectorPredictionCloud(Point sourceLocation, Mat *img, double minimalWeight=0.01);
	
	void showAllMotionVectors(Mat *img, Scalar color);

	// ------------- Storage, persistance

	void clear();

	void save(string filename);

	void load(string filename);

	// ------------- Debug, info

	float getMeanMotionVectorLength();
};

#endif
