#ifndef __MOTIONVECTOR_H
#define __MOTIONVECTOR_H

#include <opencv2/core/core.hpp>

using namespace cv;

#define WEIGHTMAXDISTANCE	20.

class MotionVector
{
	Point src;
	Point dst;

public:
	// Default constructor used by loading from FileStorage
	MotionVector();

	MotionVector(Point src, Point dst);

	Point getSrc();

	Point getDst();

	enum SourceOrDestination
	{
		Source,
		Destination
	}; 

	double getLinearWeight(Point p1, Point p2);

	float length(Point2f v);

	double getDirectionSensitiveLinearWeight(Point2f otherV, float tangentialMultiplier=0.2);	// otherV: other velocity vector

	double getWeight(Point p, SourceOrDestination srcOrDest);

	double getWeight(Point p1, Point p2);

	double getDirectionSensitiveWeight(Point otherSrc, Point otherDst);

	void save(FileStorage *fs);

	void load(FileNode *node);
};

#endif
