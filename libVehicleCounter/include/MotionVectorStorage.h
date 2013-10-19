#ifndef __MOTIONVECTORSTORAGE_H
#define __MOTIONVECTORSTORAGE_H

#include <cmath>
#include <vector>
#include <opencv2/core/core.hpp>

#include <cvblob.h>

using namespace cv;
using namespace std;

#ifndef MAX
#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

class MotionVector
{
	Point src;
	Point dst;
public:

	MotionVector(Point src, Point dst)
	{
		this->src = src;
		this->dst = dst;
	}

	Point getSrc()
	{
		return src;
	}

	Point getDst()
	{
		return dst;
	}

	enum SourceOrDestination
	{
		Source,
		Destination
	};

	double getWeight(Point p, SourceOrDestination srcOrDest)
	{
		Point p2 = (srcOrDest == Source ? src : dst);
		double sqrDistance = sqrt((double)((p.x-p2.x)*(p.x-p2.x) + (p.y-p2.y)*(p.y-p2.y)));
		return MAX(1.0-(sqrDistance/50.0), 1.0 / sqrDistance);
	}
};


class MotionVectorStorage
{
	vector<MotionVector *> motionVectors;
public:
	void addMotionVector(Point src, Point dst)
	{
		MotionVector *mv = new MotionVector(src,dst);
		motionVectors.push_back(mv);
	}

	~MotionVectorStorage()
	{
		clear();
	}

	void showMotionVectorPredictionCloud(Point sourceLocation, Mat *img, double minimalWeight=0.01)
	{
		for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
		{
			double weight = (*it)->getWeight(sourceLocation,MotionVector::SourceOrDestination::Source);
			if (weight < minimalWeight)
			{
				continue;
			}
			int color = (int)(255 * weight);
			circle(*img,(*it)->getDst(),3,Scalar(color,color,color));
		}
	}

	void showAllMotionVectors(Mat *img)
	{
		for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
		{
			Point src = (*it)->getSrc();
			Point dst = (*it)->getDst();
//			cout << "Drawing (all) MotionVector: " << src.x << ";" << src.y << " - " << dst.x << ";" << dst.y << endl;
			line(*img,src,dst,Scalar(255,255,255));
		}
	}

	void clear()
	{
		for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
		{
			delete *it;
		}
		motionVectors.clear();
	}

	void getMostRelevantBlob(Point origin, cvb::CvBlobs blobs)
	{
		
	}


};


#endif