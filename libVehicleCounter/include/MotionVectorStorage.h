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

#define WEIGHTMAXDISTANCE	30.

class MotionVector
{
	Point src;
	Point dst;

public:
	// Default constructor used by loading from FileStorage
	MotionVector()
	{
	}

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
		double linearTerm = MAX( 1.0-(sqrDistance/WEIGHTMAXDISTANCE) , 0.);
//		double reciprocalTerm = (sqrDistance > 1 ? 1.0 / sqrDistance : 1);
		return linearTerm;
	}

	double getWeight(Point p1, Point p2)
	{
		double sqrDistance1 = sqrt((double)((src.x-p1.x)*(src.x-p1.x) + (src.y-p1.y)*(src.y-p1.y)));
		double sqrDistance2 = sqrt((double)((dst.x-p2.x)*(dst.x-p2.x) + (dst.y-p2.y)*(dst.y-p2.y)));
		double linearTerm1 = MAX( 1.0-(sqrDistance1/WEIGHTMAXDISTANCE) , 0.);
		double linearTerm2 = MAX( 1.0-(sqrDistance2/WEIGHTMAXDISTANCE) , 0.);
//		double reciprocalTerm = (sqrDistance > 1 ? 1.0 / sqrDistance : 1);
		return linearTerm1 * linearTerm2;
	}

	void save(FileStorage *fs)
	{
		*fs << "{" << "src" << src << "dst" << dst << "}";
	}

	void load(FileNode *node)
	{
		FileNode srcNode = (*node)["src"];
		src = Point(srcNode[0],srcNode[1]);
		FileNode dstNode = (*node)["dst"];
		dst = Point(dstNode[0],dstNode[1]);
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

	float getConfidence(Point prevLocation, Point currentLocation)
	{
		// TODO: use more complex confidence estimation than choosing the maximal value!
		// Otherwise, all matches with outliers will have maximal confidence.
		double maxWeight = 0.;
		for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
		{
/*			double prevLocationWeight = (*it)->getWeight(prevLocation,MotionVector::SourceOrDestination::Source);
			double currentLocationWeight = (*it)->getWeight(currentLocation,MotionVector::SourceOrDestination::Destination);
			double weight = prevLocationWeight * currentLocationWeight;*/
			double weight = (*it)->getWeight(prevLocation,currentLocation);

			if (weight > maxWeight && weight<1.)	// Exact match may be the outlier itself!
			{
				maxWeight = weight;
			}
		}
		return maxWeight;
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
			circle(*img,(*it)->getDst(),3,Scalar(0,0,color));
		}
	}
	
	void showAllMotionVectors(Mat *img, Scalar color)
	{
		for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
		{
			Point src = (*it)->getSrc();
			Point dst = (*it)->getDst();
//			cout << "Drawing (all) MotionVector: " << src.x << ";" << src.y << " - " << dst.x << ";" << dst.y << endl;
			line(*img,src,dst,color);
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

	void save(string filename)
	{
		FileStorage fs(filename,FileStorage::WRITE);

		fs << "motionvectorlist" << "[";
		for(unsigned int mvIdx=0;mvIdx<motionVectors.size();++mvIdx)
		{
			motionVectors[mvIdx]->save(&fs);
		}
		fs << "]";
		fs.release();
	}

	void load(string filename)
	{
		motionVectors.clear();

		FileStorage fs(filename,FileStorage::READ);
		ostringstream oss;
		FileNode mvFileNodes = fs["motionvectorlist"];
		cout << "Loading motionvectorlist (size=" << mvFileNodes.size() << ")" << endl;

		FileNodeIterator it = mvFileNodes.begin();
		FileNodeIterator it_end = mvFileNodes.end();

		for( ; it != it_end; ++it)
		{
			MotionVector *currentMotionVector = new MotionVector();
			currentMotionVector->load(&(*it));
			motionVectors.push_back(currentMotionVector);
		}

		fs.release();
	}
};

#endif