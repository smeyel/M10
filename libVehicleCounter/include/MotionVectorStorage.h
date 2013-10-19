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

#define WEIGHTMAXDISTANCE	20.

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

	double getLinearWeight(Point p1, Point p2)
	{
		double sqrDistance = sqrt((double)((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)));
		return MAX( 1.0-(sqrDistance/WEIGHTMAXDISTANCE) , 0.);
	}

	float length(Point2f v)
	{
		return (float)sqrt(v.x*v.x + v.y*v.y);
	}

	double getDirectionSensitiveLinearWeight(Point2f otherV, float tangentialMultiplier=0.2)	// otherV: other velocity vector
	{
		// Calculate unity vector corresponding the direction of (this->src;this->dst)
		Point2f currentV((float)(dst.x-src.x), (float)(dst.y-src.y));
		float currentLen = length(currentV);
		Point2f currentVUnity(currentV.x/currentLen, currentV.y/currentLen);

		float scalarProduct = (otherV.x * currentVUnity.x) + (otherV.y * currentVUnity.y);

		// Component of otherV tangential to currentV
		Point2f otherVTangential(currentVUnity.x * scalarProduct, currentVUnity.y * scalarProduct);

		// Component of otherV perpendicular to currentV
		Point2f otherVPerpendicular(otherV.x - otherVTangential.x, otherV.y - otherVTangential.y);

		Point2f tangentialDifference(otherVTangential.x-currentV.x,otherVTangential.y-currentV.y);
		// perpendicular difference is zero.

		float tangentialLength = length(tangentialDifference) * tangentialMultiplier;
		float perpendicularLength = length(otherVPerpendicular);

		double distance = length(Point2f(tangentialLength,perpendicularLength));
		return MAX( 1.0-(distance/WEIGHTMAXDISTANCE) , 0.);
	}

	double getWeight(Point p, SourceOrDestination srcOrDest)
	{
		Point p2 = (srcOrDest == Source ? src : dst);
		return getLinearWeight(p,p2);
	}

	double getWeight(Point p1, Point p2)
	{
		double linearTerm1 = getLinearWeight(src,p1);
		double linearTerm2 = getLinearWeight(dst,p2);
		return linearTerm1 * linearTerm2;
	}

	double getDirectionSensitiveWeight(Point otherSrc, Point otherDst)
	{
		// Weight of source point differences
		double srcWeight = getWeight(otherSrc,MotionVector::SourceOrDestination::Source);

		// Weight of speed vector differences
		Point2f otherV(otherDst.x-otherSrc.x,otherDst.y-otherSrc.y);
		double dstWeight = getDirectionSensitiveLinearWeight(otherV);

		double weight = srcWeight * dstWeight;

		return weight;
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
			double weight = (*it)->getDirectionSensitiveWeight(prevLocation,currentLocation);
			//double weight = (*it)->getWeight(prevLocation,currentLocation);

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