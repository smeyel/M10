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
		return MAX(1.0-(sqrDistance/50.0), 1.0 / sqrDistance);
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