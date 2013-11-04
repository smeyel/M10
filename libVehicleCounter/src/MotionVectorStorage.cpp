#include "MotionVectorStorage.h"

void MotionVectorStorage::addMotionVector(Point src, Point dst)
{
	MotionVector *mv = new MotionVector(src,dst);
	motionVectors.push_back(mv);
}

float MotionVectorStorage::getConfidence(Point prevLocation, Point currentLocation)
{
	if (motionVectors.size() == 0)
	{
		cout << "WRN: Empty MotionVectorStorage, confidence is set 0.0F" << endl;
		return 0.0F;
	}

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

MotionVectorStorage::~MotionVectorStorage()
{
	clear();
}

void MotionVectorStorage::showMotionVectorPredictionCloud(Point sourceLocation, Mat *img, double minimalWeight)
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
	
void MotionVectorStorage::showAllMotionVectors(Mat *img, Scalar color)
{
	for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
	{
		Point src = (*it)->getSrc();
		Point dst = (*it)->getDst();
//			cout << "Drawing (all) MotionVector: " << src.x << ";" << src.y << " - " << dst.x << ";" << dst.y << endl;
		line(*img,src,dst,color);
	}
}

void MotionVectorStorage::clear()
{
	for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
	{
		delete *it;
	}
	motionVectors.clear();
}

void MotionVectorStorage::save(string filename)
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

void MotionVectorStorage::load(string filename)
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

float MotionVectorStorage::getMeanMotionVectorLength()
{
	float sumLen = 0.0F;
	float num = 0.0F;

	for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
	{
		sumLen += (*it)->length();
		num++;
	}

	return sumLen / num;
}
