#include "MotionVectorStorage.h"

void MotionVectorStorage::addMotionVector(Point src, Point dst)
{
	if (!collectNewMotionVectors)
	{
		return;
	}

	float confidence = getConfidence(src,dst);
	if (confidence < minConfidenceToSkipAddingNewMotionVector)
	{
		MotionVector *mv = new MotionVector(src,dst);
		motionVectors.push_back(mv);
	}
/*	else
	{
		cout << "New MotionVector not added due to already high confidence: " << confidence << endl;
	} */
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
	float maxWeight = 0.;
	for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
	{
		// Do not take totally equal motion vectors into account
		if ((*it)->isTheSame(prevLocation,currentLocation))
		{
			continue;
		}

		double weight = (*it)->getDirectionSensitiveWeight(prevLocation,currentLocation);
		//double weight = (*it)->getWeight(prevLocation,currentLocation);

		if (weight > maxWeight)
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


void MotionVectorStorage::consolidate(float outlierMaxConfidence, float overlapMinConfidence)
{
	cout << "Number of MotionVectors before consolidation: " << motionVectors.size() << endl;
	int tmpCounter = 0;

	// Remove outliers and overlaps
	for(vector<MotionVector *>::iterator it=motionVectors.begin(); it!=motionVectors.end(); it++)
	{
		float currentMotionVectorConfidence = (*it)->getConfidence(this);
		bool toRemove = false;
		if (currentMotionVectorConfidence <= outlierMaxConfidence)
		{
			toRemove = true;
		}
		if (currentMotionVectorConfidence >= overlapMinConfidence)
		{
			toRemove = true;
		}

		if (toRemove)
		{
			delete *it;
			motionVectors.erase(it++);
		}
		if (++tmpCounter % 1000 == 0)
		{
			cout << "Progress: " << tmpCounter << endl;
		}
	}

	cout << "Number of MotionVectors after consolidation: " << motionVectors.size() << endl;
}
