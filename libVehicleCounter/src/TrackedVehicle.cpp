#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "TrackedVehicle.h"
#include "TrackingContext.h"

#define DEFAULTCONFIDENCE 0.1F

Size TrackedVehicle::fullImageSize;	// static field

Rect TrackedVehicle::getNarrowBoundingBox(Mat &originalForeground, Rect originalRect)
{
	// Get size of object (tries to ignore shadows which were useful to improve tracking accuracy)
	cv::Mat roi = originalForeground(originalRect);	// does not copy!
	cv::Mat foreground(originalRect.height, originalRect.width, CV_8UC1);
	roi.copyTo(foreground);
	cv::compare(foreground,Scalar(200),foreground,CMP_GT);	// WRN my overwrite original foreground if not copies!
	Mat integralImage(foreground.rows, foreground.cols, CV_32SC1);
	cv::integral(foreground,integralImage);
	int lastRow = foreground.rows-1;
	int lastCol = foreground.cols-1;
	int column10percent = (foreground.rows / 10) * 255;
	int row10percent = (foreground.cols / 10) * 255;
	int all = integralImage.at<int>(lastRow,lastCol);
	int column90percent = all - column10percent;
	int row90percent = all - row10percent;
	Point newUpperLeft = Point(0,0);
	Point newLowerRight = Point(lastCol, lastRow);
	int i;
	// Shift newUpperLeft
	for(i=0;
		integralImage.at<int>(lastRow,i)<column10percent && i<lastCol;
		i++);
	newUpperLeft.x = i;
	for(i=0;
		integralImage.at<int>(i,lastCol)<row10percent && i<lastRow;
		i++);
	newUpperLeft.y = i;
	// Shift newLowerRight
	for(i=lastCol;
		integralImage.at<int>(lastRow,i)>column90percent && i>0;
		i--);
	newLowerRight.x = i;
	for(i=lastRow;
		integralImage.at<int>(i,lastCol)>row90percent && i>0;
		i--);
	newLowerRight.y = i;

	Rect narrowBoundingBox(
		originalRect.x+newUpperLeft.x,
		originalRect.y+newUpperLeft.y,
		newLowerRight.x-newUpperLeft.x,
		newLowerRight.y-newUpperLeft.y);
	return narrowBoundingBox;
}

void TrackedVehicle::registerDetection(int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *srcImg, Mat *foregroundImg, Mat *verboseImage)
{
	// ---------- Calculate current results
	Point centroid((int)currentDetectingCvTrack->centroid.x,(int)currentDetectingCvTrack->centroid.y);
	Point upperLeft(currentDetectingCvTrack->minx, currentDetectingCvTrack->miny);
	Size size(currentDetectingCvTrack->maxx - currentDetectingCvTrack->minx, currentDetectingCvTrack->maxy - currentDetectingCvTrack->miny);
	Rect rect(upperLeft,size);

	// Estimate detection confidence
	//	Last location: last element of locationRegistrations
	//	Current location: centroid
	float confidence = DEFAULTCONFIDENCE;	// Default confidence
	if (locationRegistrations.size() > 0)
	{
		Point prevLocation = (locationRegistrations.end()-1)->centroid;
		confidence = this->context->motionVectorStorage.getConfidence(prevLocation,centroid);
		//cout << "Confidence of new location: " << confidence << endl;
	}

	// Calculate real area
	/*cv::Mat mask = (*manager->currentForegroundMask)(rect);
	Mat maskThresholded = mask > 200;
	int sumArea = cv::countNonZero(maskThresholded);
	cout << "Car SumArea=" << sumArea << endl; */

	// Calculate current speed vector
	Point speed = Point(0,0);
	if (locationRegistrations.size()>0)
	{
		vector<LocationRegistration>::iterator it = locationRegistrations.end() - 1;
		if (it->frameIdx+1 == frameIdx)
		{
			speed = Point(centroid.x - it->centroid.x, centroid.y - it->centroid.y);
		}
	}
//	cout << "Current speed: " << speed.x << "," << speed.y << endl;
	
	// Get size information
	context->sizeStorage.add(centroid,speed,size);
	float sizeRatio = context->sizeStorage.getAreaRatioToMean(centroid,speed,size);
	//cout << "CarID " << this->trackID << ": size ratio to mean: " << sizeRatio << endl;

	// ---------- Store/export current results
	// Save images
	string srcImgRoiFilename;
	string foreImgRoiFilename;
	if (srcImg)
	{
		srcImgRoiFilename = context->measurementExport->saveimage(this->trackID,"S",frameIdx,*srcImg,rect);
	}
	if (foregroundImg)
	{
		foreImgRoiFilename = context->measurementExport->saveimage(this->trackID,"M",frameIdx,*foregroundImg,rect);
	}

	// Get narrow bounding box (size)
	Rect narrowBoundingBox = getNarrowBoundingBox(*foregroundImg,rect);
	rectangle(*verboseImage,narrowBoundingBox,Scalar(255,255,255));

	// Store registration data
	LocationRegistration registration;
	registration.frameIdx = frameIdx;
	registration.confidence = confidence;
	registration.centroid = centroid;
	registration.bigBoundingBox = rect;
	registration.boundingBox = narrowBoundingBox;
	registration.sizeRatioToMean = sizeRatio;
	registration.srcImageFilename = srcImgRoiFilename;
	registration.maskImageFilename = foreImgRoiFilename;
	registration.lastSpeedVector = speed;
	registration.areaHitIdx = -2;	// Not checked yet...
	locationRegistrations.push_back(registration);
}

void TrackedVehicle::exportAllDetections(float minConfidence)	// areaHits and LocationRegistrations are written to context->measurementExport
{
	recalculateLocationConfidences();

	// ------------ Area hit exports

	// Go along all locations and check for area hits
	vector<int> trackedAreaHits;
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		if (it->confidence >= minConfidence)
		{
			checkForAreaIntersections(*it,trackedAreaHits,minConfidence);
		}
	}

	// Tidy-up this list and create a final "which direction did it go to" description.
	vector<int> cleanedTrackedAreaHits;
	int runlength = 0;
	int lastAreaIdx = -1;
	for(vector<int>::iterator it=trackedAreaHits.begin(); it!=trackedAreaHits.end(); it++)
	{
		// Save value exactly at 2nd occurrance.
		//	This way, last homogeneous sequence does not have to be checked after the for loop.
		if (*it == lastAreaIdx)
		{
			runlength++;
			if (runlength == 2)
			{
				// Save it now
				cleanedTrackedAreaHits.push_back(*it);
			}
		}
		else
		{
			runlength=1;
			lastAreaIdx = *it;
		}
	}

	// Export area hits (raw hit data, not the cleaned one)
	context->measurementExport->areaHitOutput
		<< this->trackID;

	int id = context->pathValidator.getPathIdIfValid(cleanedTrackedAreaHits);
	if (id != -1)
	{
		context->measurementExport->areaHitOutput << " PathID=" << id << " ";
	}
	else
	{
		context->measurementExport->areaHitOutput << " INVALID ";
	}

	for(vector<int>::iterator it=trackedAreaHits.begin(); it!=trackedAreaHits.end(); it++)
	{
		context->measurementExport->areaHitOutput << ";" << *it;
	}
	// Validation (using the cleaned path)
	context->measurementExport->areaHitOutput << endl;

	// ------------ Registration exports

	// Export all LocationRegistrations
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		context->measurementExport->detectionOutput
			<< this->trackID << ";"
			<< it->frameIdx << ";"
			<< it->boundingBox.x << ";" << it->boundingBox.y << ";"
			<< it->boundingBox.width << ";" << it->boundingBox.height << ";"
			<< it->confidence << ";"	// new field!
			<< it->sizeRatioToMean << ";"	// new field!
			<< it->srcImageFilename << ";"
			<< it->maskImageFilename << endl;
	}
}


void TrackedVehicle::feedMotionVectorsIntoMotionVectorStorage(float minConfidence)
{
	OPENCV_ASSERT(context,"TrackedVehicle::feedMotionVectorsIntoMotionVectorStorage","context not set prior to export");
	// use locationRegistrations
	unsigned int lastFrameIdx = -2;
	Point lastLocation;
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != locationRegistrations.end(); it++)
	{
		if ((*it).frameIdx - lastFrameIdx == 1)	// For consecutive locations only
		{
			if ((*it).confidence >= minConfidence)	// With sufficient confidence
			{
				context->motionVectorStorage.addMotionVector(lastLocation,(*it).centroid);
			}
		}
		lastLocation = (*it).centroid;
		lastFrameIdx = (*it).frameIdx;
	}
}

bool TrackedVehicle::isIntersecting(cvb::CvTrack *track, Area *area)
{
	Rect rect(track->minx, track->miny, track->maxx - track->minx, track->maxy - track->miny);
	return area->isRectangleIntersecting(rect);
}

bool TrackedVehicle::isIntersecting(LocationRegistration &registration, Area *area)
{
/*	return area->isPointInside(Point(
		registration.boundingBox.x + registration.boundingBox.width/2,
		registration.boundingBox.y + registration.boundingBox.height));	// Checked point is the bottom middle point.*/
	return area->isPointInside(registration.centroid);
}

void TrackedVehicle::checkForAreaIntersections(LocationRegistration &registration, vector<int> &areaHitList, float minConfidence)
{
	// Register area hits
	for(unsigned int areaIdx=0; areaIdx<context->trackedAreas.size(); areaIdx++)
	{
		if (isIntersecting(registration, &(context->trackedAreas)[areaIdx]))
		{
/*			cout << (currentDetectingCvTrack->inactive ? "inactive " : "  active ");
			cout << "CAR " << trackID << " in AREA " << (*manager->trackedAreas)[areaIdx].id << endl;*/

			areaHitList.push_back(areaIdx);
		}
	}
}

TrackedVehicle::TrackedVehicle(int iTrackID, TrackingContext *context)
{
	this->context = context;
	trackID = iTrackID;
}

void TrackedVehicle::recalculateLocationConfidences()
{
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it != (locationRegistrations.end()-1); it++)
	{
		Point p1 = (*(it)).centroid;
		Point p2 = (*(it+1)).centroid;
		(*(it+1)).confidence = context->motionVectorStorage.getConfidence(p1,p2);
	}
}

void TrackedVehicle::exportLocationRegistrations(int frameIdx, vector<LocationRegistration*> *targetVector)
{
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		if (it->frameIdx == frameIdx)
		{
			targetVector->push_back(&(*it));
		}
	}
}

void TrackedVehicle::save(FileStorage *fs)
{
//	ostringstream oss;
//	oss << trackID;
	*fs << "{";
	*fs	<< "trackID" << trackID; //oss.str();
	*fs	<< "locationRegistrations" << "[";
	for(unsigned int idx=0; idx<locationRegistrations.size(); idx++)
	{
		*fs << "{:"
			<< "frameIdx" << locationRegistrations[idx].frameIdx
			<< "confidence" << locationRegistrations[idx].confidence
			<< "centroid" << locationRegistrations[idx].centroid
			<< "bigBoundingBox" << locationRegistrations[idx].bigBoundingBox
			<< "boundingBox" << locationRegistrations[idx].boundingBox
			<< "sizeRatioToMean" << locationRegistrations[idx].sizeRatioToMean
			<< "srcImageFilename" << locationRegistrations[idx].srcImageFilename
			<< "maskImageFilename" << locationRegistrations[idx].maskImageFilename
			<< "lastSpeedVector" << locationRegistrations[idx].lastSpeedVector
			<< "areaHitIdx" << locationRegistrations[idx].areaHitIdx
			<< "}";
	}
	*fs << "]"
		<< "}";
}

void TrackedVehicle::load(FileNode *node)
{
	(*node)["trackID"] >> trackID;

	FileNode pointFileNodes = (*node)["locationRegistrations"];
	FileNodeIterator it = pointFileNodes.begin();
	FileNodeIterator it_end = pointFileNodes.end();

	locationRegistrations.clear();
	for( ; it != it_end; ++it)
	{
		LocationRegistration lr;
		(*it)["frameIdx"] >> lr.frameIdx;
		(*it)["confidence"] >> lr.confidence;
		loadPoint((*it)["centroid"],lr.centroid);
		loadRect((*it)["bigBoundingBox"],lr.bigBoundingBox);
		loadRect((*it)["boundingBox"],lr.boundingBox);
		
/*		(*it)["centroid"] >> lr.centroid;
		(*it)["bigBoundingBox"] >> lr.bigBoundingBox;
		(*it)["boundingBox"] >> lr.boundingBox;*/
		(*it)["sizeRatioToMean"] >> lr.sizeRatioToMean;
		(*it)["srcImageFilename"] >> lr.srcImageFilename;
		(*it)["maskImageFilename"] >> lr.maskImageFilename;
//		(*it)["lastSpeedVector"] >> lr.lastSpeedVector;
		loadPoint((*it)["lastSpeedVector"],lr.lastSpeedVector);
		(*it)["areaHitIdx"] >> lr.areaHitIdx;
		locationRegistrations.push_back(lr);
	}
}

void TrackedVehicle::loadPoint(cv::FileNode& node, cv::Point &point)
{
	node[0] >> point.x;
	node[1] >> point.y;
}

void TrackedVehicle::loadRect(cv::FileNode& node, cv::Rect &rect)
{
	node[0] >> rect.x;
	node[1] >> rect.y;
	node[2] >> rect.width;
	node[3] >> rect.height;
}

/*cv::FileNode& operator>>(cv::FileNode& node, cv::Point &point)
{
	node[0] >> point.x;
	node[1] >> point.y;
	return node;
}*/
