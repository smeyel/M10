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

void TrackedVehicle::registerDetection(unsigned int frameIdx, cvb::CvTrack *currentDetectingCvTrack, Mat *srcImg, Mat *foregroundImg, Mat *verboseImage)
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
	locationRegistrations.push_back(registration);

	// ---------- Visualize current results
	// Show motion vector prediction cloud for next location
	context->motionVectorStorage.showMotionVectorPredictionCloud(centroid,verboseImage, 0.5);

	// Show size ratio
	line(*verboseImage,
		Point(upperLeft.x, upperLeft.y-5),
		Point(upperLeft.x + 50, upperLeft.y-7),
		Scalar(255,0,0), 3);
	Scalar color(0,255,255);	// yellow be default
	if (sizeRatio<0.8)
	{
		color = Scalar(0,255,0);	// Green (definitely small)
	}
	else if (sizeRatio>1.4)
	{
		color = Scalar(0,0,255);	// Red (definitely big)
	}

	line(*verboseImage,
		Point(upperLeft.x, upperLeft.y-5),
		Point(upperLeft.x + (int)(sizeRatio*50.0F), upperLeft.y-5),
		color, 3);


	// Show sumArea
/*	stringstream buffer;
	buffer << sumArea << ", R=" << sizeRatio;
	putText(*manager->currentVerboseImage, buffer.str(), cvPoint(upperLeft.x + 5, upperLeft.y + 5),
		FONT_HERSHEY_DUPLEX, 0.5, Scalar(255,255,255)); */

/*	cvPutText(manager->currentVerboseImage,
		buffer.str().c_str(), cvPoint(upperLeft.x + 5, upperLeft.y + 5), &font, CV_RGB(255.,255.,255.)); */

	// Show mean bounding box at this location
	Size meanSize = context->sizeStorage.getMeanSize(centroid,speed);
	Rect meanRect(
		centroid.x - meanSize.width/2, centroid.y - meanSize.height/2, 
		meanSize.width, meanSize.height);
	rectangle(*verboseImage,meanRect,Scalar(100,100,100));

}

void TrackedVehicle::exportAllDetections(float minConfidence)
{
	recalculateLocationConfidences();

	// ------------ Area hit exports

	// Go along all locations and check for area hits
	vector<unsigned int> trackedAreaHits;
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		if (it->confidence >= minConfidence)
		{
			checkForAreaIntersections(*it,trackedAreaHits,minConfidence);
		}
	}
	// Tidy-up this list and create a final "which direction did it go to" description.
	vector<unsigned int> cleanedTrackedAreaHits;
	int runlength = 0;
	int lastAreaIdx = -1;
	for(vector<unsigned int>::iterator it=trackedAreaHits.begin(); it!=trackedAreaHits.end(); it++)
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
	// Export area hits
	context->measurementExport->areaHitOutput
		<< this->trackID;
	for(vector<unsigned int>::iterator it=trackedAreaHits.begin(); it!=trackedAreaHits.end(); it++)
//	for(vector<unsigned int>::iterator it=cleanedTrackedAreaHits.begin(); it!=cleanedTrackedAreaHits.end(); it++)
	{
		context->measurementExport->areaHitOutput << ";" << *it;
	}
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

void TrackedVehicle::checkForAreaIntersections(LocationRegistration &registration, vector<unsigned int> &areaHitList, float minConfidence)
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

TrackedVehicle::TrackedVehicle(unsigned int iTrackID, TrackingContext *context)
{
	this->context = context;
	trackID = iTrackID;
}

// DEPRECATED
vector<unsigned int> TrackedVehicle::exportAllAreaHits(float minConfidence)
{
	vector<unsigned int> resultList;
	// Go along all locations and check for area hits
	for(vector<LocationRegistration>::iterator it=locationRegistrations.begin(); it!=locationRegistrations.end(); it++)
	{
		if (it->confidence >= minConfidence)
		{
			checkForAreaIntersections(*it,resultList,minConfidence);
		}
	}
	return resultList;
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

/*std::ostream& operator<<(std::ostream& output, TrackedVehicle &trackedVehicle)
{
	output << "Detections for trackID=" << trackedVehicle.trackID << ":" << endl << "\t";
	for(vector<unsigned int>::iterator it=trackedVehicle.areaHits.begin(); it!=trackedVehicle.areaHits.end(); it++)
	{
		output << *it << " ";
	}
	output << endl;
	return output;
} */
