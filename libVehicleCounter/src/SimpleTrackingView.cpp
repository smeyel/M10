#include "SimpleTrackingView.h"

Point SimpleTrackingView::lastMouseClickLocation;

// Mouse callback to retrieve debug information at pixel locations
void mouse_callback(int eventtype, int x, int y, int flags, void *param)
{
	if (eventtype == CV_EVENT_LBUTTONDOWN)
	{
		SimpleTrackingView::lastMouseClickLocation = Point(x,y);
		cout << "Click location saved: " << x << ";" << y << endl;
	}
}

void SimpleTrackingView::init(const char *configFileName)
{
	configmanager.init(configFileName);

	// Prepare output video
	Size outputVideoSize(2*TrackedVehicle::fullImageSize.width,TrackedVehicle::fullImageSize.height);
	outputVideo = new VideoWriter(configmanager.outputVideoName,CV_FOURCC('M','J','P','G'), 25.0, outputVideoSize, true);
	if (!outputVideo->isOpened())
	{
		std::cout  << "Could not open the output video for write: " << configmanager.outputVideoName << endl;
		return;
	}
	Mat imageToRecord(TrackedVehicle::fullImageSize.height,2*TrackedVehicle::fullImageSize.width,CV_8UC3);
	Mat foregroundToRecord(480,640,CV_8UC3);	// Foreground mask (CV_8UC1) will be converted to BGR, into this Mat.

	if (configmanager.showSRC)
	{
		namedWindow("SRC", CV_WINDOW_AUTOSIZE);
		cvSetMouseCallback("SRC", mouse_callback);
	}
	if (configmanager.showBACK)
	{
		namedWindow("BACK", CV_WINDOW_AUTOSIZE);
		cvSetMouseCallback("BACK", mouse_callback);
	}
	if (configmanager.showFORE)
	{
		namedWindow("FORE", CV_WINDOW_AUTOSIZE);
		cvSetMouseCallback("FORE", mouse_callback);
	}
	namedWindow("VERBOSE", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("VERBOSE", mouse_callback);
}

void SimpleTrackingView::drawContourAsPolygon(Mat *img, vector<Point> contour, Scalar color)
{
	// create a pointer to the data as an array of points (via a conversion to 
	// a Mat() object)
	const cv::Point *pts = (const cv::Point*) Mat(contour).data;
	int npts = Mat(contour).rows;

	//std::cout << "Number of polygon vertices: " << npts << std::endl;
	
	// draw the polygon 
	//polylines(*img, &pts,&npts, 1, true, Scalar(0,255,0), 3);
	fillPoly(*img, &pts,&npts, 1, color);
}


void SimpleTrackingView::show(int frameIdx)
{
	if (configmanager.showFORE)
	{
		imshow("FORE",*this->tracker->getCurrentForegroundImage());
	}

	if (configmanager.showSRC)
	{
		imshow("SRC",*src);
	}

	if (configmanager.showBACK)
	{
		// Just for couriosity...
/*		backgroundSubtractor->getBackgroundImage(*backgroundFrame); */
/*		imshow("BACK",this->ba);*/
	}

	for(unsigned int i=0; i<context->trackedAreas.size(); i++)
	{
		context->trackedAreas[i].draw(verbose,Scalar(0,255,0),false);
	}

	if (configmanager.showAllMotionVectors)
	{
		context->motionVectorStorage.showAllMotionVectors(verbose,Scalar(255,0,0));
	}

	if (configmanager.showPath)
	{
		for(map<unsigned int,TrackedVehicle*>::iterator it = context->trackedVehicles.begin(); it != context->trackedVehicles.end(); it++)
		{
			context->showPath(*(*it).second,*verbose,true,false,false);
		}
	}

	// ------------ Show tracking information saved by the TrackedVehicle objects
	// Do it here and not in tracking (as verbose)
	context->exportLocationRegistrations(frameIdx,&locationRegistrationsOfCurrentFrame);
	for(vector<LocationRegistration*>::iterator it=locationRegistrationsOfCurrentFrame.begin();
		it != locationRegistrationsOfCurrentFrame.end(); it++)
	{
		drawLocationRegistration(*it,*verbose);
	}

	//context->exportPathOfAllVehicle(result);
	imshow("VERBOSE", *verbose);

	if (configmanager.recordVideo)
	{
		Mat foregroundToRecord(480,640,CV_8UC3);
		Mat left(imageToRecord, Rect(0, 0, 640, 480)); // Copy constructor
		cvtColor(*this->tracker->getCurrentForegroundImage(),foregroundToRecord,CV_GRAY2BGR);
		foregroundToRecord.copyTo(left);
		Mat right(imageToRecord, Rect(640, 0, 640, 480)); // Copy constructor
		result.copyTo(right);

		*outputVideo << imageToRecord;
	}
}

void SimpleTrackingView::drawLocationRegistration(LocationRegistration *locReg, Mat &verbose)
{
	// ---------- Visualize current results
	// Show motion vector prediction cloud for next location
	if (configmanager.showAllMotionVectors)
	{
		context->motionVectorStorage.showMotionVectorPredictionCloud(locReg->centroid,&verbose, 0.5);
	}

	Point upperleft(locReg->boundingBox.x, locReg->boundingBox.y);

	// Show size ratio
	if(configmanager.showSizeRatioBars)
	{
		line(verbose,
			Point(upperleft.x, upperleft.y-5),
			Point(upperleft.x + 50, upperleft.y-7),
			Scalar(255,0,0), 3);
		Scalar color(0,255,255);	// yellow be default
		if (locReg->sizeRatioToMean<0.8)
		{
			color = Scalar(0,255,0);	// Green (definitely small)
		}
		else if (locReg->sizeRatioToMean>1.4)
		{
			color = Scalar(0,0,255);	// Red (definitely big)
		}
		line(verbose,
			Point(upperleft.x, upperleft.y-5),
			Point(upperleft.x + (int)(locReg->sizeRatioToMean*50.0F), upperleft.y-5),
			color, 3);
	}



	// Show sumArea
/*	stringstream buffer;
	buffer << sumArea << ", R=" << sizeRatio;
	putText(*manager->currentVerboseImage, buffer.str(), cvPoint(upperLeft.x + 5, upperLeft.y + 5),
		FONT_HERSHEY_DUPLEX, 0.5, Scalar(255,255,255)); */

/*	cvPutText(manager->currentVerboseImage,
		buffer.str().c_str(), cvPoint(upperLeft.x + 5, upperLeft.y + 5), &font, CV_RGB(255.,255.,255.)); */

	// Show mean bounding box at this location
	if (configmanager.showMeanSizeAtLocation)
	{
		Size meanSize = context->sizeStorage.getMeanSize(locReg->centroid,locReg->lastSpeedVector);
		Rect meanRect(
			locReg->centroid.x - meanSize.width/2, locReg->centroid.y - meanSize.height/2, 
			meanSize.width, meanSize.height);
		rectangle(verbose,meanRect,Scalar(100,100,100));
	}

	Scalar color = Scalar(255,255,255);
	// Show detection rectangle
	if (configmanager.showBoundingBox)
	{
		rectangle(verbose,locReg->boundingBox,color);
	}
	// Show trackID
	if (configmanager.showTrackId)
	{
		stringstream oss;
		oss << locReg->trackID;
		putText(verbose,
			oss.str().c_str(),
			locReg->centroid, CV_FONT_HERSHEY_SIMPLEX, 0.5, color,1,8,false);
	}
}

void SimpleTrackingView::verboseMeanSizeAtLastClickLocation()
{
	context->sizeStorage.verboseMeanSizeAtLocation(SimpleTrackingView::lastMouseClickLocation);
}
