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

void SimpleTrackingView::verboseMeanSizeAtLastClickLocation()
{
	context->sizeStorage.verboseMeanSizeAtLocation(SimpleTrackingView::lastMouseClickLocation);
}
