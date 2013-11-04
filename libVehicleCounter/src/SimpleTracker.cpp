#include "SimpleTracker.h"

void SimpleTracker::init()
{
//	vector<cvb::CvTracks> currentTrackList;

	cvblob.minBlobArea = configmanager.minBlobArea;
	cvblob.maxBlobArea = configmanager.maxBlobArea;



	MyBackgroundSubtractor *backgroundSubtractor = new MyBackgroundSubtractor();

	Mat openKernel = getStructuringElement(MORPH_ELLIPSE, Size(3,3));

	//TrackedVehicle::fullImageSize = Size(640,480);

/*	MeasurementExport *measurementExport = new MeasurementExport(configmanager.detectionOutputFilename,
		configmanager.areaHitOutputFilename, configmanager.imageOutputDirectory, configmanager.doSaveImages);
	trackedVehicleManager.measurementExport = measurementExport; */

}

void SimpleTracker::processFrame(Mat &src, int frameIdx, Mat *verbose)
{
	cv::blur(src,*blurredSrc,cv::Size(10,10));
	src.copyTo(*blurredSrc);

	backgroundSubtractor.operator()(*blurredSrc,*foregroundFrame);

	morphologyEx(*foregroundFrame,*foregroundFrame,MORPH_OPEN,openKernel,Point(-1,-1),1);

	morphologyEx(*foregroundFrame,*foregroundFrame,MORPH_CLOSE,openKernel,Point(-1,-1),1);

	// Tracking blobs
//	src.copyTo(*verbose);

	//cv::compare(*foregroundFrame,Scalar(200),*foregroundFrame,CMP_GT);
	cvblob.findBlobsForTracks(foregroundFrame,verbose);

	cvb::CvTracks *tracks = cvblob.getCvTracks();
	cvb::CvTracks::const_iterator it;
	for (it = tracks->begin(); it != tracks->end(); ++it)
	{
		TrackedVehicle *vehicle = context->getTrackedVehicleOrCreate(it->second->id);

		vehicle->registerDetection(frameIdx, it->second, &src, this->foregroundFrame, verbose);
	}
}

SimpleTracker::SimpleTracker(const char *configfilename)
{
	configmanager.init(configfilename);
	backgroundFrame = new Mat(480,640,CV_8UC1);
	foregroundFrame = new Mat(480,640,CV_8UC1);
	blurredSrc = new Mat(480,640,CV_8UC3);
	init();
}

Mat *SimpleTracker::getCurrentForegroundImage()	// for debug
{
	return foregroundFrame;
}
