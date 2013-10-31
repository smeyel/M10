#include <iostream>	// for standard I/O
#include <fstream>

#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
 
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraLocalProxy.h"

#include "StdoutLogger.h"
#include "FileLogger.h"

#include "myconfigmanager.h"

#include "MeasurementExport.h"

#include "TrackingContext.h"
#include "SimpleTracker.h"
#include "SimpleTrackingView.h"

using namespace cv;
using namespace LogConfigTime;

char *configfilename = "default.ini";
MyConfigManager configmanager;
CameraLocalProxy *camProxy = NULL;
Mat *src;

void init_defaults(const char *overrideConfigFileName = NULL)
{
	if (overrideConfigFileName != NULL)
	{
		// INI file is given as command line parameter
		strcpy(configfilename,overrideConfigFileName);
	}
	// Setup config management
	configmanager.init(configfilename);

	Logger *logger = new StdoutLogger();
	logger->SetLogLevel(Logger::LOGLEVEL_WARNING);

	if (configmanager.videoInputFileOverride)
	{
		camProxy = new CameraLocalProxy(configmanager.videoInputFilename.c_str());
	}
	else
	{
		camProxy = new CameraLocalProxy(VIDEOINPUTTYPE_PS3EYE,0);
	}
	camProxy->getVideoInput()->SetNormalizedExposure(-1);
	camProxy->getVideoInput()->SetNormalizedGain(-1);
	camProxy->getVideoInput()->SetNormalizedWhiteBalance(-1,-1,-1);

	src = new Mat(480,640,CV_8UC3);
}

void test_BlobOnForeground(const char *overrideConfigFileName = NULL)
{
	init_defaults(overrideConfigFileName);

	src = new Mat(480,640,CV_8UC3);
	Mat *verbose = new Mat(480,640,CV_8UC3);

	MeasurementExport *measurementExport = new MeasurementExport(configmanager.detectionOutputFilename,configmanager.areaHitOutputFilename,configmanager.imageOutputDirectory,configmanager.doSaveImages);

	TrackingContext context;
	context.loadTrackedAreas(configmanager.trackedAreaInputFilename.c_str());
	context.loadBackgroundAreas(configmanager.backgroundAreaInputFilename.c_str());
/*	Path newPath(5);
	//newPath.id = 5;
	newPath.areaIdxList.push_back(10);
	newPath.areaIdxList.push_back(11);
	newPath.areaIdxList.push_back(12);
	context.pathValidator.paths.push_back(newPath);
	context.pathValidator.savePathList(configmanager.validPathInputFilename.c_str()); */
	context.pathValidator.loadPathList(configmanager.validPathInputFilename.c_str());

	context.measurementExport = measurementExport;

	SimpleTracker tracker(configfilename);
	tracker.context = &context;

	SimpleTrackingView view(configfilename);
	view.context = &context;
	view.tracker = &tracker;
	view.src = src;
	view.verbose = verbose;

	unsigned int frameIdx = -1;
	enum stateEnum
	{
		run,
		pause,
		turbo,
		replay,
		finished
	} state = run;
	while (state != finished)
	{
		if (state == run || state == turbo || state == replay)
		{
			if (!camProxy->CaptureImage(0,src))
			{
				// No more frames
				state = finished;
				break;
			}
			frameIdx++;
			context.clearBackgroundAreasInImage(*src);
		}

		src->copyTo(*verbose);
		if (state != replay)
		{
			tracker.processFrame(*src,frameIdx,verbose);
		}

		stringstream oss;
		oss << "F:" << frameIdx << " ";
		oss << (state==replay ? "REPLAY" : (state==pause ? "PAUSE" : (state==turbo?"TURBO":"")));
		putText(*verbose,
			oss.str().c_str(),
			Point(20, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,200,200),1,8,false);

		if (state != turbo)
		{
			view.show(frameIdx);
		}

		char k = -1;
		if (state != turbo)
		{
			k = cvWaitKey(25);
		}
		else
		{
			if (frameIdx % 10 == 0)
			{
				cout << "FrameIdx: " << frameIdx << endl;
			}
			k = cvWaitKey(1);
		}

		switch (k)
		{
		case -1:	// No keypress
			break;
		case 27:
			state = finished;
			break;
		case 'r':
			state = run;
			break;
		case 'p':
			state = pause;
			break;
		case 't':
			state = turbo;
			break;
		case '7':
			state = replay;
			break;
		case 'v':
			view.configmanager.recordVideo = !view.configmanager.recordVideo;
			cout << "view.configmanager.recordVideo=" << view.configmanager.recordVideo << endl;
			break;
		// --------------- Data manipulation functions -----------------
		case 'c':
			context.motionVectorStorage.clear();
			context.sizeStorage.clear();
			context.clear();
			cout << "TrackedVehicle list, MotionVectorStorage, and VehicleSizeStorage cleared." << endl;
			break;
		case 'm':
			context.recollectMotionVectors(0.0F);
			context.recalculateLocationConfidences();
			break;
		case '1':
			// Amig nincs confidence becsles (egyszer 0 minConfidence-szel hivtuk), addig threshold-ra semmit nem fog adni.
			context.recollectMotionVectors(0.7F);
			context.recalculateLocationConfidences();
			break;
		// --------------- Visualization settings -----------------
		case '2':	// override doSaveImages
			measurementExport->doSaveImages = !measurementExport->doSaveImages;
			cout << "doSaveImages=" << measurementExport->doSaveImages << endl;
			break;
		case '3':	// toggle showPath
			view.configmanager.showPath = !view.configmanager.showPath;
			cout << "view.configmanager.showPath=" << view.configmanager.showPath << endl;
			break;
		case 'a':	// Average motion vector length
			cout << "Mean MotionVector.length() = " << context.motionVectorStorage.getMeanMotionVectorLength() << endl;
			break;
		// --------------- Motion Vectors persistance functions -----------------
		case 'M':
			context.motionVectorStorage.save(configmanager.motionVectorInputFilename);
			break;
		case 'i':
			context.motionVectorStorage.load(configmanager.motionVectorInputFilename);
			context.recalculateLocationConfidences();
			break;
		// --------------- Export functions -----------------
		case 'e':
			context.exportAllDetections(0.1F);	// minConfidence==0.1 to avoid self-detection of MotionVectors
			break;
		case '8':
			cout << "- recalculateLocationConfidences" << endl;
			context.recalculateLocationConfidences();
			cout << "- TODO: re-calculate sizeRatioToMean" << endl;
			cout << "- validatePath" << endl;
			context.validatePath(0.1F);	// needs calculated confidences
			cout << "- saveVehicles" << endl;
			context.saveVehicles(configmanager.registrationsFilename.c_str());
			cout << "- done" << endl;
			break;
		case '9':
			cout << "- loadVehicles" << endl;
			context.loadVehicles(configmanager.registrationsFilename.c_str());
			// Motion vectors and sizes not loaded anymore! They are not needed as confidences are already set!
			//	But do not forget that they are not loaded!
			// VehicleSizes are also only used for sizeRatioToMean which is already calculated.
			// TODO: recollect motion vectors and sizes. Confidences already loaded.
			cout << "- done, WARNING: motion vectors and sizes are not reloaded!" << endl;
			//context.recollectMotionVectors(0.0F);
			//TODO: RELOAD motion vectors, vehicleMeanSizes
			break;
		// --------------- Debug functions -----------------
		case 's':	// Show mean size
			view.verboseMeanSizeAtLastClickLocation();
			break;
		default:
			cout
				<< "--- run control functions ---" << endl
				<< "r	Run mode" << endl
				<< "p	Pause mode" << endl
				<< "t	Turbo mode" << endl
				<< "v	Toggle Video recording (output video)" << endl
				<< "Esc	Exit program" << endl
				<< "--- data manipulation functions ---" << endl
				<< "c	Clear TrackedVehicle, MotionVector and VehicleSize storages" << endl
				<< "m	Collect motion vectors and re-calculate LocationRegistration confidences" << endl
				<< "1	Collect only confident motion vectors and re-calculate LocationRegistration confidences" << endl
				<< "--- visualization functions ---" << endl
				<< "2	Toggle doSaveImages (override ini)" << endl
				<< "3	Toggle showPath" << endl
				<< "a	Show average motion vector length" << endl
				<< "--- MotionVector persistance functions ---" << endl
				<< "M	Save motion vectors (filename defined by ini)" << endl
				<< "i	Import motion vectors and re-calculate LocationRegistration confidences" << endl
				<< "--- Export functions ---" << endl
				<< "e	Export all detection data" << endl
				<< "--- Debug functions ---" << endl
				<< "s	Show mean sizes at lastMouseClickLocation" << endl;
			break;
		}
	}
}

int main(int argc, char *argv[], char *window_name)
{
	test_BlobOnForeground();
}
