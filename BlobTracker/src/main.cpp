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
unsigned int frameIdx;
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

void exportMeasurementData(TrackingContext &context)
{
/*	cout << "- recalculateLocationConfidences" << endl;
	context.recalculateLocationConfidences(); */
	cout << "- TODO: re-calculate sizeRatioToMean" << endl;
	cout << "- exportAllDetections, includes path validation" << endl;
	// exportAllDetections includes path validation
	context.exportAllDetections(0.1F);	// minConfidence==0.1 to avoid self-detection of MotionVectors
	cout << "- saveVehicles" << endl;
	context.saveVehicles(configmanager.registrationsFilename.c_str());

	cout << "- motionVectorStorage.save" << endl;
	context.motionVectorStorage.consolidate(0.5F,0.9F);
	context.motionVectorStorage.save(configmanager.motionVectorInputFilename);
	cout << "- savePathCounts" << endl;
	context.savePathCounts();

	cout << "- done" << endl;
}

void dumpLocationRegistration(TrackingContext &context, TrackedVehicle *vehicle)
{
	cout << "dumpLocationRegistration:" << endl;
	for(vector<LocationRegistration>::iterator it=vehicle->locationRegistrations.begin();
		it != vehicle->locationRegistrations.end();
		it++)
	{
		cout << "F:" << it->frameIdx << ", r:" << it->centroid << ", v:" << it->lastSpeedVector << ", conf:" << it->confidence << ", Area:" << it->areaHitId << endl;
	}
}

void jumpToFirstLocationRegistration(TrackingContext &context, TrackedVehicle *vehicle)
{
	int firstDetectionFrame = -1;
	for(vector<LocationRegistration>::iterator it=vehicle->locationRegistrations.begin();
		it != vehicle->locationRegistrations.end();
		it++)
	{
		if (firstDetectionFrame == -1 || firstDetectionFrame > it->frameIdx)
		{
			firstDetectionFrame = it->frameIdx;
		}
	}
	cout << "Jump to frame " << firstDetectionFrame << endl;
	camProxy->JumpToFrame(firstDetectionFrame);
	frameIdx = firstDetectionFrame;
}

void queryProcessor_Vehicle(TrackingContext &context, int vehicleId)
{
	if (vehicleId >= context.trackedVehicles.size())
	{
		cout << "No vehicle with ID=" << vehicleId << endl;
		return;
	}
	TrackedVehicle *vehicle = context.trackedVehicles[vehicleId];
	bool isFinished = false;
	while (!isFinished)
	{
		cout << "--- Query processor - Vehicle ---" << endl
			<< "ID: " << vehicle->trackID << " Path:" << vehicle->pathID << endl
			<<	"(1) LocationRegistration dump" << endl
			<<	"(2) Jump to first detection frame" << endl
			<<	"(x) Back" << endl
			<<	"->";
		char option;
		cin >> option;
		cout << endl;
		switch(option)
		{
		case '1':
			dumpLocationRegistration(context, vehicle);
			break;
		case '2':
			jumpToFirstLocationRegistration(context, vehicle);
			break;
		case 'x':
			isFinished=true;
			break;
		default:
			cout << "Unknown option." << endl;
			break;
		}
	}
}

void queryProcessor(TrackingContext &context)
{
	bool isFinished = false;
	while (!isFinished)
	{
		cout << "--- Query processor ---" << endl
			<<	"(1) Vehicle queries" << endl
			<<	"(2) Size queries" << endl
			<<	"(9) Exit query processor" << endl
			<<	"->";
		char option;
		cin >> option;
		cout << endl;
		switch(option)
		{
		case '1':
			int vehicleId;
			cout << "Vehicle ID: ";
			cin >> vehicleId;
			cout << endl;
			queryProcessor_Vehicle(context,vehicleId);
			break;
		case '2':
			break;
		case 'x':
			isFinished=true;
			break;
		default:
			cout << "Unknown option." << endl;
		}
	}
}

void test_BlobOnForeground(const char *overrideConfigFileName = NULL)
{
	init_defaults(overrideConfigFileName);

	src = new Mat(480,640,CV_8UC3);
	Mat *verbose = new Mat(480,640,CV_8UC3);

	MeasurementExport *measurementExport = new MeasurementExport(configmanager.detectionOutputFilename,configmanager.areaHitOutputFilename,configmanager.pathCounterOutputFilename,configmanager.imageOutputDirectory,configmanager.doSaveImages);

	TrackingContext context;
	context.loadTrackedAreas(configmanager.trackedAreaInputFilename.c_str());
	context.loadBackgroundAreas(configmanager.backgroundAreaInputFilename.c_str());
	context.pathValidator.loadPathList(configmanager.validPathInputFilename.c_str());

	context.measurementExport = measurementExport;

	context.motionVectorStorage.minConfidenceToSkipAddingNewMotionVector = 0.95F;
	context.motionVectorStorage.collectNewMotionVectors = true;

	SimpleTracker tracker(configfilename);
	tracker.context = &context;
	tracker.cvblob.context = &context;	// Do this some other way, not here!

	SimpleTrackingView view(configfilename);
	view.context = &context;
	view.tracker = &tracker;
	view.src = src;
	view.verbose = verbose;

	frameIdx = -1;
	bool saveMeasurementDataUponVideoEnd = true;	// Disabled if Esc is pressed

	enum ProcessingModeEnum
	{
		processing,
		turbo,
		replay
	} processingMode = processing;

	enum controlStateEnum
	{
		run,
		pause,
		single,
		finished
	} controlState = run;
	while (controlState != finished)
	{
		if (controlState == run || controlState == single)
		{
			if (!camProxy->CaptureImage(0,src))
			{
				// No more frames
				cout << "No more input frames." << endl;
				controlState = finished;
				break;
			}
			frameIdx++;
			context.clearBackgroundAreasInImage(*src);
		}

		src->copyTo(*verbose);
		if (processingMode == processing || processingMode == turbo)
		{
			tracker.processFrame(*src,frameIdx,verbose);
		}

		stringstream oss;
		oss << "F:" << frameIdx << " ";
		if (processingMode == turbo)
		{
			oss << " TURBO";
		}
		if (processingMode == replay)
		{
			oss << " REPLAY";
		}
		if (controlState == pause)
		{
			oss << " PAUSE";
		}
		putText(*verbose,
			oss.str().c_str(),
			Point(20, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,200,200),1,8,false);

		if (processingMode != turbo)
		{
			view.show(frameIdx);
		}

		if (controlState == single)
		{
			controlState = pause;
		}

		char k = -1;
		if (processingMode != turbo)
		{
			k = cvWaitKey(25);
		}
		else
		{
			if (frameIdx % 25 == 0)
			{
				cout << "FrameIdx: " << frameIdx << endl;
			}
			k = cvWaitKey(1);
		}

		switch (k)
		{
		case -1:	// No keypress
			break;
		// --------------- Control state and processing mode
		case 27:
			saveMeasurementDataUponVideoEnd = false;
			controlState = finished;
			break;
		case 'r':
			controlState = run;
			break;
		case 'p':
			controlState = pause;
			break;
		case 't':
			processingMode = turbo;
			break;
		case 's':
			controlState = single;
		// --------------- Switch to replay mode
		case 'l':	// Load
			cout << "- loadVehicles" << endl;
			context.loadVehicles(configmanager.registrationsFilename.c_str());
			// Motion vectors and sizes not loaded anymore! They are not needed as confidences are already set!
			//	But do not forget that they are not loaded!
			// VehicleSizes are also only used for sizeRatioToMean which is already calculated.
			// TODO: recollect motion vectors and sizes. Confidences already loaded.

			cout << "- loading motion vectors" << endl;
			context.motionVectorStorage.load(configmanager.motionVectorInputFilename);
			cout << "- turning off motion vector collection" << endl;
			context.motionVectorStorage.collectNewMotionVectors = false;

			cout << "- Switching to REPLAY mode" << endl;
			processingMode = replay;
			saveMeasurementDataUponVideoEnd = false;
			break;
		// --------------- Debug and Verbose functions
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
/*		case 'm':
			context.recollectMotionVectors(0.0F);
			context.recalculateLocationConfidences();
			break; */
		case 'a':
			tracker.cvblob.trackingMode = CvBlobWrapper::TrackingModeEnum::adaptive;
			cout << "Tracking mode: ADAPTIVE" << endl;
			break;
		case '1':
			tracker.cvblob.trackingMode =
				tracker.cvblob.trackingMode == CvBlobWrapper::TrackingModeEnum::cvblob ?
				CvBlobWrapper::TrackingModeEnum::motionvector :
				CvBlobWrapper::TrackingModeEnum::cvblob;
			cout << "Tracking mode: " << (tracker.cvblob.trackingMode == CvBlobWrapper::TrackingModeEnum::cvblob ? "cvblob" : "motionvector") << endl;
/*			// Amig nincs confidence becsles (egyszer 0 minConfidence-szel hivtuk), addig threshold-ra semmit nem fog adni.
			context.recollectMotionVectors(0.7F);
			context.recalculateLocationConfidences(); */
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
/*		case 'a':	// Average motion vector length
			cout << "Mean MotionVector.length() = " << context.motionVectorStorage.getMeanMotionVectorLength() << endl;
			break; */
		// --------------- Motion Vectors persistance functions -----------------
/*		case 'M':
			context.motionVectorStorage.save(configmanager.motionVectorInputFilename);
			break; */
		case 'i':
			context.motionVectorStorage.load(configmanager.motionVectorInputFilename);
			break;
		// --------------- Export functions -----------------
		case 'e':
			exportMeasurementData(context);
			break;
		case 'o':	// cOnsolidate MotionVectorStorage
			cout << "motionVectorStorage.consolidate(0.5F,0.9F)..." << endl;
			context.motionVectorStorage.consolidate(0.5F,0.9F);
			break;

/*		case '8':
			cout << "- recalculateLocationConfidences" << endl;
			context.recalculateLocationConfidences();
			cout << "- TODO: re-calculate sizeRatioToMean" << endl;
			cout << "- validatePath" << endl;
			context.validatePath(0.1F);	// needs calculated confidences
			cout << "- saveVehicles" << endl;
			context.saveVehicles(configmanager.registrationsFilename.c_str());
			cout << "- done" << endl;
			break; */
		// --------------- Debug functions -----------------
/*		case 's':	// Show mean size
			view.verboseMeanSizeAtLastClickLocation();
			break; */
		case 'q':	// Start query based processor
			queryProcessor(context);
			break;
		default:
			cout
				<< "--- run control functions ---" << endl
				<< "r	Run mode" << endl
				<< "p	Pause mode (no frame capture, no processing)" << endl
				<< "t	Turbo processing mode (no visualization)" << endl
				<< "s	Single step" << endl
				<< "l	Load previous data for replay mode" << endl
				<< "Esc	Exit program" << endl
				<< "--- Debug functions ---" << endl
				<< "q	Query mode" << endl;
			break;
		}
	}

	if (saveMeasurementDataUponVideoEnd)
	{
		cout << "Video finished, exporting data..." << endl;
		exportMeasurementData(context);
	}
}

int main(int argc, char *argv[], char *window_name)
{
	test_BlobOnForeground();
}
