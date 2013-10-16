#include <iostream>	// for standard I/O
#include <fstream>

#include<opencv2/opencv.hpp>
#include<iostream>
#include<vector>
 
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
#include <opencv2/highgui/highgui.hpp>  // Video write

#include "CameraLocalProxy.h"

//#include "DefaultLutColorFilter.h"
#include "StdoutLogger.h"
//#include "FileLogger.h"

#include "myconfigmanager.h"

#include "area.h"

using namespace cv;
using namespace LogConfigTime;
//using namespace smeyel;

char *configfilename = "default.ini";
MyConfigManager configmanager;
CameraLocalProxy *camProxy = NULL;
Mat *src;

// Globals used by mouse event handler
Point lastMouseClickLocation;

// Mouse callback to retrieve debug information at pixel locations
void mouse_callback(int eventtype, int x, int y, int flags, void *param)
{
	if (eventtype == CV_EVENT_LBUTTONDOWN)
	{
		lastMouseClickLocation = Point(x,y);
		cout << "Click location saved: " << x << ";" << y << endl;
	}
}

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


int main(int argc, char *argv[], char *window_name)
{
	init_defaults();

	src = new Mat(480,640,CV_8UC3);
	//Mat *result = new Mat(480,640,CV_8UC3);

	namedWindow("SRC", CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback("SRC", mouse_callback);

    std::vector<Area> areas;

	Area currentArea;
	currentArea.id = 0;

	enum runstate
	{
		run,
		freeze,
		finish
	} state = run;
	while (state != finish)
	{
		if (state==run)
		{
			camProxy->CaptureImage(0,src);
		}

		for(unsigned int i=0; i<areas.size(); i++)
		{
			areas[i].draw(src,Scalar(0,0,255),false);
		}
		currentArea.draw(src,Scalar(0,255,0),false);
		circle(*src,lastMouseClickLocation,3,Scalar(255,0,0),2);

		imshow("SRC",*src);

		char k = cvWaitKey(25);
		switch (k)
		{
			case -1:	// No key pressed
				break;
			case 27:
				state = finish;
				break;
			case 'f':
				state = freeze;
				cout << "Freeze" << endl;
				break;
			case 'r':
				state = run;
				cout << "Run" << endl;
				break;
			case 'c':	// Clear current area
				currentArea.clear();
				cout << "Cleared" << endl;
				break;
			case 'p':	// Add point (last clicked location)
				currentArea.add(lastMouseClickLocation);
				cout << "Point added" << endl;
				break;
			case 's':	// Save area, start new one
				areas.push_back(currentArea);
				currentArea.clear();
				currentArea.id++;
				cout << "Saved and started new" << endl;
				break;
			case 'l':
				Area::loadAreaList(configmanager.areaOutputFilename.c_str(), &areas);
				break;
			case 'w':
				Area::saveAreaList(configmanager.areaOutputFilename.c_str(), areas);
				break;
			default:
				cout << "Esc:	exit program" << endl
					<< "--- video control functions" << endl
					<< "f:	freeze" << endl
					<< "r:	run" << endl
					<< "--- area edit functions" << endl
					<< "c:	clear current area" << endl
					<< "p:	add last click point" << endl
					<< "s:	save current area and start new one" << endl
					<< "--- file operations" << endl
					<< "w:	write areas into file" << endl
					<< "l:	load areas from file" << endl;
				break;
		}
	}
}
