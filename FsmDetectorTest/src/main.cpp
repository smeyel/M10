//#include <iostream>	// for standard I/O
//#include <fstream>
#include <string>   // for strings

//#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)
//#include <opencv2/highgui/highgui.hpp>  // Video write

//#include "CameraLocalProxy.h"
//#include "StdoutLogger.h"

//#include "MyLutColorFilter.h"

//#include "TimeMeasurement.h"

//#include "TransitionStat.h"

#include "fsmlearning.h"
#include "foregrounddetection.h"

/*using namespace std;
using namespace cv;
using namespace smeyel;
using namespace LogConfigTime; */

/** This program uses a local camera picture to locate pre-learned marker locations.
	- 
*/

int main(int argc, char *argv[], char *window_name)
{
	//test_learnFromImagesAndMasks(0,14,NULL);
	test_foregroundDetector(NULL);
}
