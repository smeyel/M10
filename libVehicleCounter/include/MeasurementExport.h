#ifndef __MEASUREMENTEXPORT_H
#define __MEASUREMENTEXPORT_H

#include <fstream>
#include<opencv2/opencv.hpp>

using namespace std;

class MeasurementExport
{
	unsigned int filenameIdx;
	string imageOutputDir;
	bool doSaveImages;
public:
	ofstream detectionOutput;
	ofstream areaHitOutput;

	MeasurementExport(string measurementOutputFilename, string areaHitOutputFilename, string imageOutputDirectory, bool doSaveImages=true);
	~MeasurementExport();

	string saveimage(cv::Mat *image, cv::Rect &roi);
};


#endif
