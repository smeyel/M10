#ifndef __MEASUREMENTEXPORT_H
#define __MEASUREMENTEXPORT_H

#include <fstream>
#include<opencv2/opencv.hpp>

using namespace std;

class MeasurementExport
{
	string imageOutputDir;

	bool createDirIfNeeded(string dirname);
public:
	bool doSaveImages;

	ofstream detectionOutput;
	ofstream areaHitOutput;

	MeasurementExport(string measurementOutputFilename, string areaHitOutputFilename, string imageOutputDirectory, bool doSaveImages=true);
	~MeasurementExport();

	string saveimage(int objectID, const char *filenamePrefix, int frameIdx, cv::Mat &image, cv::Rect &roi, bool overrideDoSaveImagesSetting=false);
};

#endif
