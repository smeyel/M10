#include <iostream>
#include <opencv2/highgui/highgui.hpp>  // for imwrite

#include "MeasurementExport.h"

using namespace std;

MeasurementExport::MeasurementExport(string detectionOutputFilename, string areaHitOutputFilename, string imageOutputDirectory, bool doSaveImages)
{
	detectionOutput.open(detectionOutputFilename);
	areaHitOutput.open(areaHitOutputFilename);
	imageOutputDir = imageOutputDirectory;
	this->doSaveImages = doSaveImages;
	filenameIdx = 0;
}

MeasurementExport::~MeasurementExport()
{
	detectionOutput.close();
	areaHitOutput.close();
}

string MeasurementExport::saveimage(cv::Mat *image, cv::Rect &roi)
{
	if (doSaveImages)
	{
		stringstream ss;
		ss << imageOutputDir << "/" << filenameIdx << ".png";
		string filename = ss.str();

		cv::Mat roiImage = (*image)(roi);
		imwrite(filename.c_str(),roiImage);

		filenameIdx++;
		return filename;
	}
	else
	{
		return string();
	}
}
