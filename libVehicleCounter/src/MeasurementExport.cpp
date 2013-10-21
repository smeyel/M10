#include <Windows.h>	// for CreateDirectory
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
}

MeasurementExport::~MeasurementExport()
{
	detectionOutput.close();
	areaHitOutput.close();
}

bool MeasurementExport::createDirIfNeeded(string dirname)
{
	bool res = false;//CreateDirectory((LPCWSTR)dirname.c_str(), NULL);

	if (res)
	{
		return true;
	}

	DWORD lastErrorCode = GetLastError();
	if (ERROR_ALREADY_EXISTS == lastErrorCode)
	{
		return true;
	}

	return false;
}

string MeasurementExport::saveimage(int objectID, const char *filenamePrefix, int frameIdx, cv::Mat &image, cv::Rect &roi)
{
	if (doSaveImages)
	{
		// Create directory if needed

		stringstream ss;
		ss << imageOutputDir << "\\" << objectID;
		//createDirIfNeeded(ss.str());
		
		//ss << "\\" << filenamePrefix << frameIdx << ".png";
		ss << filenamePrefix << "F" << frameIdx << ".png";
		string filename = ss.str();

		cv::Mat roiImage = image(roi);
		imwrite(filename.c_str(),roiImage);

		return filename;
	}
	else
	{
		return string();
	}
}
