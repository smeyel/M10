#ifndef __PATHVALIDATOR_H
#define __PATHVALIDATOR_H

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)

#include "Path.h"

using namespace std;
using namespace cv;


class PathValidator
{
public:
	vector<Path> paths;

	void savePathList(const char *filename);

	void loadPathList(const char *filename);

	int getPathIdIfValid(vector<int> &areaHits);

};

#endif
