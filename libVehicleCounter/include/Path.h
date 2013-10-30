#ifndef __PATH_H
#define __PATH_H
#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)

using namespace std;
using namespace cv;

class Path
{
public:
	int id;
	vector<int> areaIdxList;


	Path(FileNode *node);

	Path(int id);

	void save(FileStorage *fs);

	void load(FileNode *node);

	/** Check wether given area hit list conforms this path*/
	bool isValid(vector<int> &areaHits);

	int getIdIfValid(vector<int> &areaHits);

};

#endif
