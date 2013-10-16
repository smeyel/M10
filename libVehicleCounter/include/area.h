#ifndef __AREA_H
#define __AREA_H

#include <vector>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)

using namespace std;
using namespace cv;

class Area
{
	vector<Point> points;

	void save(FileStorage *fs);
	void load(FileNode *node);
public:
	int id;
	void add(Point p);
	void clear();
	void draw(Mat *img, Scalar color, bool fill=false);

	static void saveAreaList(const char *filename, vector<Area> areas);
	static void loadAreaList(const char *filename, vector<Area> *areas);
};

#endif
