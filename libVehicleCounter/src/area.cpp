#include <iostream>	// for standard I/O

#include "area.h"

void Area::add(Point p)
{
	points.push_back(p);
}

void Area::clear()
{
	points.clear();
}

void Area::draw(Mat *img, Scalar color, bool fill)
{
	const cv::Point *pts = (const cv::Point*) Mat(points).data;
	int npts = Mat(points).rows;

	if (fill)
	{
		fillPoly(*img, &pts,&npts, 1, color);
	}
	else
	{
		polylines(*img, &pts,&npts, 1, true, Scalar(0,255,0), 3);
	}
}

void Area::save(FileStorage *fs)
{
	ostringstream oss;
	oss << id;
	*fs << "{";	// start mapping with name "area"
	*fs	<< "id" << id;//oss.str();
	*fs	<< "points" << "[";	// start sequence with name "points"
	for(unsigned int idx=0; idx<points.size(); idx++)
	{
		*fs << points[idx];
	}
	*fs << "]"	// finish sequence "points"
		<< "}";	// finish mapping "area"
}

void Area::load(FileNode *node)
{
	(*node)["id"] >> id;

	FileNode pointFileNodes = (*node)["points"];
	FileNodeIterator it = pointFileNodes.begin();
	FileNodeIterator it_end = pointFileNodes.end();

	points.clear();
	for( ; it != it_end; ++it)
	{
		Point p;
		(*it)[0] >> p.x;
		(*it)[1] >> p.y;
		points.push_back(p);
	}

	cout << "Area " << id << " saved." << endl;
}

void Area::saveAreaList(const char *filename, vector<Area> areas)
{
	FileStorage fs(filename,FileStorage::WRITE);

	fs << "arealist" << "[";
	for(unsigned int areaIdx=0;areaIdx<areas.size();++areaIdx)
	{
		areas[areaIdx].save(&fs);
	}
	fs << "]";
	fs.release();
}

void Area::loadAreaList(const char *filename, vector<Area> *areas)
{
	areas->clear();
	FileStorage fs(filename,FileStorage::READ);
	ostringstream oss;

	FileNode areaFileNodes = fs["arealist"];
	cout << "Loading arealist (size=" << areaFileNodes.size() << ")" << endl;

	FileNodeIterator it = areaFileNodes.begin();
	FileNodeIterator it_end = areaFileNodes.end();

	for( ; it != it_end; ++it)
	{
		Area currentArea;
		currentArea.load(&(*it));
		areas->push_back(currentArea);
	}

	fs.release();
}
