#include <iostream>	// for standard I/O

#include<opencv2/opencv.hpp>
#include<clipper.hpp>

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

	if (npts==0)
	{
		// Nothing to draw
		return;
	}

	if (fill)
	{
		fillPoly(*img, &pts,&npts, 1, color);
	}
	else
	{
		polylines(*img, &pts,&npts, 1, true, color, 1);
	}

	// find rightmost point to show ID beside it
	int rightmostIdx = 0;
	for(unsigned int i=1; i<points.size(); i++)
	{
		if (points[i].x > points[rightmostIdx].x)
		{
			rightmostIdx = i;
		}
	}
	ostringstream oss;
	oss << id;
	putText(*img, oss.str().c_str(), Point(points[rightmostIdx].x+5, points[rightmostIdx].y), CV_FONT_HERSHEY_SIMPLEX, 0.5, color,1,8,false);


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


/*double PolygonArea(Point *polygon,int N)
{
	// Written by Paul Bourke
	// http://paulbourke.net/geometry/polygonmesh/source1.c

   int i,j;
   double area = 0;

   for (i=0;i<N;i++) {
      j = (i + 1) % N;
      area += polygon[i].x * polygon[j].y;
      area -= polygon[i].y * polygon[j].x;
   }

   area /= 2;
   return(area < 0 ? -area : area);
}*/


bool Area::isRectangleIntersecting(Rect rect)
{
	// Use the Clipper library
	ClipperLib::Clipper c;

	//create clipper polygons from your points
	ClipperLib::Polygon rectPolygon;
	rectPolygon.push_back(ClipperLib::IntPoint( rect.x, rect.y ));
	rectPolygon.push_back(ClipperLib::IntPoint( rect.x+rect.width, rect.y ));
	rectPolygon.push_back(ClipperLib::IntPoint( rect.x+rect.width, rect.y+rect.height ));
	rectPolygon.push_back(ClipperLib::IntPoint( rect.x, rect.y+rect.height ));

	ClipperLib::Polygon areaPolygon;
	for(int i=0; i<points.size(); i++)
	{
		areaPolygon.push_back(ClipperLib::IntPoint( points[i].x, points[i].y ));
	}
	c.AddPolygon(areaPolygon, ClipperLib::PolyType::ptSubject);
	c.AddPolygon(rectPolygon, ClipperLib::PolyType::ptClip);

	ClipperLib::PolyTree solution;
	c.Execute(ClipperLib::ClipType::ctIntersection, solution, ClipperLib::PolyFillType::pftNonZero, ClipperLib::PolyFillType::pftNonZero);

	bool isIntersecting = (solution.Childs.size() > 0);	// Is this enough?
	return isIntersecting;
}
