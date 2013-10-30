#ifndef __PATHVALIDATOR_H
#define __PATHVALIDATOR_H

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


	Path(FileNode *node)
	{
		load(node);
	}

	Path(int id)
	{
		this->id = id;
	}

	void save(FileStorage *fs)
	{
		ostringstream oss;
		oss << id;
		*fs << "{";	// start mapping with name "area"
		*fs	<< "id" << id;//oss.str();
		*fs << "areas" << areaIdxList
/*		*fs	<< "areas" << "[";	// start sequence with name "points"
		for(unsigned int idx=0; idx<areaIdxList.size(); idx++)
		{
			oss.clear();
			oss << areaIdxList[idx];
			*fs << oss.str();
		}
		*fs << "]"	// finish sequence "points" */
			<< "}";	// finish mapping "area"

	}

	void load(FileNode *node)
	{
		(*node)["id"] >> id;

		FileNode areaNode = (*node)["areas"];

		(*node)["areas"] >> areaIdxList;
	}

	/** Check wether given area hit list conforms this path*/
	bool isValid(vector<int> &areaHits)
	{
		if (areaIdxList.size() == 0)
		{
			cout << "WARNING: Path object is not valid, cannot be used for validation. Empty areaIdxList." << endl;
			return false;	// Path itself is not valid. areaHits cannot be considered valid based on this Path.
		}

		int currentPathAreaIdx = 0;
		int currentAreaIdx = areaIdxList[currentPathAreaIdx];

		for(vector<int>::iterator areaHitIterator=areaHits.begin(); areaHitIterator!=areaHits.end(); areaHitIterator++)
		{
			if (*areaHitIterator == currentAreaIdx)
			{
				continue;
			}
			// Jump to next area?
			if (currentPathAreaIdx < areaIdxList.size()-1)
			{
				currentPathAreaIdx++;
				currentAreaIdx = areaIdxList[currentPathAreaIdx];
				if (*areaHitIterator == currentAreaIdx)
				{
					continue;	// Next area is also next in the Path
				}
				else
				{
					// Jump to the wrong area, cannot be valid
					return false;
				}
			}
			else
			{
				// No more areaIdx, cannot be valid
				return false;
			}
		}

		// areaHits finished. Is the Path also finished?
		if (currentPathAreaIdx == areaIdxList.size()-1)
		{
			return true;	// Path and areaHits also finished, areaHits is valid
		}
		else
		{
			return false;	// Path defined more areaIdx then the areaHits have contained.
		}
	}

	int getIdIfValid(vector<int> &areaHits)
	{
		if (isValid(areaHits))
		{
			return this->id;
		}
		return -1;
	}
};

class PathValidator
{
public:
	vector<Path> paths;


	void savePathList(const char *filename)
	{
		FileStorage fs(filename,FileStorage::WRITE);

		fs << "pathlist" << "[";
		for(unsigned int pathIdx=0;pathIdx<paths.size();pathIdx++)
		{
			paths[pathIdx].save(&fs);
		}
		fs << "]";
		fs.release();
	}

	void loadPathList(const char *filename)
	{
		paths.clear();
		FileStorage fs(filename,FileStorage::READ);
		ostringstream oss;

		FileNode pathFileNodes = fs["pathlist"];
		cout << "Loading pathlist (size=" << pathFileNodes.size() << ")" << endl;

		FileNodeIterator it = pathFileNodes.begin();
		FileNodeIterator it_end = pathFileNodes.end();

		for( ; it != it_end; ++it)
		{
			Path currentPath(&(*it));	// create and load
			paths.push_back(currentPath);
		}

		fs.release();
	}

	int getPathIdIfValid(vector<int> &areaHits)
	{
		int id = -1;	// Valid path not found
		for(vector<Path>::iterator it=paths.begin(); it!=paths.end(); it++)
		{
			id = it->getIdIfValid(areaHits);
			if (id != -1)
			{
				// Valid path found
				break;
			}
		}
		return id;
	}


};

#endif
