#include "PathValidator.h"

void PathValidator::savePathList(const char *filename)
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

void PathValidator::loadPathList(const char *filename)
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

int PathValidator::getPathIdIfValid(vector<int> &areaHits)
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


