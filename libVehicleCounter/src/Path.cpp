#include "Path.h"

Path::Path(FileNode *node)
{
	load(node);
}

Path::Path(int id)
{
	this->id = id;
}

void Path::save(FileStorage *fs)
{
	ostringstream oss;
	oss << id;
	*fs << "{";	// start mapping with name "area"
	*fs	<< "id" << id;//oss.str();
	*fs << "areas" << areaIdxList
		<< "}";	// finish mapping "area"

}

void Path::load(FileNode *node)
{
	(*node)["id"] >> id;

	FileNode areaNode = (*node)["areas"];

	(*node)["areas"] >> areaIdxList;
}

/** Check wether given area hit list conforms this path*/
bool Path::isValid(vector<int> &areaHits)
{
	if (areaIdxList.size() == 0)
	{
		cout << "WARNING: Path object is not valid, cannot be used for validation. Empty areaIdxList." << endl;
		return false;	// Path itself is not valid. areaHits cannot be considered valid based on this Path.
	}

	int currentPathAreaIdx = 0;
	int currentAreaIdx = areaIdxList[currentPathAreaIdx];
	bool currentSeenAtLeastOnce = false;	// Do not allow two jumps immediately after each other!

	for(vector<int>::iterator areaHitIterator=areaHits.begin(); areaHitIterator!=areaHits.end(); areaHitIterator++)
	{
		if (*areaHitIterator == currentAreaIdx)
		{
			currentSeenAtLeastOnce = true;
			continue;
		}
		if (!currentSeenAtLeastOnce)
		{
			// Current areaIdx not seen, cannot jump over!
			return false;
		}
		// Jump to next area?
		if (currentPathAreaIdx < areaIdxList.size()-1)
		{
			currentPathAreaIdx++;
			currentAreaIdx = areaIdxList[currentPathAreaIdx];
			currentSeenAtLeastOnce = false;
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

int Path::getIdIfValid(vector<int> &areaHits)
{
	if (isValid(areaHits))
	{
		return this->id;
	}
	return -1;
}
