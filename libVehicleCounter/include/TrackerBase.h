#ifndef __TRACKERBASE_H
#define __TRACKERBASE_H

#include "TrackingContext.h"

class TrackerBase 
{
public:
	TrackingContext *context;

	virtual void processFrame(Mat &src, int frameIdx, Mat *verbose) = 0;
	virtual Mat *getCurrentForegroundImage() = 0;	// for debug

};

#endif
