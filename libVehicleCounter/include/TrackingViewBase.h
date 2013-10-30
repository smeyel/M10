#ifndef __TRACKINGVIEWBASE_H
#define __TRACKINGVIEWBASE_H

#include<opencv2/opencv.hpp>
#include "TrackingContext.h"

class TrackingViewBase
{
public:
	TrackingContext *context;

	virtual void show(int frameIdx) = 0;
};

#endif
