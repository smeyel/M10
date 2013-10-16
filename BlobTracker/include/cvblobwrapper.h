#ifndef __CVBLOBWRAPPER_H
#define __CVBLOBWRAPPER_H

#include <opencv/cv.h>
#include <cvblob.h>

#include "cvblobwrapper.h"

class CvBlobWrapper
{
	//IplConvKernel* morphKernel;
	cvb::CvTracks tracks;
	CvSize imgSize;
    cvb::CvBlobs blobs;
	unsigned int blobNumber;

public:

	unsigned int minBlobArea;
	unsigned int maxBlobArea;

	CvBlobWrapper();
	~CvBlobWrapper();
	void findWhiteBlobs(cv::Mat *src, cv::Mat *result);
	
	cvb::CvTracks *getCvTracks()
	{
		return &tracks;
	}
};

#endif
