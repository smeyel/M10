#ifndef __CVBLOBWRAPPER_H
#define __CVBLOBWRAPPER_H

#include <opencv/cv.h>
#include <cvblob.h>

class CvBlobWrapper
{
	IplConvKernel* morphKernel;
	cvb::CvTracks tracks;
	CvSize imgSize;
    cvb::CvBlobs blobs;
	unsigned int blobNumber;

public:
	CvBlobWrapper();
	~CvBlobWrapper();
	void findBlobsInRgb(cv::Mat *src, cv::Mat *result);
	void findWhiteBlobs(cv::Mat *src, cv::Mat *result);
	
};

#endif
