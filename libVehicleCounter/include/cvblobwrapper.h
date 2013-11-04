#ifndef __CVBLOBWRAPPER_H
#define __CVBLOBWRAPPER_H

#include <opencv/cv.h>
#include <cvblob.h>

#include "cvblobwrapper.h"

class TrackingContext;

class CvBlobWrapper
{
	//IplConvKernel* morphKernel;
	cvb::CvTracks tracks;
	CvSize imgSize;
    cvb::CvBlobs blobs;
	unsigned int blobNumber;

	void OriginalUpdateTracks(cvb::CvBlobs const &blobs, cvb::CvTracks &tracks);

	// Use only after the motion vectors have been collected!
	void MotionVectorBasedUpdateTracks(cvb::CvBlobs const &blobs, cvb::CvTracks &tracks);

	double thDistance;
	unsigned int thInactive;
	unsigned int thActive;

	float minConfidence;
	double confidenceBlobTrack(cvb::CvBlob const *b, cvb::CvTrack const *t);

public:
	TrackingContext *context;

	unsigned int minBlobArea;
	unsigned int maxBlobArea;

	enum TrackingModeEnum
	{
		cvblob,
		motionvector,
		adaptive
	} trackingMode;

	CvBlobWrapper();
	~CvBlobWrapper();
	void findBlobsForTracks(cv::Mat *src, cv::Mat *result);
	
	cvb::CvTracks *getCvTracks();
};

#endif
