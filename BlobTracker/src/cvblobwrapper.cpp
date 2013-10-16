#include <iostream>
#include <iomanip>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "cvblobwrapper.h"

using namespace cvb;
using namespace cv;

CvBlobWrapper::CvBlobWrapper()
{
	blobNumber = 0;

	minBlobArea = 500;
	maxBlobArea = 1000000;

}

CvBlobWrapper::~CvBlobWrapper()
{
	cvReleaseBlobs(blobs);
}

void CvBlobWrapper::findWhiteBlobs(Mat *src, Mat *result)
{
	IplImage imgSrc = *src;
	IplImage imgRes = *result;

    IplImage *labelImg = cvCreateImage(cvGetSize(&imgSrc), IPL_DEPTH_LABEL, 1);

    unsigned int labelNum = cvLabel(&imgSrc, labelImg, blobs);
    cvb::cvFilterByArea(blobs, minBlobArea, maxBlobArea);
    cvb::cvRenderBlobs(labelImg, blobs, &imgSrc, &imgRes, CV_BLOB_RENDER_BOUNDING_BOX);
    cvb::cvUpdateTracks(blobs, tracks, 200., 5);
    cvb::cvRenderTracks(tracks, &imgSrc, &imgRes, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

    cvReleaseImage(&labelImg);

	Mat tempMat(&imgRes,true);
	tempMat.copyTo(*result);

    /*for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
    {
        std::stringstream filename;
        filename << "redobject_blob_" << std::setw(5) << std::setfill('0') << blobNumber << ".png";
        cvSaveImageBlob(filename.str().c_str(), img, it->second);
        blobNumber++;

        std::cout << filename.str() << " saved!" << std::endl;
    }
    break;*/

  return;
}

