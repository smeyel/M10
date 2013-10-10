#include <iostream>
#include <iomanip>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "cvblobwrapper.h"

using namespace cvb;
using namespace cv;

CvBlobWrapper::CvBlobWrapper()
{
	morphKernel = cvCreateStructuringElementEx(5, 5, 1, 1, CV_SHAPE_RECT, NULL);
	blobNumber = 0;
}

CvBlobWrapper::~CvBlobWrapper()
{
	cvReleaseStructuringElement(&morphKernel);
	cvReleaseBlobs(blobs);
}

void CvBlobWrapper::findBlobs(Mat *src, Mat *result)
{
	Size imgSize = src->size();
	IplImage img = *src;

	IplImage srcIpl = *src;
	//IplImage segmentedIpl = *segmented;

	IplImage *frame = cvCreateImage(imgSize, img.depth, img.nChannels);
    cvConvertScale(&img, frame, 1, 0);

    IplImage *segmentatedIpl = cvCreateImage(imgSize, 8, 1);
    
    // Detecting red pixels:
    // (This is very slow, use direct access better...)
	for (unsigned int j=0; j<imgSize.height; j++)
		for (unsigned int i=0; i<imgSize.width; i++)
		{
			CvScalar c = cvGet2D(frame, j, i);

			double b = ((double)c.val[0])/255.;
			double g = ((double)c.val[1])/255.;
			double r = ((double)c.val[2])/255.;
			unsigned char f = 255*((r>0.2+g)&&(r>0.2+b));

			cvSet2D(segmentatedIpl, j, i, CV_RGB(f, f, f));
		}

    cvMorphologyEx(segmentatedIpl, segmentatedIpl, NULL, morphKernel, CV_MOP_OPEN, 1);

    //cvShowImage("segmentated", segmentated);

    IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

    unsigned int labelNum = cvLabel(segmentatedIpl, labelImg, blobs);
    cvb::cvFilterByArea(blobs, 500, 1000000);
    cvb::cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_BOUNDING_BOX);
    cvb::cvUpdateTracks(blobs, tracks, 200., 5);
    cvb::cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

//    cvShowImage("red_object_tracking", frame);

    /*std::stringstream filename;
    filename << "redobject_" << std::setw(5) << std::setfill('0') << frameNumber << ".png";
    cvSaveImage(filename.str().c_str(), frame);*/

    cvReleaseImage(&labelImg);
    cvReleaseImage(&segmentatedIpl);

	Mat tempMat(frame,true);
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


    //frameNumber++;

  cvReleaseImage(&frame);

  return;
}

