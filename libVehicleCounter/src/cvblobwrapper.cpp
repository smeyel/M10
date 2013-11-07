#include <iostream>
#include <iomanip>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "cvblobwrapper.h"
#include "TrackingContext.h"

using namespace cvb;
using namespace cv;

CvBlobWrapper::CvBlobWrapper()
{
	blobNumber = 0;

	thDistance = 40.;
	thInactive = 5;
	thActive = 2;

	minConfidence = 0.7;

	minBlobArea = 500;
	maxBlobArea = 1000000;

	trackingMode = cvblob;
}

CvBlobWrapper::~CvBlobWrapper()
{
	cvReleaseBlobs(blobs);
}

cvb::CvTracks *CvBlobWrapper::getCvTracks()
{
	return &tracks;
}

void CvBlobWrapper::findBlobs(Mat *src, Mat *verbose, vector<Blob> &targetBlobList)
{
	IplImage imgSrc = *src;
	IplImage imgRes = *verbose;

	IplImage *labelImg = cvCreateImage(cvGetSize(&imgSrc), IPL_DEPTH_LABEL, 1);

	unsigned int labelNum = cvLabel(&imgSrc, labelImg, blobs);
	cvb::cvFilterByArea(blobs, minBlobArea, maxBlobArea);
	cvb::cvRenderBlobs(labelImg, blobs, &imgSrc, &imgRes, CV_BLOB_RENDER_BOUNDING_BOX);

	for(cvb::CvBlobs::iterator it=blobs.begin(); it!=blobs.end(); it++)
	{
		CvBlob *b = it->second;
		Blob newBlob;
		newBlob.centroid = Point((int)b->centroid.x,(int)b->centroid.y);
		newBlob.area = b->area;
		newBlob.id = b->label;
		newBlob.rect = Rect(b->minx, b->miny, b->maxx - b->minx, b->maxy - b->miny);
		targetBlobList.push_back(newBlob);
	}
}

void CvBlobWrapper::findBlobsForTracks(Mat *src, Mat *verbose)
{
	IplImage imgSrc = *src;
	IplImage imgRes = *verbose;

    IplImage *labelImg = cvCreateImage(cvGetSize(&imgSrc), IPL_DEPTH_LABEL, 1);

    unsigned int labelNum = cvLabel(&imgSrc, labelImg, blobs);
    cvb::cvFilterByArea(blobs, minBlobArea, maxBlobArea);
    cvb::cvRenderBlobs(labelImg, blobs, &imgSrc, &imgRes, CV_BLOB_RENDER_BOUNDING_BOX);

	if (this->trackingMode == cvblob)
	{
		OriginalUpdateTracks(blobs, tracks);
	}
	else if (this->trackingMode == motionvector)
	{
		MotionVectorBasedUpdateTracks(blobs, tracks);
	}
	else if (this->trackingMode == adaptive)
	{
		if (blobs.size()>1)
		{
			MotionVectorBasedUpdateTracks(blobs, tracks);
		}
		else
		{
			OriginalUpdateTracks(blobs, tracks);
		}
	}
	else
	{
		cout << "ERROR: unknown trackingMode!" << endl;
		return;
	}

    cvb::cvRenderTracks(tracks, &imgSrc, &imgRes, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);

    cvReleaseImage(&labelImg);

	Mat tempMat(&imgRes,true);
	if (verbose)
		tempMat.copyTo(*verbose);

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

	// Access to matrix
#define C(blob, track) close[((blob) + (track)*(nBlobs+2))]
	// Access to accumulators
#define AB(label) C((label), (nTracks))
#define AT(id) C((nBlobs), (id))
	// Access to identifications
#define IB(label) C((label), (nTracks)+1)
#define IT(id) C((nBlobs)+1, (id))
	// Access to registers
#define B(label) blobs.find(IB(label))->second
#define T(id) tracks.find(IT(id))->second

double distantBlobTrack(CvBlob const *b, CvTrack const *t)
{
 	double d1;	// blob centroid és track hozzá legközelebbi oldalának távolsága, csak az egyik irányban (x vagy y)
	if (b->centroid.x<t->minx)
	{
		if (b->centroid.y<t->miny)
			d1 = MAX(t->minx - b->centroid.x, t->miny - b->centroid.y);
		else if (b->centroid.y>t->maxy)
			d1 = MAX(t->minx - b->centroid.x, b->centroid.y - t->maxy);
		else // if (t->miny < b->centroid.y)&&(b->centroid.y < t->maxy)
			d1 = t->minx - b->centroid.x;
	}
	else if (b->centroid.x>t->maxx)
	{
		if (b->centroid.y<t->miny)
			d1 = MAX(b->centroid.x - t->maxx, t->miny - b->centroid.y);
		else if (b->centroid.y>t->maxy)
			d1 = MAX(b->centroid.x - t->maxx, b->centroid.y - t->maxy);
		else
			d1 = b->centroid.x - t->maxx;
	}
	else // if (t->minx =< b->centroid.x) && (b->centroid.x =< t->maxx)
	{
		if (b->centroid.y<t->miny)
			d1 = t->miny - b->centroid.y;
		else if (b->centroid.y>t->maxy)
			d1 = b->centroid.y - t->maxy;
		else 
			return 0.;
	}

	double d2;	// track centroid és blob oldallapra ugyanez
	if (t->centroid.x<b->minx)
	{
		if (t->centroid.y<b->miny)
			d2 = MAX(b->minx - t->centroid.x, b->miny - t->centroid.y);
		else if (t->centroid.y>b->maxy)
			d2 = MAX(b->minx - t->centroid.x, t->centroid.y - b->maxy);
		else // if (b->miny < t->centroid.y)&&(t->centroid.y < b->maxy)
			d2 = b->minx - t->centroid.x;
	}
	else if (t->centroid.x>b->maxx)
	{
		if (t->centroid.y<b->miny)
			d2 = MAX(t->centroid.x - b->maxx, b->miny - t->centroid.y);
		else if (t->centroid.y>b->maxy)
			d2 = MAX(t->centroid.x - b->maxx, t->centroid.y - b->maxy);
		else
			d2 = t->centroid.x - b->maxx;
	}
	else // if (b->minx =< t->centroid.x) && (t->centroid.x =< b->maxx)
	{
		if (t->centroid.y<b->miny)
			d2 = b->miny - t->centroid.y;
		else if (t->centroid.y>b->maxy)
			d2 = t->centroid.y - b->maxy;
		else 
			return 0.;
	}

	return MIN(d1, d2);
}

double CvBlobWrapper::confidenceBlobTrack(CvBlob const *b, CvTrack const *t)
{
	double confidence = 0.0;

	Point prev((int)t->centroid.x,(int)t->centroid.y);
	Point current((int)b->centroid.x,(int)b->centroid.y);
	confidence = context->motionVectorStorage.getConfidence(prev,current);

	return confidence;
}



void getClusterForTrack(unsigned int trackPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt);

void getClusterForBlob(unsigned int blobPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt)
{
	for (unsigned int j=0; j<nTracks; j++)
	{
		if (C(blobPos, j))
		{
			// Blob sufficiently near to track (?)
			tt.push_back(T(j));

			unsigned int c = AT(j);

			C(blobPos, j) = 0;
			AB(blobPos)--;
			AT(j)--;

			if (c>1)
			{
				getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);
			}
		}
	}
}

void getClusterForTrack(unsigned int trackPos, CvID *close, unsigned int nBlobs, unsigned int nTracks, CvBlobs const &blobs, CvTracks const &tracks, list<CvBlob*> &bb, list<CvTrack*> &tt)
{
	for (unsigned int i=0; i<nBlobs; i++)
	{
		if (C(i, trackPos))
		{
			// Blob is sufficiently near to current track (?)
			bb.push_back(B(i));

			unsigned int c = AB(i);

			C(i, trackPos) = 0;
			AB(i)--;
			AT(trackPos)--;

			if (c>1)
			{
				getClusterForBlob(i, close, nBlobs, nTracks, blobs, tracks, bb, tt);
			}
		}
	}
}

// Kri: Made global
CvID maxTrackID = 0;

void CvBlobWrapper::OriginalUpdateTracks(CvBlobs const &blobs, CvTracks &tracks)
{
	CV_FUNCNAME("OriginalUpdateTracks");
	__CV_BEGIN__;

	unsigned int nBlobs = blobs.size();
	unsigned int nTracks = tracks.size();

	// Proximity matrix:
	// Last row/column is for ID/label.
	// Last-1 "/" is for accumulation.
	CvID *close = new unsigned int[(nBlobs+2)*(nTracks+2)]; // XXX Must be same type than CvLabel.

	try
	{
		// Inicialization:
		unsigned int i=0;
		for (CvBlobs::const_iterator it = blobs.begin(); it!=blobs.end(); ++it, i++)
		{
			AB(i) = 0;
			IB(i) = it->second->label;
		}

//			CvID maxTrackID = 0;

		unsigned int j=0;
		for (CvTracks::const_iterator jt = tracks.begin(); jt!=tracks.end(); ++jt, j++)
		{
			AT(j) = 0;
			IT(j) = jt->second->id;
/*				if (jt->second->id > maxTrackID)
				maxTrackID = jt->second->id;*/
		}

		// Proximity matrix calculation and "used blob" list inicialization:
		for (i=0; i<nBlobs; i++)
			for (j=0; j<nTracks; j++)
				if (C(i, j) = (distantBlobTrack(B(i), T(j)) < thDistance))
				{
					AB(i)++;
					AT(j)++;
				}

				/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				// Detect inactive tracks
				for (j=0; j<nTracks; j++)
				{
					unsigned int c = AT(j);

					if (c==0)	// Track not close enough to any blobs
					{
						//cout << "Inactive track: " << j << endl;

						// Inactive track.
						CvTrack *track = T(j);
						track->inactive++;	// increase inactivity time (in frame number)
						track->label = 0;
					}
				}

				// Detect new tracks
				for (i=0; i<nBlobs; i++)
				{
					unsigned int c = AB(i);

					if (c==0)	// Not close enough to any of the existing tracks.
					{
						//cout << "Blob (new track): " << maxTrackID+1 << endl;
						//cout << *B(i) << endl;

						// New track.
						maxTrackID++;
						CvBlob *blob = B(i);
						CvTrack *track = new CvTrack;
						track->id = maxTrackID;
						track->label = blob->label;
						track->minx = blob->minx;
						track->miny = blob->miny;
						track->maxx = blob->maxx;
						track->maxy = blob->maxy;
						track->centroid = blob->centroid;
						track->lifetime = 0;
						track->active = 0;
						track->inactive = 0;
						tracks.insert(CvIDTrack(maxTrackID, track));
					}
				}

				// Clustering
				for (j=0; j<nTracks; j++)	// Minden track-re
				{
					// Melyik blob-ot nyeri meg? A tobbi blob inaktiv lesz.
					unsigned int c = AT(j);

					if (c)
					{
						list<CvTrack*> tt; tt.push_back(T(j));
						list<CvBlob*> bb;

						getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);

						// Select track
						//	Choosing the track with maximal size
						CvTrack *track;
						unsigned int area = 0;
						for (list<CvTrack*>::const_iterator it=tt.begin(); it!=tt.end(); ++it)
						{
							CvTrack *t = *it;

							unsigned int a = (t->maxx-t->minx)*(t->maxy-t->miny);
							if (a>area)
							{
								area = a;
								track = t;
							}
						}

						// Select blob
						//	Choosing the blob with maximal size
						CvBlob *blob;
						area = 0;
						//cout << "Matching blobs: ";
						for (list<CvBlob*>::const_iterator it=bb.begin(); it!=bb.end(); ++it)
						{
							CvBlob *b = *it;

							//cout << b->label << " ";

							if (b->area>area)
							{
								area = b->area;
								blob = b;
							}
						}
						//cout << endl;

						// Update track
						//	Update size of track based on maximal size blob
						//cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
						track->label = blob->label;
						track->centroid = blob->centroid;
						track->minx = blob->minx;
						track->miny = blob->miny;
						track->maxx = blob->maxx;
						track->maxy = blob->maxy;
						if (track->inactive)
							track->active = 0;	// If it were inactive until now, active counter is reset.
						track->inactive = 0;	// Now active, inactive counter reset.

						// Others to inactive
						for (list<CvTrack*>::const_iterator it=tt.begin(); it!=tt.end(); ++it)
						{
							CvTrack *t = *it;

							if (t!=track)
							{
								//cout << "Inactive: track=" << t->id << endl;
								t->inactive++;
								t->label = 0;
							}
						}
					}
				}
				/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

				for (CvTracks::iterator jt=tracks.begin(); jt!=tracks.end();)
					if (	(jt->second->inactive>=thInactive)
							// inactive for too long time
						||	((jt->second->inactive) && (thActive) && (jt->second->active<thActive)))
							// thActive is set and the track has not been active for sufficient time before getting inactive
					{
						delete jt->second;
						tracks.erase(jt++);
					}
					else
					{
						// Increment lifetime and activity counter
						jt->second->lifetime++;
						if (!jt->second->inactive)
							jt->second->active++;
						++jt;
					}
	}
	catch (...)
	{
		delete[] close;
		throw; // TODO: OpenCV style.
	}

	delete[] close;

	__CV_END__;
}

void CvBlobWrapper::MotionVectorBasedUpdateTracks(cvb::CvBlobs const &blobs, cvb::CvTracks &tracks)
{
	CV_FUNCNAME("MotionVectorBasedUpdateTracks");
	__CV_BEGIN__;

	unsigned int nBlobs = blobs.size();
	unsigned int nTracks = tracks.size();

	cout << "--- MotionVectorBasedUpdateTracks, nBlobs=" << nBlobs << ", nTracks=" << nTracks << endl;

	// Proximity matrix:
	// Last row/column is for ID/label.
	// Last-1 "/" is for accumulation.
	CvID *close = new unsigned int[(nBlobs+2)*(nTracks+2)]; // XXX Must be same type than CvLabel.

	try
	{
		// Inicialization:
		unsigned int i=0;
		for (CvBlobs::const_iterator it = blobs.begin(); it!=blobs.end(); ++it, i++)
		{
			AB(i) = 0;
			IB(i) = it->second->label;
		}

//			CvID maxTrackID = 0;

		unsigned int j=0;
		for (CvTracks::const_iterator jt = tracks.begin(); jt!=tracks.end(); ++jt, j++)
		{
			AT(j) = 0;
			IT(j) = jt->second->id;
/*				if (jt->second->id > maxTrackID)
				maxTrackID = jt->second->id;*/
		}

		// Proximity matrix calculation and "used blob" list inicialization:
		for (i=0; i<nBlobs; i++)
		{
			cout << "Checking BLOB " << i << " @(" << B(i)->centroid.x << ";" << B(i)->centroid.y << ")" << endl;
			for (j=0; j<nTracks; j++)
			{
				int trackID = T(j)->id;
				cout << "  Checking TRACK" << trackID << " @(" << T(j)->centroid.x << ";" << T(j)->centroid.y << ")" << endl;
				double confidence = confidenceBlobTrack(B(i), T(j));
				if (C(i, j) = (confidence >= minConfidence))
				{
					cout << "    CLOSE, conf=" << confidence << endl;
					AB(i)++;
					AT(j)++;
				}
			}
		}

		cout << "Now processing..." << endl;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Detect inactive tracks (no close blobs)
		for (j=0; j<nTracks; j++)
		{
			unsigned int c = AT(j);	// AT: Accumulator Track

			if (c==0)	// Track not close enough to any blobs
			{
				//cout << "Inactive track: " << j << endl;

				// Inactive track.
				CvTrack *track = T(j);
				track->inactive++;	// increase inactivity time (in frame number)
				track->label = 0;
			}
		}

		// Detect new tracks	(no close tracks to blob)
		for (i=0; i<nBlobs; i++)
		{
			unsigned int c = AB(i);	// AB: Accumulator Blob

			if (c==0)	// Not close enough to any of the existing tracks.
			{
				//cout << "Blob (new track): " << maxTrackID+1 << endl;
				//cout << *B(i) << endl;

				// New track.
				maxTrackID++;
				CvBlob *blob = B(i);
				CvTrack *track = new CvTrack;
				track->id = maxTrackID;
				track->label = blob->label;
				track->minx = blob->minx;
				track->miny = blob->miny;
				track->maxx = blob->maxx;
				track->maxy = blob->maxy;
				track->centroid = blob->centroid;
				track->lifetime = 0;
				track->active = 0;
				track->inactive = 0;
				tracks.insert(CvIDTrack(maxTrackID, track));
			}
		}

		// Clustering
		for (j=0; j<nTracks; j++)	// Minden track-re
		{
			// Melyik blob-ot nyeri meg? A tobbi blob inaktiv lesz.
			unsigned int c = AT(j);	// Accumulator Track

			if (c)	// If there are multiple close blobs for the same track
			{
				list<CvTrack*> tt; tt.push_back(T(j));
				list<CvBlob*> bb;

				getClusterForTrack(j, close, nBlobs, nTracks, blobs, tracks, bb, tt);

				// Select track
				//	Choosing the track with maximal size
				CvTrack *trackWithMaxSize;
				unsigned int maxArea = 0;
				for (list<CvTrack*>::const_iterator it=tt.begin(); it!=tt.end(); ++it)
				{
					CvTrack *currentTrack = *it;

					unsigned int currentArea = (currentTrack->maxx - currentTrack->minx)*(currentTrack->maxy - currentTrack->miny);
					if (currentArea>maxArea)
					{
						maxArea = currentArea;
						trackWithMaxSize = currentTrack;
					}
				}
				// Select blob
				//	Choosing the blob with maximal size
				CvBlob *blobWithMaxSize;
				maxArea = 0;
				//cout << "Matching blobs: ";
				for (list<CvBlob*>::const_iterator it=bb.begin(); it!=bb.end(); ++it)
				{
					CvBlob *currentBlob = *it;

					//cout << b->label << " ";

					if (currentBlob->area>maxArea)
					{
						maxArea = currentBlob->area;
						blobWithMaxSize = currentBlob;
					}
				}
				//cout << endl;

				// Update track
				//	Update size of track based on maximal size blob
				//cout << "Matching: track=" << track->id << ", blob=" << blob->label << endl;
				trackWithMaxSize->label = blobWithMaxSize->label;
				trackWithMaxSize->centroid = blobWithMaxSize->centroid;
				trackWithMaxSize->minx = blobWithMaxSize->minx;
				trackWithMaxSize->miny = blobWithMaxSize->miny;
				trackWithMaxSize->maxx = blobWithMaxSize->maxx;
				trackWithMaxSize->maxy = blobWithMaxSize->maxy;
				if (trackWithMaxSize->inactive)
					trackWithMaxSize->active = 0;	// If it were inactive until now, active counter is reset.
				trackWithMaxSize->inactive = 0;	// Now active, inactive counter reset.

				// Others to inactive
				for (list<CvTrack*>::const_iterator it=tt.begin(); it!=tt.end(); ++it)
				{
					CvTrack *t = *it;

					if (t!=trackWithMaxSize)
					{
						//cout << "Inactive: track=" << t->id << endl;
						t->inactive++;
						t->label = 0;
					}
				}
			}
		}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		for (CvTracks::iterator jt=tracks.begin(); jt!=tracks.end();)
			if (	(jt->second->inactive>=thInactive)
					// inactive for too long time
				||	((jt->second->inactive) && (thActive) && (jt->second->active<thActive)))
					// thActive is set and the track has not been active for sufficient time before getting inactive
			{
				delete jt->second;
				tracks.erase(jt++);
			}
			else
			{
				// Increment lifetime and activity counter
				jt->second->lifetime++;
				if (!jt->second->inactive)
					jt->second->active++;
				++jt;
			}
	}
	catch (...)
	{
		delete[] close;
		throw; // TODO: OpenCV style.
	}

	delete[] close;

	__CV_END__;
}
