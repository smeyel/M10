#include "MotionVector.h"

#include "MotionVectorStorage.h"

#ifndef MAX
#  define MAX(a,b)  ((a) < (b) ? (b) : (a))
#endif

// Default constructor used by loading from FileStorage
MotionVector::MotionVector()
{
}

MotionVector::MotionVector(Point src, Point dst)
{
	this->src = src;
	this->dst = dst;
}

Point MotionVector::getSrc()
{
	return src;
}

Point MotionVector::getDst()
{
	return dst;
}

double MotionVector::getLinearWeight(Point p1, Point p2)
{
	double sqrDistance = sqrt((double)((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y)));
	return MAX( 1.0-(sqrDistance/WEIGHTMAXDISTANCE) , 0.);
}

float MotionVector::length(Point2f v)	// static
{
	return (float)sqrt(v.x*v.x + v.y*v.y);
}

float MotionVector::length()
{
	float dx = this->dst.x - this->src.x;
	float dy = this->dst.y - this->src.y;
	return (float)sqrt(dx*dx + dy*dy);
}

double MotionVector::getDirectionSensitiveLinearWeight(Point2f otherV, float tangentialMultiplier)	// otherV: other velocity vector
{
	// Calculate unity vector corresponding the direction of (this->src;this->dst)
	Point2f currentV((float)(dst.x-src.x), (float)(dst.y-src.y));
	float currentLen = length(currentV);
	Point2f currentVUnity(currentV.x/currentLen, currentV.y/currentLen);

	float scalarProduct = (otherV.x * currentVUnity.x) + (otherV.y * currentVUnity.y);

	// Component of otherV tangential to currentV
	Point2f otherVTangential(currentVUnity.x * scalarProduct, currentVUnity.y * scalarProduct);

	// Component of otherV perpendicular to currentV
	Point2f otherVPerpendicular(otherV.x - otherVTangential.x, otherV.y - otherVTangential.y);

	Point2f tangentialDifference(otherVTangential.x-currentV.x,otherVTangential.y-currentV.y);
	// perpendicular difference is zero.

	float tangentialLength = length(tangentialDifference) * tangentialMultiplier;
	float perpendicularLength = length(otherVPerpendicular);

	double distance = length(Point2f(tangentialLength,perpendicularLength));
	return MAX( 1.0-(distance/WEIGHTMAXDISTANCE) , 0.);
}

double MotionVector::getWeight(Point p, SourceOrDestination srcOrDest)
{
	Point p2 = (srcOrDest == Source ? src : dst);
	return getLinearWeight(p,p2);
}

double MotionVector::getWeight(Point p1, Point p2)
{
	double linearTerm1 = getLinearWeight(src,p1);
	double linearTerm2 = getLinearWeight(dst,p2);
	return linearTerm1 * linearTerm2;
}

double MotionVector::getDirectionSensitiveWeight(Point otherSrc, Point otherDst)
{
	// Weight of source point differences
	double srcWeight = getWeight(otherSrc,MotionVector::SourceOrDestination::Source);

	// Weight of speed vector differences
	Point2f otherV((float)(otherDst.x-otherSrc.x),(float)(otherDst.y-otherSrc.y));
	double dstWeight = getDirectionSensitiveLinearWeight(otherV);

	double weight = srcWeight * dstWeight;

	return weight;
}

void MotionVector::save(FileStorage *fs)
{
	*fs << "{" << "src" << src << "dst" << dst << "}";
}

void MotionVector::load(FileNode *node)
{
	FileNode srcNode = (*node)["src"];
	src = Point(srcNode[0],srcNode[1]);
	FileNode dstNode = (*node)["dst"];
	dst = Point(dstNode[0],dstNode[1]);
}

float MotionVector::getConfidence(MotionVectorStorage *storage)
{
	return storage->getConfidence(this->src, this->dst);
}

bool MotionVector::isTheSame(Point src, Point dst)
{
	return (this->src.x == src.x
		&& this->src.y == src.y
		&& this->dst.x == dst.x
		&& this->dst.y == dst.y);
}
