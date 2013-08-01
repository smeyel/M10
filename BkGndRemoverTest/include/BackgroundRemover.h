#ifndef __BACKGROUNDREMOVER_H
#define __BACKGROUNDREMOVER_H
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat)

namespace smeyel
{
	class BackgroundRemover
	{
	public:
		virtual void removeBackground(cv::Mat &src, cv::Mat &dstMask) = 0;
	};
}

#endif
