// adapted from the OpenCV SimpleBlobDetector
#ifndef ICSL_BLOBDETECTOR
#define ICSL_BLOBDETECTOR
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "toadlet/egg.h"

namespace ICSL {
namespace Quadrotor {
using namespace std;

class BlobDetector
{
	public:
	struct Blob
	{
		cv::Point2f location;
		double radius, area;
		double confidence;
		vector<cv::Point2f> contour;

		cv::Point2f locationCorrected;
		double radiusCorrected, areaCorrected;
		vector<cv::Point2f> contourCorrected;
	};

	public:
	BlobDetector();
	virtual ~BlobDetector();

	void findBlobs(const cv::Mat &image, const cv::Mat &binaryImage, std::vector<Blob> &centers) const;
	void detectImpl(const cv::Mat& image, std::vector<Blob> &blobs);

    float thresholdStep, minThreshold, maxThreshold, minRepeatability, minDistBetweenBlobs;

    bool filterByColor;
    int blobColor;

    bool filterByArea;
    float minArea, maxArea;

    bool filterByCircularity;
    float minCircularity, maxCircularity;

    bool filterByInertia;
    float minInertiaRatio, maxInertiaRatio;

    bool filterByConvexity;
    float minConvexity, maxConvexity;
};

} // namespace Quadrotor
} // namespace ICSL
#endif // ICSL_BLOBDETECTOR
