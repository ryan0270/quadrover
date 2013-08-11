#include "BlobDetector.h"

namespace ICSL {
namespace Quadrotor {
using namespace cv;
BlobDetector::BlobDetector()
{
    thresholdStep = 10;
    minThreshold = 50;
    maxThreshold = 220;
    minRepeatability = 2;
    minDistBetweenBlobs = 10;

    filterByColor = true;
    blobColor = 0;

    filterByArea = true;
    minArea = 25;
    maxArea = 5000;

    filterByCircularity = false;
    minCircularity = 0.8f;
    maxCircularity = std::numeric_limits<float>::max();

    filterByInertia = true;
    //minInertiaRatio = 0.6;
    minInertiaRatio = 0.1f;
    maxInertiaRatio = std::numeric_limits<float>::max();

    filterByConvexity = true;
    //minConvexity = 0.8;
    minConvexity = 0.95f;
    maxConvexity = std::numeric_limits<float>::max();
}

BlobDetector::~BlobDetector()
{
}

void BlobDetector::findBlobs(const cv::Mat &image, const cv::Mat &binaryImage, std::vector<Blob> &centers) const
{
    (void)image;
    centers.clear();

    std::vector < std::vector<Point> > contours;
    Mat tmpBinaryImage = binaryImage.clone();
//    findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    findContours(tmpBinaryImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        Blob center;
		center.contour.clear();
		// have to do this manually since OpenCV won't cast it
		for(int i=0; i<contours[contourIdx].size(); i++)
			center.contour.push_back(contours[contourIdx][i]);
//		center.contour = contours[contourIdx];
        center.confidence = 1;
        Moments moms = moments(Mat(contours[contourIdx]));
        if (filterByArea)
        {
            double area = moms.m00;
            if (area < minArea || area >= maxArea)
                continue;
        }

        if (filterByCircularity)
        {
            double area = moms.m00;
            double perimeter = arcLength(Mat(contours[contourIdx]), true);
            double ratio = 4 * CV_PI * area / (perimeter * perimeter);
            if (ratio < minCircularity || ratio >= maxCircularity)
                continue;
        }

        if (filterByInertia)
        {
            double denominator = std::sqrt(std::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
            const double eps = 1e-2;
            double ratio;
            if (denominator > eps)
            {
                double cosmin = (moms.mu20 - moms.mu02) / denominator;
                double sinmin = 2 * moms.mu11 / denominator;
                double cosmax = -cosmin;
                double sinmax = -sinmin;

                double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
                double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
                ratio = imin / imax;
            }
            else
            {
                ratio = 1;
            }

            if (ratio < minInertiaRatio || ratio >= maxInertiaRatio)
                continue;

            center.confidence = ratio * ratio;
        }


		double area = contourArea(Mat(contours[contourIdx]));
		center.area = area;
        if (filterByConvexity)
        {
            std::vector < Point > hull;
            convexHull(Mat(contours[contourIdx]), hull);
            double hullArea = contourArea(Mat(hull));
            double ratio = area / hullArea;
            if (ratio < minConvexity || ratio >= maxConvexity)
                continue;
        }

        center.location = Point2f(moms.m10 / moms.m00, moms.m01 / moms.m00);

        if (filterByColor)
        {
            if (binaryImage.at<uchar> (cvRound(center.location.y), cvRound(center.location.x)) != blobColor)
                continue;
        }

        //compute blob radius
        {
            std::vector<double> dists;
            for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            {
                Point2f pt = contours[contourIdx][pointIdx];
                dists.push_back(norm(center.location - pt));
            }
            std::sort(dists.begin(), dists.end());
            center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        centers.push_back(center);

    }
}

void BlobDetector::detectImpl(const cv::Mat& image, std::vector<Blob> &blobs)
{
    //TODO: support mask
    blobs.clear();
    Mat grayscaleImage;
//    if (image.channels() == 3)
//        cvtColor(image, grayscaleImage, CV_BGR2GRAY);
//    else
//        grayscaleImage = image;
//
//    std::vector < std::vector<Blob> > centers;
//    for (double thresh = minThreshold; thresh < maxThreshold; thresh += thresholdStep)
//    {
//        Mat binarizedImage;
//        threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);
//
//        std::vector < Blob > curBlobs;
//        findBlobs(grayscaleImage, binarizedImage, curBlobs);
//        std::vector < std::vector<Blob> > newBlobs;
//        for (size_t i = 0; i < curBlobs.size(); i++)
//        {
//            bool isNew = true;
//            for (size_t j = 0; j < centers.size(); j++)
//            {
//                double dist = norm(centers[j][ centers[j].size() / 2 ].location - curBlobs[i].location);
//                isNew = dist >= minDistBetweenBlobs && dist >= centers[j][ centers[j].size() / 2 ].radius && dist >= curBlobs[i].radius;
//                if (!isNew)
//                {
//                    centers[j].push_back(curBlobs[i]);
//
//                    size_t k = centers[j].size() - 1;
//                    while( k > 0 && centers[j][k].radius < centers[j][k-1].radius )
//                    {
//                        centers[j][k] = centers[j][k-1];
//                        k--;
//                    }
//                    centers[j][k] = curBlobs[i];
//
//                    break;
//                }
//            }
//            if (isNew)
//            {
//                newBlobs.push_back(std::vector<Blob> (1, curBlobs[i]));
//                //centers.push_back(std::vector<Blob> (1, curBlobs[i]));
//            }
//        }
//        std::copy(newBlobs.begin(), newBlobs.end(), std::back_inserter(centers));
//    }
//
//    for (size_t i = 0; i < centers.size(); i++)
//    {
//        if (centers[i].size() < minRepeatability)
//            continue;
//        Point2f sumPoint(0, 0);
//        double normalizer = 0;
//        for (size_t j = 0; j < centers[i].size(); j++)
//        {
//            sumPoint += centers[i][j].confidence * centers[i][j].location;
//            normalizer += centers[i][j].confidence;
//        }
//        sumPoint *= (1. / normalizer);
//		Blob b;
//		b.location = sumPoint;
//		b.radius = centers[i][centers[i].size()/2].radius;
//		b.area = centers[i][centers[i].size()/2].area;
//		b.contour = centers[i][centers[i].size()/2].contour;
//		blobs.push_back(b);
////        KeyPoint kpt(sumPoint, (float)(centers[i][centers[i].size() / 2].radius));
////        keypoints.push_back(kpt);
//    }
}

} // namespace Quadrotor
} // namespace ICSL
