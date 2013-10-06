#ifndef MSER_h
#define MSER_h

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include <limits>

// for TimeKeeper
#include "funcs.h"
#include "Time.h"

namespace ICSL
{

CV_EXPORTS bool initModule_features2d();

/*!
 Maximal Stable Extremal Regions class.

 The class implements MSER algorithm introduced by J. Matas.
 Unlike SIFT, SURF and many other detectors in OpenCV, this is salient region detector,
 not the salient point detector.

 It returns the regions, each of those is encoded as a contour.
*/
class CV_EXPORTS_W MSER// : public FeatureDetector
{
public:
    //! the full constructor
    CV_WRAP explicit MSER( int _delta=5, int _min_area=60, int _max_area=14400,
          double _max_variation=0.25, double _min_diversity=.2,
          int _max_evolution=200, double _area_threshold=1.01,
          double _min_margin=0.003, int _edge_blur_size=5 );

    //! the operator that extracts the MSERs from the image or the specific part of it
    CV_WRAP_AS(detect) void operator()( const cv::Mat& image, CV_OUT std::vector<std::vector<cv::Point> >& msers,
                                        const cv::Mat& mask=cv::Mat() ) const;
//    AlgorithmInfo* info() const;

protected:
    void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const;

    int delta;
    int minArea;
    int maxArea;
    double maxVariation;
    double minDiversity;
    int maxEvolution;
    double areaThreshold;
    double minMargin;
    int edgeBlurSize;
};

typedef MSER MserFeatureDetector;
} /* namespace cv */

#endif
