#ifndef ICSL_MSER
#define ICSL_MSER
/*!
 Maximal Stable Extremal Regions class.

 The class implements MSER algorithm introduced by J. Matas.
 Unlike SIFT, SURF and many other detectors in OpenCV, this is salient region detector,
 not the salient point detector.

 It returns the regions, each of those is encoded as a contour.
*/

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/core/internal.hpp>
#include <opencv2/core/types_c.h>

namespace ICSL {
class MSER : public cv::FeatureDetector
{
public:
    //! the full constructor
    explicit MSER( int _delta=5, int _min_area=60, int _max_area=14400,
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

} // namespace ICSL
#endif
