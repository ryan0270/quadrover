//////////////////////////////////
// Lim Hyon's feature matcher
/////////////////////////////////
#ifndef _MATCHER_H_
#define _MATCHER_H_
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tbb/parallel_for.h"

#include <vector>
#include <list>
#include <set>
#include <algorithm>
#include <iostream>

#include "Time.h"

//#include "BRIEF.h"

namespace ICSL{
namespace Quadrotor{

class Feature
{
public:
	cv::KeyPoint keypoint;
	cv::Mat descriptor;
//	BRIEF::briefbit brief_descriptor;

    /// Scoring related variables
    float scoreFirst, scoreSecond;
    float scoreFirst_back, scoreSecond_back;
    int bestPos, bestPos_back;      // best index of forward match

    Feature() { ResetScore(); }

    /// Copy constructor
    Feature(const Feature& feat);
    void ResetScore(void);
};

class KeyFrame
{
public:
        int id;
        std::vector<Feature> features;  //! feature.
};

class Matcher
{
	public:
	int timeSum1, timeSum2;
	vector<cv::Mat> frames;
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;

    //! Keyframe
    std::vector<KeyFrame> vKeyFrames;

/// previous, current feature vector
    std::vector<Feature> vPrevFeatures;
    std::vector<Feature> vCurrentFeatures;
    std::vector<unsigned int> vKeypointRowLUT;

    /// In case BRIEF or FREAK or ORB are used, this flag will turned on.
    bool bUseHammingDistance;

    // structure for storing pairwise match information.
    struct p_match {
            float u1p, v1p; // u,v-coordinates in previous left  image
            int32_t i1p;     // feature index (for tracking)
            float u1c, v1c; // u,v-coordinates in current  left  image
            int32_t i1c;     // feature index (for tracking)
            bool isInlier;

            p_match() {
                    u1p = v1p = -1;
                    i1p = -1;
                    u1c = v1c = -1;
                    i1c = -1;
                    isInlier = true;
            }
            p_match(float u1p, float v1p, int32_t i1p, float u2p, float v2p,
                            int32_t i2p, float u1c, float v1c, int32_t i1c, float u2c,
                            float v2c, int32_t i2c,bool isInlier) :
                            u1p(u1p), v1p(v1p), i1p(i1p), u1c(u1c), v1c(v1c), i1c(i1c), isInlier(isInlier)  {
            }
    };

    // parameter settings
    struct parameters {

            int32_t maxFeatureNum;
            float ratio_threshold;
            int32_t match_binsize; // matching bin width/height (affects efficiency only)
            int32_t match_radius;           // matching radius (du/dv in pixels)
            int32_t match_disp_tolerance; // dv tolerance for stereo matches (in pixels)
            int32_t half_resolution; // 0=disabled,1=match at half resolution, refine at full resolution
            int32_t minMatches;
            int32_t minHammingDist;

            // default settings
            parameters() {
                    ratio_threshold = 0.65f; // lower is more strict
                    match_radius = 60; // distance between pixels
                    maxFeatureNum = 1500;
                    match_binsize = 60;
                    match_disp_tolerance = 2;
                    half_resolution = 0;
                    minMatches = 20;
                    minHammingDist = 60;
            }
    };

    struct FeatureSorter {
                    bool operator()(const Feature& lhs, const Feature& rhs) {
                            return lhs.keypoint.pt.y < rhs.keypoint.pt.y;
                    }
            };

    /// Parameters
    parameters params;
    std::string strMatcherName;
    std::string strDescriptorName;

    //! Matches
    std::vector<p_match> vMatches;

    unsigned int keyFrameId;

        //! Keyframe relative
    int addKeyframe(void)
    {
            KeyFrame keyframe;
            keyframe.features = vCurrentFeatures;
            vKeyFrames.push_back(keyframe);
            return keyFrameId++;
    }

	/// Constructor
	Matcher(const std::string& detectorType,
			const std::string& extractorType);
	
		//! input:
	//! if (replace == false) : do not replace previous frame.
	//! if (replace == true) : replace previous frame.
	//!
	//! return:
	//! -1 : failed
	//! +1 : success
	int Track(const cv::Mat &inputImg, bool replace);
};
}
}

#endif
