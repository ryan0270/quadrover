#ifndef _MATCHER_H_
#define _MATCHER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vector>
#include <list>
#include <set>
#include <algorithm>
#include <iostream>

#include "BRIEF.h"

namespace ICSL{
namespace Quadrotor{

class Feature
{
public:
	cv::KeyPoint keypoint;
	cv::Mat descriptor;
	BRIEF::briefbit brief_descriptor;

        /// Scoring related variables
        float scoreFirst, scoreSecond;
        float scoreFirst_back, scoreSecond_back;
        int bestPos, bestPos_back;      // best index of forward match

        Feature()
        {
                ResetScore();
        }

        /// Copy constructor
        Feature(const Feature& feat)
        {
                keypoint = feat.keypoint;
                feat.descriptor.copyTo(descriptor);
                scoreFirst = feat.scoreFirst;
                scoreSecond = feat.scoreSecond;
                scoreFirst_back = feat.scoreFirst_back;
                scoreSecond_back = feat.scoreSecond_back;
                bestPos = feat.bestPos;
                bestPos_back = feat.bestPos_back;
		brief_descriptor = feat.brief_descriptor;
        }

        void ResetScore(void)
        {
                scoreFirst = 10000;
                scoreSecond = 10000;
                bestPos = -1;

                scoreFirst_back = 10000;
                scoreSecond_back = 10000;
                bestPos_back = -1;
        }
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
                        ratio_threshold = 0.85f;
                        maxFeatureNum = 1500;
                        match_binsize = 60;
                        match_radius = 60;
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
			const std::string& extractorType): keyFrameId(0)
	{
		bUseHammingDistance = false;
		strMatcherName = detectorType;
		strDescriptorName = extractorType;

		//std::cout << "Matcher : " << strMatcherName << std::endl;
		//std::cout << "Descriptor : " << strDescriptorName << std::endl;

		if (!detectorType.compare("HARRIS"))
		{
			detector = new cv::GoodFeaturesToTrackDetector(params.maxFeatureNum, /* number of maximum feature which will be detected. */
					0.01, /* quality level? */
					15, /* minimum distance? */
					3, /* block size? */
					false, /* use harris detector? if yes, below threshold will be used. */
					0.02 /* harris threshold */
					);
		}
		else if (!detectorType.compare("FAST"))
		{
			detector = new cv::FastFeatureDetector();
		}
		else if (!detectorType.compare("FAST_GRID")) {
			//! 1226 x 370
			detector = new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector(15));//, params.maxFeatureNum, 5, 5);
		}
		else if (!detectorType.compare("FAST_PYRAMID")) {

			detector = new cv::PyramidAdaptedFeatureDetector
									(new cv::FastFeatureDetector(50),3);

			//detector = new cv::PyramidAdaptedFeatureDetector(cv::FeatureDetector::create("FAST"),3);

			//detector = new cv::FeatureDetector::create("PyramidFAST",30);
		}
		else if (!detectorType.compare("HARRIS_PYRAMID")) {
			int pyr_level = 2;
			detector = new cv::PyramidAdaptedFeatureDetector
							(new cv::GridAdaptedFeatureDetector(
									new cv::GoodFeaturesToTrackDetector(params.maxFeatureNum, /* number of maximum feature which will be detected. */
									0.01, /* quality level? */
									10, /* minimum distance between maximal point. */
									3, /* block size? */
									true, /* use harris detector? */
									0.02 /* 0.04 harris threshold */
									), params.maxFeatureNum, /* number of maximum feature throughout the bucket*/
									5, /* bucket size rows */
									5 /* bucket size cols */
									), pyr_level);
		}
		else if (!detectorType.compare("HARRIS_GRID")) {
				/// In the D.Nister paper, he have used 10x10 grid and allow 100 features per bucket.
				detector = new cv::GridAdaptedFeatureDetector(
						new cv::GoodFeaturesToTrackDetector(100, /* number of maximum feature which will be detected. */
						0.01, /* quality level? */
						10, /* minimum distance between maximal point. */
						3, /* block size? */
						true, /* use harris detector? */
						0.04 /* 0.04 harris threshold */
						), params.maxFeatureNum, /* number of maximum feature throughout the bucket*/
						7, /* bucket size rows */
						7 /* bucket size cols */
						);
		}
		else if (!detectorType.compare("ORB")) {
			detector =
					new cv::OrbFeatureDetector(params.maxFeatureNum,
													2.0f, /* pyramid factor */
													4,		/* max pyramid */
													31,
													0,		/* first level */
													2,		/* WTK */
													cv::ORB::HARRIS_SCORE,
													31);
		}
		else if (!detectorType.compare("STAR")) {
			//detector = new cv::OrbFeatureDetector(params.maxFeatureNum,1.2f,8,13,0,2,cv::ORB::HARRIS_SCORE,16);
			//detector = new cv::StarDetector();
			detector = cv::FeatureDetector::create("PyramidSTAR");
		}
		else if (!detectorType.compare("DENSE")) {
			detector = new cv::DenseFeatureDetector();
		}
		else if (!detectorType.compare("SIMPLEBLOB")) {
			detector = new cv::SimpleBlobDetector();
		}
		else
		{
			detector = cv::FeatureDetector::create(detectorType);
		}

		if (!extractorType.compare("BRIEF")) {
			extractor = new cv::BriefDescriptorExtractor(32);
			bUseHammingDistance = true;
		}
		else if (!extractorType.compare("ORB")) {
			extractor = new cv::OrbFeatureDetector(500,
					2.0f, /* pyramid factor */
					4,		/* max pyramid */
					31,
					0,		/* first level */
					2,		/* WTK */
					cv::ORB::HARRIS_SCORE,
					31);
			bUseHammingDistance = true;
		}
		else
		{
			extractor = new cv::BriefDescriptorExtractor(32);
			bUseHammingDistance = true;
		}
	};

		//! input:
	//! if (replace == false) : do not replace previous frame.
	//! if (replace == true) : replace previous frame.
	//!
	//! return:
	//! -1 : failed
	//! +1 : success
	int Track(const cv::Mat &inputImg, bool replace)
	{
		std::vector<cv::KeyPoint> kpts;
		cv::Mat descs;
		cv::Mat frame;
		
		//! perform half resolution tracking
		if(params.half_resolution)
		{
			//std::cout << "### MAKE IMAGE HALF!\n";
			cv::resize(inputImg,frame,cv::Size(),0.5,0.5,cv::INTER_LANCZOS4);
		}
		else
		{
			inputImg.copyTo(frame);
		}

		detector->detect(frame,kpts);

		if(kpts.size() < 10)
			return -3;

		//! In case of good tracking condition
		if(!vCurrentFeatures.empty() && !replace)	//! in case of no replace, we track prev-prev-frame.
		{
			//std::cout << "\nMake it previous!\n";
			vPrevFeatures = vCurrentFeatures;	/// Make it previous
		}
		else
		{
			//std::cout << "\nreplace : " << replace << std::endl;
			//std::cout << "\nUse existing " << vCurrentFeatures.empty() << std::endl;
		}


		//! detect keypoints and extract descriptors.
		extractor->compute(frame,kpts,descs);

		vCurrentFeatures.clear();

		//! Allocate current feature vector
		for(unsigned int i=0;i<kpts.size();i++)
		{
			Feature feat;
			feat.keypoint = kpts[i];
			descs.row(i).copyTo(feat.descriptor);
			vCurrentFeatures.push_back(feat);
		}

		if(vPrevFeatures.empty())	//! In case of first frame
		{
			//std::cout << "\nMATCHER INITIALIZATION\n";
			vPrevFeatures = vCurrentFeatures;	//! Make it previous (first frame)
			std::sort(vPrevFeatures.begin(), vPrevFeatures.end(), FeatureSorter());	//! sort for lookup table.
			return -1;
		}

		//! Make image row LUT of previous image.
		vKeypointRowLUT.clear();

		unsigned int v = 0;
		for (int y = 0; y < frame.rows; y++) {
			while (v < (vPrevFeatures.size() - 1) && y > vPrevFeatures[v].keypoint.pt.y)
				v++;
			vKeypointRowLUT.push_back(v);
		}


		//! Comparison between current and previous
		for(std::vector<Feature>::iterator it=vCurrentFeatures.begin(); it!= vCurrentFeatures.end();++it)
		{
			//! Reset score.
			it->ResetScore();

			int32_t match_binsize;

			if(params.half_resolution)
				match_binsize = params.match_binsize/2;
			else
				match_binsize = params.match_binsize;

			//! boundary computation
			const cv::KeyPoint &currentKpt = it->keypoint;
			int nCenter = currentKpt.pt.x;
			int nTop = currentKpt.pt.y - match_binsize;
			int nBottom = currentKpt.pt.y + match_binsize; // because we'd like to search before +1
			int nLeft = nCenter - match_binsize;
			int nRight = nCenter + match_binsize;

			//! Simple rejection framework
			if (nTop < 0)
				nTop = 0;
			if (nTop >= frame.rows)
				continue;
			if (nBottom <= 0)
				continue;

			//! Assign iterators
			std::vector<Feature>::iterator it_start;
			std::vector<Feature>::iterator it_end;

			it_start = vPrevFeatures.begin() + vKeypointRowLUT[nTop];

			if (nBottom >= frame.rows)
				it_end = vPrevFeatures.end();
			else
				it_end = vPrevFeatures.begin() + vKeypointRowLUT[nBottom];

			//! TODO: Can we make this multi-threaded?
			//! Sweep over iterators.
			for (; it_start < it_end; it_start++)
			{
				if (it_start->keypoint.pt.x < nLeft || it_start->keypoint.pt.x > nRight)
					continue;

				int dx = currentKpt.pt.x - it_start->keypoint.pt.x;
				int dy = currentKpt.pt.y - it_start->keypoint.pt.y;

				int32_t match_radius;

				if(params.half_resolution)
					match_radius = params.match_radius/2;
				else
					match_radius = params.match_radius;

				//! Simple rejection framework
				if (dx * dx + dy * dy > match_radius*match_radius)
				{
					continue;
				}

				int idx = std::distance(vPrevFeatures.begin(),it_start);

				float r = 10000.0;

				if (bUseHammingDistance)
					r = cv::normHamming(it_start->descriptor.data,it->descriptor.data,extractor->descriptorSize());
				else
					//! SIFT or SURF
					r = std::sqrt(cv::normL2Sqr_(it_start->descriptor.ptr<float>(),it->descriptor.ptr<float>(),extractor->descriptorSize()));

				if (r < it->scoreFirst)
				{
					it->scoreSecond = it->scoreFirst;
					it->scoreFirst = r;
					it->bestPos = idx; //! set best previous position, initial value -1
				} else if (r > it->scoreFirst && r < it->scoreSecond)
				{
					it->scoreSecond = r;
				}
			} //! end inner for
		} //! end outer for

		/// Make image row LUT of current image.
		vKeypointRowLUT.clear();
		std::sort(vCurrentFeatures.begin(), vCurrentFeatures.end(), FeatureSorter());

		v = 0;	//! reuse v.
		for (int y = 0; y < frame.rows; y++)
		{
			while (v < (vCurrentFeatures.size() - 1) && y > vCurrentFeatures[v].keypoint.pt.y)
				v++;
			vKeypointRowLUT.push_back(v);
		}

		//! Backward match
		for(std::vector<Feature>::iterator it=vPrevFeatures.begin(); it!= vPrevFeatures.end();++it)
		{
			const cv::KeyPoint &currentKpt = it->keypoint;
			int nCenter = currentKpt.pt.x;

			int32_t match_binsize;

			if(params.half_resolution)
				match_binsize = params.match_binsize/2;
			else
				match_binsize = params.match_binsize;

			int nTop = currentKpt.pt.y - match_binsize;
			int nBottom = currentKpt.pt.y + match_binsize; // because we'd like to search before +1
			int nLeft = nCenter - match_binsize;
			int nRight = nCenter + match_binsize;

			//! Simple rejection framework
			if (nTop < 0)
				nTop = 0;
			if (nTop >= frame.rows)
				continue;
			if (nBottom <= 0)
				continue;

			std::vector<Feature>::iterator it_start;
			std::vector<Feature>::iterator it_end;

			it_start = vCurrentFeatures.begin() + vKeypointRowLUT[nTop];

			if (nBottom >= frame.rows)
				it_end = vCurrentFeatures.end();
			else
				it_end = vCurrentFeatures.begin() + vKeypointRowLUT[nBottom];

			/// Current;
			for (; it_start < it_end; it_start++)
			{
				if (it_start->keypoint.pt.x < nLeft || it_start->keypoint.pt.x > nRight)
					continue;

				int dx = currentKpt.pt.x - it_start->keypoint.pt.x;
				int dy = currentKpt.pt.y - it_start->keypoint.pt.y;

				int32_t match_radius;

				if(params.half_resolution)
					match_radius = params.match_radius/2;
				else
					match_radius = params.match_radius;

				if (dx * dx + dy * dy > match_radius*match_radius)
				{
					continue;
				}

				int idx = std::distance(vCurrentFeatures.begin(),it_start);
				float r = 10000.0;

				if (bUseHammingDistance)
					r = cv::normHamming(it_start->descriptor.data,it->descriptor.data,extractor->descriptorSize());
				else
					r = std::sqrt(cv::normL2Sqr_(it_start->descriptor.ptr<float>(),it->descriptor.ptr<float>(),extractor->descriptorSize()));

				if (r < it_start->scoreFirst_back)
				{
					it_start->scoreSecond_back = it_start->scoreFirst_back;
					it_start->scoreFirst_back = r;
					it_start->bestPos_back = idx; // set best next position, initial value -1
				} else if (r > it_start->scoreFirst_back && r < it_start->scoreSecond_back)
				{
					it_start->scoreSecond_back = r;
				}
			} //! end inner for
		} //! end outer for

		//! feature marriage
		vMatches.clear();
		std::set<int> featurePool;

		for(std::vector<Feature>::iterator it=vCurrentFeatures.begin(); it!= vCurrentFeatures.end();++it)
		{
			int idx = std::distance(vCurrentFeatures.begin(),it);

			if ((float) (it->scoreFirst / it->scoreSecond) < params.ratio_threshold
							&& it->bestPos_back == idx) /* feature marriage check */
			{
				if(bUseHammingDistance)
				{
					if(it->scoreFirst > params.minHammingDist)
						continue;
				}

				float factor;

				if(params.half_resolution)
					factor = 2.0;
				else
					factor = 1.0;

				/// Skip duplicated feature.
				if(featurePool.count(it->bestPos) > 0)
					continue;

				/// p_match preparation
				p_match match;

				/// from current frame.
				match.u1c = it->keypoint.pt.x*factor;
				match.v1c = it->keypoint.pt.y*factor;
				match.i1c = idx;	/// index number in current frame.

				/// from previous frame.
				match.u1p = vPrevFeatures[it->bestPos].keypoint.pt.x*factor;
				match.v1p = vPrevFeatures[it->bestPos].keypoint.pt.y*factor;
				match.i1p = it->bestPos;	/// index number in previous frame.

				featurePool.insert(it->bestPos);

				/// Pushback
				vMatches.push_back(match);
			}
		}

		//! in case of agreed matches are below minMatches, return negative.
		if(featurePool.size() < params.minMatches)
		{
			//std::cout << "Less than min matches!\n";
			return 2;
		}
		else
			return 1;
	} //! end of function Track()

};
}
}

#endif
