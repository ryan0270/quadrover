#include "Matcher.h"

namespace ICSL{
namespace Quadrotor{
// Copy constructor
Feature::Feature(const Feature& feat)
{
	keypoint = feat.keypoint;
	feat.descriptor.copyTo(descriptor);
	scoreFirst = feat.scoreFirst;
	scoreSecond = feat.scoreSecond;
	scoreFirst_back = feat.scoreFirst_back;
	scoreSecond_back = feat.scoreSecond_back;
	bestPos = feat.bestPos;
	bestPos_back = feat.bestPos_back;
//	brief_descriptor = feat.brief_descriptor;
}

void Feature::ResetScore(void)
{
	scoreFirst = 10000;
	scoreSecond = 10000;
	bestPos = -1;

	scoreFirst_back = 10000;
	scoreSecond_back = 10000;
	bestPos_back = -1;
}

Matcher::Matcher(const std::string& detectorType,
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
}

	//! input:
//! if (replace == false) : do not replace previous frame.
//! if (replace == true) : replace previous frame.
//!
//! return:
//! -1 : failed
//! +1 : success
int Matcher::Track(const cv::Mat &inputImg, bool replace)
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


	cv::Size sz = frame.size();
 	vector<cv::Rect> masks;
	vector<cv::Point2f> maskOffset;
	float nGrid = 4;
 	for(int i=0; i<nGrid; i++)
 	{
 		for(int j=0; j<nGrid; j++)
 		{
 			cv::Rect mask((int)(i*sz.width/nGrid+0.5), (int)(j*sz.height/nGrid+0.5), (int)(sz.width/nGrid+0.5), (int)(sz.height/nGrid+0.5));
			cv::Point2f offset((int)(i*sz.width/nGrid+0.5), (int)(j*sz.height/nGrid+0.5));
 			masks.push_back(mask);
			maskOffset.push_back(offset);
 		}
 	}

	vector<vector<cv::KeyPoint> > kptsList(masks.size());
	tbb::parallel_for(size_t(0), size_t(masks.size()), [&](size_t i){
		detector->detect(frame(masks[i]), kptsList[i]);
	});
	// adjust keypoints to account for the mask offset
	int kptCnt = 0;
	for(int i=0; i<kptsList.size(); i++)
		for(int j=0; j<kptsList[i].size(); j++)
		{
			kptsList[i][j].pt = maskOffset[i]+kptsList[i][j].pt;
			kptCnt++;
			kpts.push_back(kptsList[i][j]);
		}

//	detector->detect(frame,kpts);

//	if(kptCnt < 10)
//		return -3;

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
//	vector<cv::Mat> descList(masks.size());
//	// pass the full image in for descriptors since they might use
//	// data from across mask boundaries
//	tbb::parallel_for(size_t(0), size_t(masks.size()), [&](size_t i){
////		extractor->compute(frame, kptsList[i], descList[i]);
//		extractor->compute(frame(masks[i]), kptsList[i], descList[i]);
//	});
//	kpts.clear();
//	for(int i=0; i<kptsList.size(); i++)
//		for(int j=0; j<kptsList[i].size(); j++)
//		{
//			kptsList[i][j].pt = maskOffset[i]+kptsList[i][j].pt;
//			kpts.push_back(kptsList[i][j]);
//		}
//	descs = descList[0];
//	for(int i=1; i<descList.size(); i++)
//		descs.push_back(descList[i]); 

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
//	for(int i=0; i<vCurrentFeatures.size(); i++)
	tbb::parallel_for(size_t(0), size_t(vCurrentFeatures.size()), [&](size_t i)
	{
		//! Reset score.
		vCurrentFeatures[i].ResetScore();

		int32_t match_binsize;

		if(params.half_resolution)
			match_binsize = params.match_binsize/2;
		else
			match_binsize = params.match_binsize;

		//! boundary computation
		const cv::KeyPoint &currentKpt = vCurrentFeatures[i].keypoint;
		int nCenter = currentKpt.pt.x;
		int nTop = currentKpt.pt.y - match_binsize;
		int nBottom = currentKpt.pt.y + match_binsize; // because we'd like to search before +1
		int nLeft = nCenter - match_binsize;
		int nRight = nCenter + match_binsize;

		//! Simple rejection framework
		if (nTop < 0)
			nTop = 0;
		if(nTop < frame.rows && nBottom > 0)
		{
//			if (nTop >= frame.rows)
//				continue;
//			if (nBottom <= 0)
//				continue;

			int start = vKeypointRowLUT[nTop];

			int stop;
			if (nBottom >= frame.rows)
				stop = vPrevFeatures.size();
			else
				stop = vKeypointRowLUT[nBottom];

			//! TODO: Can we make this multi-threaded?
			for(int j=start; j<stop; j++)
			{
				if (vPrevFeatures[j].keypoint.pt.x < nLeft || vPrevFeatures[j].keypoint.pt.x > nRight)
					continue;

				int dx = currentKpt.pt.x - vPrevFeatures[j].keypoint.pt.x;
				int dy = currentKpt.pt.y - vPrevFeatures[j].keypoint.pt.y;

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

				float r = 10000.0;

				if (bUseHammingDistance)
					r = cv::normHamming(vPrevFeatures[j].descriptor.data,vCurrentFeatures[i].descriptor.data,extractor->descriptorSize());
				else
					//! SIFT or SURF
					r = std::sqrt(cv::normL2Sqr_(vPrevFeatures[j].descriptor.ptr<float>(),vCurrentFeatures[i].descriptor.ptr<float>(),extractor->descriptorSize()));

				if (r < vCurrentFeatures[i].scoreFirst)
				{
					vCurrentFeatures[i].scoreSecond = vCurrentFeatures[i].scoreFirst;
					vCurrentFeatures[i].scoreFirst = r;
					vCurrentFeatures[i].bestPos = j; //! set best previous position, initial value -1
				} else if (r > vCurrentFeatures[i].scoreFirst && r < vCurrentFeatures[i].scoreSecond)
				{
					vCurrentFeatures[i].scoreSecond = r;
				}
			} //! end inner for
		}
	} //! end outer for
	);

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
//	for(int i=0; i<vPrevFeatures.size(); i++)
	tbb::parallel_for(size_t(0), size_t(vPrevFeatures.size()), [&](size_t i)
	{
		const cv::KeyPoint &currentKpt = vPrevFeatures[i].keypoint;
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
		if(nTop < frame.rows && nBottom > 0)
		{
//			if (nTop >= frame.rows)
//				continue;
//			if (nBottom <= 0)
//				continue;

			int start = vKeypointRowLUT[nTop];

			int stop;
			if (nBottom >= frame.rows)
				stop = vCurrentFeatures.size();
			else
				stop = vKeypointRowLUT[nBottom];

			/// Current;
			for(int j=start; j<stop; j++)
			{
				if (vCurrentFeatures[j].keypoint.pt.x < nLeft || vCurrentFeatures[j].keypoint.pt.x > nRight)
					continue;

				int dx = currentKpt.pt.x - vCurrentFeatures[j].keypoint.pt.x;
				int dy = currentKpt.pt.y - vCurrentFeatures[j].keypoint.pt.y;

				int32_t match_radius;

				if(params.half_resolution)
					match_radius = params.match_radius/2;
				else
					match_radius = params.match_radius;

				if (dx * dx + dy * dy > match_radius*match_radius)
				{
					continue;
				}

				float r = 10000.0;

				if (bUseHammingDistance)
					r = cv::normHamming(vCurrentFeatures[j].descriptor.data,vPrevFeatures[i].descriptor.data,extractor->descriptorSize());
				else
					r = std::sqrt(cv::normL2Sqr_(vCurrentFeatures[j].descriptor.ptr<float>(),vPrevFeatures[i].descriptor.ptr<float>(),extractor->descriptorSize()));

				if (r < vCurrentFeatures[j].scoreFirst_back)
				{
					vCurrentFeatures[j].scoreSecond_back = vCurrentFeatures[j].scoreFirst_back;
					vCurrentFeatures[j].scoreFirst_back = r;
					vCurrentFeatures[j].bestPos_back = j; // set best next position, initial value -1
				} else if (r > vCurrentFeatures[j].scoreFirst_back && r < vCurrentFeatures[j].scoreSecond_back)
				{
					vCurrentFeatures[j].scoreSecond_back = r;
				}
			} //! end inner for
		}
	} //! end outer for
	);

	//! feature marriage
	vMatches.clear();
	std::set<int> featurePool;

	for(int i=0; i<vCurrentFeatures.size(); i++)
	{
		if ((float) (vCurrentFeatures[i].scoreFirst / vCurrentFeatures[i].scoreSecond) < params.ratio_threshold
						&& vCurrentFeatures[i].bestPos_back == i) /* feature marriage check */
		{
			if(bUseHammingDistance)
			{
				if(vCurrentFeatures[i].scoreFirst > params.minHammingDist)
					continue;
			}

			float factor;

			if(params.half_resolution)
				factor = 2.0;
			else
				factor = 1.0;

			/// Skip duplicated feature.
			if(featurePool.count(vCurrentFeatures[i].bestPos) > 0)
				continue;

			/// p_match preparation
			p_match match;

			/// from current frame.
			match.u1c = vCurrentFeatures[i].keypoint.pt.x*factor;
			match.v1c = vCurrentFeatures[i].keypoint.pt.y*factor;
			match.i1c = i;	/// index number in current frame.

			/// from previous frame.
			match.u1p = vPrevFeatures[vCurrentFeatures[i].bestPos].keypoint.pt.x*factor;
			match.v1p = vPrevFeatures[vCurrentFeatures[i].bestPos].keypoint.pt.y*factor;
			match.i1p = vCurrentFeatures[i].bestPos;	/// index number in previous frame.

			featurePool.insert(vCurrentFeatures[i].bestPos);

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

} // namespace Quadrotor
} // namespace ICSL
