#ifndef ICSL_VISIOFINDER_H
#define ICSL_VISIONFINDER_H
#include <memory>
#include <sched.h>
#include <thread>
#include <cmath>
#include <list>

#include <toadlet/egg.h>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/video/tracking.hpp>
//#include <opencv2/objdetect/objdetect.hpp>

//#include "TNT/tnt.h"
//#include "TNT_Utils.h"

#include "constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Observer_Angular.h"
#include "Time.h"
#include "CommManager.h"
#include "SensorManager.h"
#include "Data.h"

namespace ICSL {
namespace Quadrotor {
class FeatureFinderListener
{
	public:
		virtual ~FeatureFinderListener(){};

		virtual void onFeaturesFound(shared_ptr<ImageFeatureData> const &data)=0;
};

class FeatureFinder : public CommManagerListener,
						public SensorManagerListener
{
	public:
		explicit FeatureFinder();
		virtual ~FeatureFinder(){};

		void shutdown();
		void start(){ thread th(&FeatureFinder::run, this); th.detach(); }
		void initialize();
		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

		bool isFirstImageProcessed(){return mFirstImageProcessed;}

		void setVisionParams(toadlet::egg::Collection<int> const &p);
		void setStartTime(Time t){mStartTime = t;}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

		void enableIbvs(bool enable);

		int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImageProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
		int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImageProcTimeUS; mMutex_data.unlock(); return temp;}
		void getLastImage(cv::Mat *outImage);
		void getLastImageAnnotated(cv::Mat *outImage);
		toadlet::egg::Collection<int> getVisionParams();

		void addListener(FeatureFinderListener *listener){mListeners.push_back(listener);}

		// CommManagerListener functions
		void onNewCommVisionFeatureFindQualityLevel(float const &qLevel);
		void onNewCommVisionFeatureFindSeparationDistance(int const &sepDist);
		void onNewCommVisionFeatureFindFASTThreshold(int const &thresh);
		void onNewCommVisionFeatureFindPointCntTarget(int const &target);
		void onNewCommVisionFeatureFindFASTAdaptRate(float const &r);
		
		// SensorManagerListener
		void onNewSensorUpdate(shared_ptr<IData> const &data);

	protected:
		bool mUseIbvs;
		bool mFirstImageProcessed;
		bool mRunning, mFinished;
		bool mNewImageReady, mNewImageReady_targetFind;
		bool mLogImages;
		bool mHaveUpdatedSettings;
//		cv::Mat	mCurImage, mCurImageGray;
		shared_ptr<cv::Mat> mCurImageAnnotated;

//		shared_ptr<DataImage> mImageDataPrev, mImageDataCur, mImageDataNext;
		shared_ptr<DataImage> mImageDataNext;
		shared_ptr<DataAnnotatedImage> mImageAnnotatedLast;

		Time mStartTime, mLastProcessTime;

		toadlet::uint32 mImageProcTimeUS;

		QuadLogger *mQuadLogger;

		toadlet::egg::Mutex mMutex_data, mMutex_image, mMutex_imageData, mMutex_buffers;
		toadlet::egg::Mutex mMutex_logger;
		toadlet::egg::Mutex mMutex_params;

		Collection<FeatureFinderListener*> mListeners;

		float mQualityLevel, mFASTThreshold, mFASTAdaptRate;
		int mSepDist, mPointCntTarget;

		void run();


		int mThreadPriority, mScheduler;

		static vector<cv::Point2f> findFeaturePoints(cv::Mat const &image, 
															 double const &qualityLevel,
															 double const &minDistance,
															 int const &fastThreshold);
		static void eigenValResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize);
		static void drawPoints(vector<cv::Point2f> const &points, cv::Mat &img);

};

} // namespace Quadrotor
} // namespace ICSL

#endif
