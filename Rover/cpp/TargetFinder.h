#ifndef ICSL_TARGETFINDER_H
#define ICSL_TARGETFINDER_H
#include <memory>
#include <sched.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <toadlet/egg.h>

//#include "TNT/tnt.h"
//#include "TNT_Utils.h"

#include "constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Observer_Angular.h"
//#include "Observer_Translational.h"
#include "Time.h"
#include "CommManager.h"
#include "SensorManager.h"
#include "Data.h"

namespace ICSL {
namespace Quadrotor {
class TargetFinderListener
{
	public:
		virtual ~TargetFinderListener(){};

		virtual void onTargetFound(shared_ptr<ImageTargetFindData> const &data)=0;
};

class TargetFinder : public CommManagerListener,
						public SensorManagerListener
{
	public:
		explicit TargetFinder();
		virtual ~TargetFinder(){};

		void shutdown();
		void start(){ thread th(&TargetFinder::run, this); th.detach(); }
		void initialize();
		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

		bool isFirstImageProcessed(){return mFirstImageProcessed;}

		void setStartTime(Time t){mStartTime = t;}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

//		void enableIbvs(bool enable);

		int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImageProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
		int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImageProcTimeUS; mMutex_data.unlock(); return temp;}
		void getLastImage(cv::Mat *outImage);
		toadlet::egg::Collection<int> getVisionParams();

		void addListener(TargetFinderListener *listener){mListeners.push_back(listener);}

		// CommManagerListener functions
		void onNewCommMotorOn(){mIsMotorOn = true;}
		void onNewCommMotorOff(){mIsMotorOn = false;}
		
		// SensorManagerListener
		void onNewSensorUpdate(shared_ptr<IData> const &data);

	protected:
		bool mUseIbvs;
		bool mFirstImageProcessed;
		bool mRunning, mFinished;
		bool mNewImageReady, mNewImageReady_targetFind;
		bool mLogImages;
		bool mHaveUpdatedSettings;
		bool mIsMotorOn;
//		cv::Mat	mCurImage, mCurImageGray;
//		shared_ptr<cv::Mat> mCurImageAnnotated;

//		shared_ptr<DataImage> mImageDataPrev, mImageDataCur, mImageDataNext;
		shared_ptr<DataImage> mImageDataNext;
		shared_ptr<DataAnnotatedImage> mImageAnnotatedLast;

		Time mStartTime;//, mLastProcessTime;

		toadlet::uint32 mImageProcTimeUS;

		QuadLogger *mQuadLogger;

		std::mutex mMutex_data, mMutex_image, mMutex_imageData, mMutex_buffers;
		std::mutex mMutex_logger;
		std::mutex mMutex_params;

		toadlet::egg::Collection<TargetFinderListener*> mListeners;

//		float mQualityLevel, mFASTThreshold, mFASTAdaptRate;
//		int mSepDist, mPointCntTarget;

		void run();

		int mThreadPriority, mScheduler;

//		shared_ptr<RectGroup> findTarget(cv::Mat &image);
		shared_ptr<RectGroup> findTarget(cv::Mat &image, cv::Mat const &cameraMatrix, cv::Mat const &distCoeffs);
		static void drawTarget(cv::Mat &img, shared_ptr<RectGroup> const &target);

		// helper function:
		// finds a cosine of angle between vectors
		// from pt0->pt1 and from pt0->pt2
		static double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 )
		{
			double dx1 = pt1.x - pt0.x;
			double dy1 = pt1.y - pt0.y;
			double dx2 = pt2.x - pt0.x;
			double dy2 = pt2.y - pt0.y;
			return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
		}
};

} // namespace Quadrotor
} // namespace ICSL

#endif
