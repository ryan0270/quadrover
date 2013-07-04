#ifndef ICSL_VISIOFINDER_H
#define ICSL_VISIONFINDER_H
#include <memory>
#include <sched.h>
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

		virtual void onFeaturesFound(shared_ptr<ImageFeatureData> const data)=0;
};

class FeatureFinder : public toadlet::egg::Thread, 
						public CommManagerListener,
						public SensorManagerListener
{
	public:
		explicit FeatureFinder();
		virtual ~FeatureFinder(){};

		void shutdown();
		void initialize();
		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

		bool isFirstImageProcessed(){return mFirstImageProcessed;}

		void setVisionParams(toadlet::egg::Collection<int> const &p);
		void setStartTime(Time t){mStartTime = t;}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

		void enableIbvs(bool enable);

		int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImgProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
		int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImgProcTimeUS; mMutex_data.unlock(); return temp;}
		void getLastImage(cv::Mat *outImage);
		void getLastImageAnnotated(cv::Mat *outImage);
		toadlet::egg::Collection<int> getVisionParams();

		void addListener(FeatureFinderListener *listener){mListeners.push_back(listener);}

		// CommManagerListener functions
		void onNewCommLogMask(uint32 mask);
		void onNewCommImgBufferSize(int size);
		void onCommConnectionLost();
		void onNewCommMotorOn(){mMotorOn = true;}
		void onNewCommMotorOff(){mMotorOn = false;}
		void onNewCommLogClear();
		
		// SensorManagerListener
		void onNewSensorUpdate(shared_ptr<IData> const &data);

	protected:
		bool mUseIbvs;
		bool mFirstImageProcessed;
		bool mRunning, mFinished;
		bool mNewImageReady, mNewImageReady_targetFind;
		bool mLogImages;
		cv::Mat	mCurImage, mCurImageGray;
		shared_ptr<cv::Mat> mCurImageAnnotated;
		vector<vector<double> > mMSERHuMoments;

		shared_ptr<DataImage> mImageDataPrev, mImageDataCur, mImageDataNext;

		Time mStartTime, mLastProcessTime;

		toadlet::uint32 mImgProcTimeUS;

		QuadLogger *mQuadLogger;

		toadlet::egg::Mutex mMutex_data, mMutex_image, mMutex_imageData, mMutex_buffers;
		toadlet::egg::Mutex mMutex_logger;

		Collection<FeatureFinderListener*> mListeners;

		void run();

		int mImgBufferMaxSize;

		vector<cv::Point2f> findFeaturePoints(cv::Mat const &img);
		static void drawPoints(vector<cv::Point2f> const &points, cv::Mat &img);

		bool mMotorOn;
		int mThreadPriority, mScheduler;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
