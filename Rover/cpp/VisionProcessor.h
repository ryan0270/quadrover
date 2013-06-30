#ifndef VISIONPROCESSOR_H
#define VISIONPROCESSOR_H
#include <memory>
#include <sched.h>
#include <cmath>
#include <list>

#include <toadlet/egg.h>
//using toadlet::egg::String;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Observer_Angular.h"
#include "Time.h"
#include "CommManager.h"
#include "SensorManager.h"
#include "BlobDetector.h"
#include "Data.h"

#include "Matcher.h"

namespace ICSL {
namespace Quadrotor {
class VisionProcessorListener
{
	public:
		virtual ~VisionProcessorListener(){};

		virtual void onImageProcessed(shared_ptr<ImageMatchData> const data)=0;
		virtual void onImageTargetFound(shared_ptr<ImageTargetFindData> const data)=0;
		virtual void onImageLost()=0;
};

class VisionProcessor : public toadlet::egg::Thread, 
						public CommManagerListener,
						public SensorManagerListener
{
	public:
		explicit VisionProcessor();
		virtual ~VisionProcessor(){};

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

		void addListener(VisionProcessorListener *listener){mListeners.push_back(listener);}

		// CommManagerListener functions
		void onNewCommLogMask(uint32 mask);
		void onNewCommImgBufferSize(int size);
		void onNewCommVisionRatioThreshold(float h);
		void onNewCommVisionMatchRadius(float r);
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
		bool mNewImageReady_featureMatch, mNewImageReady_targetFind;
		bool mLogImages;
		cv::Mat	mCurImage, mCurImageGray;
		shared_ptr<cv::Mat> mCurImageAnnotated;
		vector<vector<double> > mMSERHuMoments;
		vector<cv::Point2f> mMSERCentroids;

		shared_ptr<DataImage> mImageDataPrev, mImageDataCur, mImageDataNext;

		Time mStartTime, mLastImgFoundTime, mLastProcessTime;

		toadlet::uint32 mImgProcTimeUS;

		QuadLogger *mQuadLogger;

		toadlet::egg::Mutex mMutex_data, mMutex_image, mMutex_imageData, mMutex_buffers, mMutex_matcher;
		toadlet::egg::Mutex mMutex_logger;

		Collection<VisionProcessorListener*> mListeners;

		void run();

		int mImgBufferMaxSize;
		list<shared_ptr<DataImage> > mImgDataBuffer;
		list<shared_ptr<ImageMatchData> > mImgMatchDataBuffer;
		list<vector<vector<cv::Point2f> > > mFeatureMatchBuffer;
		list<Time> mFeatureMatchTimeBuffer;
		list<double> mFeatureMatchDTBuffer;
		list<TNT::Array2D<double> > mFeatureMatchAttPrevBuffer, mFeatureMatchAttCurBuffer;

		Matcher mFeatureMatcher;

		vector<vector<cv::Point2f> > getMatchingPoints(cv::Mat const &img);
		vector<BlobDetector::Blob> findCircles(cv::Mat const &img);
		static void drawMatches(vector<vector<cv::Point2f> > const &points, cv::Mat &img);
		static void drawTarget(vector<BlobDetector::Blob> const &circles, cv::Mat &img);

		void runTargetFinder();

		bool mMotorOn;
		int mThreadPriority, mScheduler;

		vector<BlobDetector::Blob> mLastCircles;
};

} // namespace Quadrotor
} // namespace ICSL

#endif // VISIONPROCESSOR_H
