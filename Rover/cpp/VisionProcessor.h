#ifndef VISIONPROCESSOR_H
#define VISIONPROCESSOR_H
#include <memory>
#include <sched.h>
#include <math.h>
#include <list>

#include <toadlet/egg.h>
using toadlet::egg::String;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "ferns/mcv.h"
//#include "ferns/template_matching_based_tracker.h"

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "ICSL/constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Observer_Angular.h"
#include "Time.h"
#include "CommManager.h"
#include "SensorManager.h"
#include "Matcher.h"

namespace ICSL {
namespace Quadrotor {

class ImageMatchData
{
	public:
	vector<vector<cv::Point2f> > featurePoints;
	shared_ptr<SensorDataImage> imgData0, imgData1;
	double dt;

	void lock(){mMutex.lock(); if(imgData0 != NULL) imgData0->lock(); if(imgData1 != NULL) imgData1->lock();}
	void unlock(){mMutex.unlock(); if(imgData0 != NULL) imgData0->unlock(); if(imgData1 != NULL) imgData1->unlock();}

	protected:
	toadlet::egg::Mutex mMutex;
};

class VisionProcessorListener
{
	public:
		virtual ~VisionProcessorListener(){};

		virtual void onImageProcessed(shared_ptr<ImageMatchData> const data)=0;
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
		
		// SensorManagerListener
		void onNewSensorUpdate(shared_ptr<SensorData> const &data);

	protected:
		bool mUseIbvs;
		bool mFirstImageProcessed;
		bool mRunning, mFinished;
		bool mNewImageReady;
		bool mLogImages;
		cv::Mat	mCurImage, mCurImageGray, mCurImageAnnotated;
		vector<vector<double> > mMSERHuMoments;
		vector<cv::Point2f> mMSERCentroids;

		shared_ptr<SensorDataImage> mImageDataPrev, mImageDataCur, mImageDataNext;

		Time mStartTime, mLastImgFoundTime, mLastProcessTime;

		toadlet::uint32 mImgProcTimeUS;

		QuadLogger *mQuadLogger;

		toadlet::egg::Mutex mMutex_data, mMutex_image, mMutex_imageSensorData, mMutex_imgBuffer, mMutex_matcher;

		Collection<VisionProcessorListener*> mListeners;

		void run();

		int mImgBufferMaxSize;
		list<shared_ptr<SensorDataImage> > mImgDataBuffer;

		Matcher mFeatureMatcher;

		vector<vector<cv::Point2f> > getMatchingPoints(cv::Mat const &img);
		static void drawMatches(vector<vector<cv::Point2f> > const &points, cv::Mat &img);
};

} // namespace Quadrotor
} // namespace ICSL

#endif
