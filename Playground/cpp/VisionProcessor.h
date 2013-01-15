#ifndef VISIONPROCESSOR_H
#define VISIONPROCESSOR_H
#include <sched.h>

#include <toadlet/egg.h>
using toadlet::egg::String;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "TNT/tnt.h"

#include "QuadLogger.h"
#include "Common.h"
#include "Time.h"

namespace ICSL {
namespace Quadrotor {
class ImageGrabber : public toadlet::egg::Thread
{
	public:
		explicit ImageGrabber();
		virtual ~ImageGrabber(){};

		void copyImage(cv::Mat *dstImage);

		void shutdown();

		bool isNewImageReady(){return  mNewImageReady;}

		void markBottleneck(bool isBottleneck){mIsBottleneck = isBottleneck;}

		TNT::Array2D<double> getImageAtt();
		TNT::Array2D<double> getRotVel();

		void setStartTime(Time t){mStartTime = t;}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

	protected:
		bool mNewImageReady, mIsBottleneck;
		bool mRunning, mFinished;
		cv::Mat mLastImage;
		toadlet::egg::Mutex mMutex_image;

		Time mStartTime;
		QuadLogger *mQuadLogger;

		void run();
};

class VisionProcessorListener
{
	public:
		virtual ~VisionProcessorListener(){};

		virtual void onImageProcessed(toadlet::egg::Collection<cv::Point2f> const &boxCenters, toadlet::egg::Collection<bool> const &boxFound, TNT::Array2D<double> const &attitude, TNT::Array2D<double> const &rotVel)=0;
		virtual void onImageLost()=0;
};

class VisionProcessor : public toadlet::egg::Thread
{
	public:
		explicit VisionProcessor();
		virtual ~VisionProcessor(){};

		void shutdown();

		void processImage();
		bool isFirstImageProcessed(){return mFirstImageProcessed;}

		void setStartTime(Time t){mStartTime = t; mImageGrabber.setStartTime(t);}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log; mImageGrabber.setQuadLogger(log);}

		void enableIbvs(bool enable);

		int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImgProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
		int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImgProcTimeUS; mMutex_data.unlock(); return temp;}
		void getLastImage(cv::Mat *outImage);

		void addListener(VisionProcessorListener *listener){mListeners.push_back(listener);}

	protected:
		bool mUseIbvs;
		bool mFirstImageProcessed;
		bool mRunning, mFinished;
		cv::Mat	mLastImage;
		Time mStartTime, mLastImgFoundTime;

		toadlet::uint64 mImgProcTimeUS;

		ImageGrabber mImageGrabber;

		QuadLogger *mQuadLogger;

		toadlet::egg::Mutex mMutex_data, mMutex_image;

		Collection<VisionProcessorListener*> mListeners;

		void run();
};

} // namespace Quadrotor
} // namespace ICSL

#endif
