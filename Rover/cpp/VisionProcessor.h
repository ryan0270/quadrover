#ifndef VISIONPROCESSOR_H
#define VISIONPROCESSOR_H
#include <sched.h>
#include <math.h>

#include <toadlet/egg.h>
using toadlet::egg::String;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "ICSL/constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Observer_Angular.h"
#include "Time.h"
#include "CommManager.h"

namespace ICSL {
namespace Quadrotor {
class ImageGrabber : public toadlet::egg::Thread
{
	public:
		explicit ImageGrabber();
		virtual ~ImageGrabber(){};

		void copyImage(cv::Mat *dstImage);
		void copyImageHSV(cv::Mat *dstImage);
		void copyImageGray(cv::Mat *dstImage);

		void shutdown();

		bool isNewImageReady(){return  mNewImageReady;}

		void markBottleneck(bool isBottleneck){mIsBottleneck = isBottleneck;}

		bool imageConversionDone(){return mImgConversionDone;}

		TNT::Array2D<double> getImageAtt();
		TNT::Array2D<double> getRotVel();

		void setAttitudeObserver(Observer_Angular *obs){mAttObserver = obs;}

		void setStartTime(Time t){mStartTime = t;}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

	protected:
		bool mNewImageReady, mIsBottleneck;
		bool mRunning, mFinished;
		bool mImgConversionDone;
		cv::Mat mCurImage, mCurImageHSV, mCurImageGray;
		toadlet::egg::Mutex mMutex_image, mMutex_data;
		TNT::Array2D<double> mImgAtt, mImgRotVel;

		Observer_Angular *mAttObserver;

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


class VisionProcessor : public toadlet::egg::Thread, public CommManagerListener
{
	public:
		explicit VisionProcessor();
		virtual ~VisionProcessor(){};

		void shutdown();

		void processImage(TNT::Array2D<double> const &imgAtt, TNT::Array2D<double> const &rotVel);
		bool isFirstImageProcessed(){return mFirstImageProcessed;}

		void setBoxColorCenters(toadlet::egg::Collection<int> const &data);
		void setBoxColorHalfWidth(Collection<int> const &data);
		void setVisionParams(toadlet::egg::Collection<int> const &p);
		void setSatMin(int val);
		void setSatMax(int val);
		void setValMin(int val);
		void setValMax(int val);
		void setCircMin(int val);
		void setCircMax(int val);
		void setConvMin(int val);
		void setConvMax(int val);
		void setAreaMin(int val);
		void setAreaMax(int val);
		void setViewType(int val);
		void setStartTime(Time t){mStartTime = t; mImageGrabber.setStartTime(t);}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log; mImageGrabber.setQuadLogger(log);}

		void setAttitudeObserver(Observer_Angular *obs){mImageGrabber.setAttitudeObserver(obs);};

		void enableIbvs(bool enable);

		int getImgViewType(){return mImgViewType;}
		int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImgProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
		int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImgProcTimeUS; mMutex_data.unlock(); return temp;}
		void getLastImage(cv::Mat *outImage);
		toadlet::egg::Collection<int> getVisionParams();

		void addListener(VisionProcessorListener *listener){mListeners.push_back(listener);}

		// CommManagerListener functions
		void onNewCommImgProcBoxColorCenter(toadlet::egg::Collection<int> const &data);
		void onNewCommImgProcBoxColorHalfRange(toadlet::egg::Collection<int> const &data);
		void onNewCommImgProcSatMin(int val);
		void onNewCommImgProcSatMax(int val);
		void onNewCommImgProcValMin(int val);
		void onNewCommImgProcValMax(int val);
		void onNewCommImgProcCircMin(int val);
		void onNewCommImgProcCircMax(int val);
		void onNewCommImgProcConvMin(int val);
		void onNewCommImgProcConvMax(int val);
		void onNewCommImgProcAreaMin(int val);
		void onNewCommImgProcAreaMax(int val);
		void onNewCommImgViewType(int val);

	protected:
		bool mUseIbvs;
		bool mFirstImageProcessed;
		bool mRunning, mFinished;
		cv::Mat	mCurImage, mCurImageGray, mTempSum, mTempColor;
		cv::Mat mLastImageGray;
		cv::Mat mChanH, mChanS, mChanV, mTempS, mTempV;
		toadlet::egg::Collection<cv::Point2f> mBoxCenters;
		vector<cv::KeyPoint> mTempKeyPoints;
//		toadlet::egg::Collection<int> mBoxAreas;
		toadlet::egg::Collection<int> mFiltBoxColorCenter, mFiltBoxColorCenterActive, mFiltBoxColorHalfWidth; // the active variable to for online adaptiation, but I still want to remember the original
		int mFiltValMin, mFiltValMax;
		int mFiltSatMin, mFiltSatMax;
		int mFiltCircMin, mFiltCircMax;
		int mFiltConvMin, mFiltConvMax;
		int mFiltAreaMin, mFiltAreaMax;
		int mImgViewType;
//		TNT::Array2D<double> mImgMoment, mDesImgMoment, mDesLinearVel;
		Time mStartTime, mLastImgFoundTime, mLastProcessTime;

		toadlet::uint32 mImgProcTimeUS;

		ImageGrabber mImageGrabber;

		QuadLogger *mQuadLogger;

		toadlet::egg::Mutex mMutex_data, mMutex_image;

		Collection<VisionProcessorListener*> mListeners;

		void run();
//		static bool compareBlobsBySize(cv::KeyPoint p1, cv::KeyPoint p2)
//			{return (p1.size > p2.size);}
};

} // namespace Quadrotor
} // namespace ICSL

#endif
