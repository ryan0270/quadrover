#ifndef VISIONPROCESSOR_H
#define VISIONPROCESSOR_H
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

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "ICSL/constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Observer_Angular.h"
#include "Time.h"
#include "CommManager.h"
#include "SensorManager.h"

namespace ICSL {
namespace Quadrotor {

class VisionProcessorListener
{
	public:
		virtual ~VisionProcessorListener(){};

		virtual void onImageProcessed(toadlet::egg::Collection<cv::Point2f> const &boxCenters, toadlet::egg::Collection<bool> const &boxFound, TNT::Array2D<double> const &attitude, TNT::Array2D<double> const &rotVel)=0;
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

		vector<vector<cv::Point2f> > getMatchingPoints();
		bool isFirstImageProcessed(){return mFirstImageProcessed;}

		void setVisionParams(toadlet::egg::Collection<int> const &p);
		void setStartTime(Time t){mStartTime = t;}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

		void enableIbvs(bool enable);

		int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImgProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
		int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImgProcTimeUS; mMutex_data.unlock(); return temp;}
		void getLastImage(cv::Mat *outImage);
		toadlet::egg::Collection<int> getVisionParams();

		void addListener(VisionProcessorListener *listener){mListeners.push_back(listener);}

		// this will eventually move to some listener
		void calcOpticalFlow(vector<vector<cv::Point2f> > const &points);

		// CommManagerListener functions
		void onNewCommLogMask(uint32 mask);
		void onNewCommImgBufferSize(int size);
		
		// SensorManagerListener
		void onNewSensorUpdate(SensorData const *data);

	protected:
		bool mUseIbvs;
		bool mFirstImageProcessed;
		bool mRunning, mFinished;
		bool mNewImageReady;
		bool mLogImages;
		cv::Mat	mCurImage, mCurImageGray;
		cv::Mat mLastImageGray;
		vector<vector<double> > mMSERHuMoments;
		vector<cv::Point2f> mMSERCentroids;

		double mFocalLength;

		SensorDataImage mImageData;

		Time mStartTime, mLastImgFoundTime, mLastProcessTime;

		toadlet::uint32 mImgProcTimeUS;

		QuadLogger *mQuadLogger;

		toadlet::egg::Mutex mMutex_data, mMutex_image, mMutex_imageSensorData, mMutex_imgBuffer;

		Collection<VisionProcessorListener*> mListeners;

		void run();

		int mImgBufferMaxSize;
		list<cv::Mat> mImgBuffer;
		list<SensorDataImage> mImgDataBuffer;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
