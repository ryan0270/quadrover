#ifndef ICSL_REGIONFINDER_H
#define ICSL_REGIONFINDER_H
#include <memory>
#include <sched.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <toadlet/egg.h>

#include "constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Time.h"
#include "Data.h"
#include "Listeners.h"

namespace ICSL {
namespace Quadrotor {
class RegionFinder : public CommManagerListener,
						public SensorManagerListener
{
	public:
	explicit RegionFinder();
	virtual ~RegionFinder(){};

	void shutdown();
	void start(){ thread th(&RegionFinder::run, this); th.detach(); }
	void initialize();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

	int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImageProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
	int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImageProcTimeUS; mMutex_data.unlock(); return temp;}
	void getLastImage(cv::Mat *outImage);
	void getLastImageAnnotated(cv::Mat *outImage);

	static vector<vector<cv::Point2f>> findRegions(const cv::Mat &image);
	static void drawRegions(cv::Mat &image,
							const vector<vector<cv::Point2f>> &regions,
							const vector<cv::Point2f> &centroids);

	void addListener(RegionFinderListener *listener){mListeners.push_back(listener);}

	// CommManagerListener functions
	void onNewCommMotorOn(){mIsMotorOn = true;};
	void onNewCommMotorOff(){mIsMotorOn = false;};
	
	// SensorManagerListener
	void onNewSensorUpdate(const shared_ptr<IData> &data);

	protected:
	bool mUseIbvs;
	bool mRunning, mFinished;
	bool mNewImageReady;
	bool mHaveUpdatedSettings;
	bool mIsMotorOn;

	shared_ptr<DataImage> mImageDataNext;
	shared_ptr<DataAnnotatedImage> mImageAnnotatedLast;

	Time mStartTime, mLastProcessTime;

	uint32_t mImageProcTimeUS;

	QuadLogger *mQuadLogger;

	std::mutex mMutex_data, mMutex_image, mMutex_imageData, mMutex_buffers;
	std::mutex mMutex_logger;
	std::mutex mMutex_params;

	toadlet::egg::Collection<RegionFinderListener*> mListeners;

	float mQualityLevel, mFASTThreshold, mFASTAdaptRate;
	int mSepDist, mPointCntTarget;

	void run();

	int mThreadPriority, mScheduler;

	static vector<vector<cv::Point>> getContours(const cv::Mat &image, const vector<vector<cv::Point>> &regions);
};

} // namespace Quadrotor
} // namespace ICSL

#endif
