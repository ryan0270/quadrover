#ifndef ICSL_TARGETFINDER2_H
#define ICSL_TARGETFINDER2_H
#include <memory>
#include <sched.h>
#include <thread>
#include <mutex>
#include <cmath>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <toadlet/egg.h>

#include "constants.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Time.h"
//#include "Data.h"
#include "ActiveRegion.h"
#include "Listeners.h"
#include "Observer_Angular.h"
#include "Observer_Translational.h"

namespace ICSL {
namespace Quadrotor {
class TargetFinder2 : public CommManagerListener,
						public SensorManagerListener
{
public:
	explicit TargetFinder2();
	virtual ~TargetFinder2(){};

	void shutdown();
	void start(){ thread th(&TargetFinder2::run, this); th.detach(); }
	void initialize();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void setObserverAngular(Observer_Angular *obsv){mObsvAngular = obsv;}
	void setObserverTranslational(Observer_Translational *obsv){mObsvTranslational = obsv;}

	bool isFirstImageProcessed(){return mFirstImageProcessed;}

	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

	int getImageProcTimeMS(){mMutex_data.lock(); int temp = mImageProcTimeUS/1000.0; mMutex_data.unlock(); return temp;}
	int getImageProcTimeUS(){mMutex_data.lock(); int temp = mImageProcTimeUS; mMutex_data.unlock(); return temp;}
	void getLastImage(cv::Mat *outImage);
	toadlet::egg::Collection<int> getVisionParams();

	void addListener(TargetFinder2Listener *listener){mListeners.push_back(listener);}

	// CommManagerListener functions
	void onNewCommMotorOn(){mIsMotorOn = true;}
	void onNewCommMotorOff(){mIsMotorOn = false;}
	
	// SensorManagerListener
	void onNewSensorUpdate(const shared_ptr<IData> &data);

protected:
	bool mUseIbvs;
	bool mFirstImageProcessed;
	bool mRunning, mFinished;
	bool mNewImageReady, mNewImageReady_targetFind;
	bool mLogImages;
	bool mHaveUpdatedSettings;
	bool mIsMotorOn;

	shared_ptr<DataImage> mImageDataNext;
	shared_ptr<DataAnnotatedImage> mImageAnnotatedLast;

	Time mStartTime;//, mLastProcessTime;

	toadlet::uint32 mImageProcTimeUS;

	QuadLogger *mQuadLogger;

	std::mutex mMutex_data, mMutex_image, mMutex_imageData, mMutex_buffers;
	std::mutex mMutex_logger;
	std::mutex mMutex_params;

	toadlet::egg::Collection<TargetFinder2Listener*> mListeners;

	void run();

	int mThreadPriority, mScheduler;

	static void drawTarget(cv::Mat &image,
						   const vector<shared_ptr<ActiveRegion>> &curRegions,
						   const vector<shared_ptr<ActiveRegion>> &repeatObjets);

	// for new targetfinder
	Observer_Angular *mObsvAngular;
	Observer_Translational *mObsvTranslational;
	vector<shared_ptr<ActiveRegion>> mActiveRegions;
	vector<vector<cv::Point>> findContours(const cv::Mat &image);

	vector<shared_ptr<ActiveRegion>> objectify(const vector<vector<cv::Point>> &contours,
			const TNT::Array2D<double> Sn,
			const TNT::Array2D<double> SnInv,
			double varxi, double probNoCorr,
			const Time &imageTime);

	void matchify(const vector<shared_ptr<ActiveRegion>> &curRegions,
			vector<RegionMatch> &goodMatches,
			vector<shared_ptr<ActiveRegion>> &repeatRegions,
			const TNT::Array2D<double> Sn,
			const TNT::Array2D<double> SnInv,
			double varxi, double probNoCorr,
			const Time &imageTime);
};

} // namespace Quadrotor
} // namespace ICSL

#endif
//#endif
