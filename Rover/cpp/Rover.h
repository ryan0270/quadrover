#ifndef ROVER_H
#define ROVER_H
#include <memory>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>

#include <cpu-features.h>

#include <opencv2/core/core.hpp>

#include <toadlet/egg.h>

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "constants.h"
#include "Common.h"
#include "Data.h"
#include "Observer_Angular.h"
#include "Observer_Translational.h"
#include "QuadLogger.h"
#include "CommManager.h"
#include "Time.h"
#include "TranslationController.h"
#include "AttitudeThrustController.h"
#include "SensorManager.h"
#include "VideoMaker.h"
#include "MotorInterface.h"
#include "FeatureFinder.h"
//#include "TargetFinder.h"
#include "TargetFinder2.h"
#include "VelocityEstimator.h"
#include "Listeners.h"

namespace ICSL {
namespace Quadrotor {
class Rover: public Observer_AngularListener,
				 public CommManagerListener,
				 public SensorManagerListener,
				 public FeatureFinderListener,
				 public TargetFinder2Listener
{
public:
	Rover();
	virtual ~Rover();

	void initialize();
	void shutdown();

	void setLogFilename(String name);
	void setLogDir(String dir);
	void startLogging();
	void stopLogging();

	void start(){ thread th(&Rover::run, this); th.detach(); }
	void run();

	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
	
	// these functions are primarily for the jni interface
	void copyImageData(cv::Mat *m);
	TNT::Array2D<double> getGyroValue();
	TNT::Array2D<double> getAccelValue();
	TNT::Array2D<double> getMagValue();
	TNT::Array2D<double> getAttitude();
	int getImageProcTimeMS(){mMutex_vision.lock(); int temp = mVelocityEstimator.getLastVisionDelayTimeUS()/1.0e3; mMutex_vision.unlock(); return temp;}
//	void toggleViewType(){Log::alert("Not implemented");}
//	void toggleUseIbvs(){onNewCommUseIbvs(!mUseIbvs);}
//	toadlet::egg::Collection<int> getVisionParams();
//	void setVisionParams(toadlet::egg::Collection<int> p);
	bool pcIsConnected(){return mCommManager.pcIsConnected();}
	void passNewImage(cv::Mat *img, int64 const &timestampNS){mSensorManager.passNewImage(img, timestampNS);}
	vector<uint16> getMotorCmds() {return mMotorInterface.getMotorCmds();}
	void onNewSonarReading(int heightMM, uint64 timestampNS){mSensorManager.onNewSonarReading(heightMM, timestampNS);}

	// Observer_AngularListener
	void onObserver_AngularUpdated(const shared_ptr<SO3Data<double>> &attData, const shared_ptr<DataVector<double>> &angularVelData);

	// for CommManagerListener
	void onNewCommTimeSync(int time);
	void onNewCommLogTransfer();
	void onNewCommLogMask(uint32 mask);
	void onNewCommLogClear();

	// for SensorManagerListener
	void onNewSensorUpdate(shared_ptr<IData> const &data);

	// for FeatureFinderListener
	void onFeaturesFound(shared_ptr<ImageFeatureData> const &data);

	// for TargetFinderListener
//	void onTargetFound(shared_ptr<ImageTargetFindData> const &data);

	// for TargetFinder2Listener
	void onTargetFound2(shared_ptr<ImageTargetFind2Data> const &data);

protected:
	CommManager mCommManager;
	bool mRunning, mRunnerIsDone;
	bool mDataIsSending, mImageIsSending;

	TranslationController mTranslationController;
	AttitudeThrustController mAttitudeThrustController;

	Observer_Angular mObsvAngular;
	Observer_Translational mObsvTranslational;
	TNT::Array2D<double> mCurAtt, mCurAngularVel;

	Time mStartTime, mLastDataSendTime, mLastImageSendTime;

	TNT::Array2D<double> mRotViconToQuad, mRotQuadToPhone, mRotCamToPhone, mRotPhoneToCam, mRotViconToPhone;

	std::mutex mMutex_cntl, mMutex_observer, mMutex_vision, mMutex_vicon, mMutex_data;
	
	void transmitDataUDP();
	void transmitImage();

	QuadLogger mQuadLogger;

	VelocityEstimator mVelocityEstimator;
	FeatureFinder mFeatureFinder;
//	TargetFinder mTargetFinder;
	TargetFinder2 mTargetFinder2;

	SensorManager mSensorManager;

	bool mUseIbvs;

	static TNT::Array2D<int> getCpuUsage(int numCpuCores);

	int mNumCpuCores;

	double mPressure, mPhoneTemp;

	shared_ptr<DataImage> mImageData;
//	shared_ptr<ImageMatchData> mImageMatchData;
	shared_ptr<ImageFeatureData> mFeatureData;
//	shared_ptr<ImageTargetFindData> mTargetData;
	shared_ptr<ImageTargetFind2Data> mTargetData2;

	VideoMaker mVideoMaker;
	
	MotorInterface mMotorInterface;

	int mThreadPriority, mScheduler;

	int getCpuFreq();
}; // class Rover

} // namespace Quadrotor
} // namespace ICSL

#endif
