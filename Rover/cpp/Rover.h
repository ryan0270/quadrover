#ifndef ROVER_H
#define ROVER_H
#include <memory>
#include <fstream>
#include <sstream>

#include <cpu-features.h>

#include <toadlet/egg.h>
//using toadlet::int64;
//using toadlet::uint64;
//using toadlet::egg::String;

#include <opencv2/core/core.hpp>

#include "Observer_Translational.h"
#include "TNT/tnt.h"
#include "TNT_Utils.h"
#include "constants.h"
#include "Observer_Angular.h"
#include "QuadLogger.h"
#include "CommManager.h"
#include "VisionProcessor.h"
#include "Common.h"
#include "Time.h"
#include "TranslationController.h"
#include "AttitudeThrustController.h"
#include "SensorManager.h"
#include "VideoMaker.h"
#include "MotorInterface.h"
#include "Data.h"

namespace ICSL {
namespace Quadrotor {
class Rover: public Observer_AngularListener,
				 public CommManagerListener,
				 public SensorManagerListener,
				 public VisionProcessorListener,
				 public toadlet::egg::Thread
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

	void run();
	
	// these functions are primarily for the jni interface
	void copyImageData(cv::Mat *m);
	TNT::Array2D<double> getGyroValue();
	TNT::Array2D<double> getAccelValue();
	TNT::Array2D<double> getMagValue();
	TNT::Array2D<double> getAttitude();
	int getImageProcTimeMS(){mMutex_vision.lock(); int temp = mVisionProcessor.getImageProcTimeMS(); mMutex_vision.unlock(); return temp;}
//	void toggleViewType(){Log::alert("Not implemented");}
//	void toggleUseIbvs(){onNewCommUseIbvs(!mUseIbvs);}
//	toadlet::egg::Collection<int> getVisionParams();
//	void setVisionParams(toadlet::egg::Collection<int> p);
	bool pcIsConnected(){return mCommManager.pcIsConnected();}

	// Observer_AngularListener
	void onObserver_AngularUpdated(shared_ptr<DataVector> attData, shared_ptr<DataVector> angularVelData);

	// for CommManagerListener
	void onNewCommTimeSync(int time);
	void onNewCommLogTransfer();
	void onNewCommLogMask(uint32 mask);
	void onNewCommLogClear();

	// for SensorManagerListener
	void onNewSensorUpdate(shared_ptr<Data> const &data);

	// for VisionProcessorListener
	void onImageProcessed(shared_ptr<ImageMatchData> const data);
	void onImageTargetFound(shared_ptr<ImageTargetFindData> const data){};
	void onImageLost(){};

protected:
	CommManager mCommManager;
	bool mRunCommPC, mRunnerIsDone;
	bool mDataIsSending, mImageIsSending;

	TranslationController mTranslationController;
	AttitudeThrustController mAttitudeThrustController;

	Observer_Angular mObsvAngular;
	Observer_Translational mObsvTranslational;
	TNT::Array2D<double> mCurAtt, mCurAngularVel;

	Time mStartTime, mLastDataSendTime, mLastImageSendTime;

	TNT::Array2D<double> mRotViconToQuad, mRotQuadToPhone, mRotCamToPhone, mRotPhoneToCam, mRotViconToPhone;

	Mutex mMutex_cntl, mMutex_observer, mMutex_vision, mMutex_vicon, mMutex_data;
	
	void transmitDataUDP();
	void transmitImage();

	QuadLogger mQuadLogger;

	VisionProcessor mVisionProcessor;

	SensorManager mSensorManager;

	bool mUseIbvs;

	static TNT::Array2D<int> getCpuUsage(int numCpuCores);

	int mNumCpuCores;

	double mPressure, mPhoneTemp;

	shared_ptr<ImageMatchData> mImageMatchData;

	VideoMaker mVideoMaker;
	
	MotorInterface mMotorInterface;
}; // class Rover

} // namespace Quadrotor
} // namespace ICSL

#endif
