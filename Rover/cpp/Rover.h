#ifndef ROVER_H
#define ROVER_H
#include <fstream>
#include <sstream>

#include <toadlet/egg.h>
using toadlet::int64;
using toadlet::uint64;
using toadlet::egg::String;

#include <opencv2/core/core.hpp>

#include "Observer_Translational.h"
#include "TNT/tnt.h"
#include "TNT_Utils.h"
#include "ICSL/constants.h"
#include "Observer_Angular.h"
#include "QuadLogger.h"
#include "CommManager.h"
#include "VisionProcessor.h"
#include "Common.h"
#include "Time.h"
#include "TranslationController.h"
#include "AttitudeThrustController.h"

namespace ICSL {
namespace Quadrotor {
class Rover: public Observer_AngularListener,
				 public CommManagerListener,
				 public toadlet::egg::Thread
{
public:
	Rover();
	virtual ~Rover();

	void initialize();
	void shutdown();

	void setNumCpuCores(int numCores){mNumCpuCores = numCores;}
	void setLogFilename(String name);
	void setLogDir(String dir);
	void startLogging();
	void stopLogging();
	
	// these functions are primarily for the jni interface
	void copyImageData(cv::Mat *m);
	TNT::Array2D<double> getGyroValue();
	TNT::Array2D<double> getAccelValue();
	TNT::Array2D<double> getMagValue();
	TNT::Array2D<double> getAttitude();
	int getImageProcTimeMS(){mMutex_vision.lock(); int temp = mVisionProcessor.getImageProcTimeMS(); mMutex_vision.unlock(); return temp;}
	void toggleViewType(){mVisionProcessor.setViewType(mVisionProcessor.getImgViewType() == 0 ? 1 : 0);}
	void toggleUseIbvs(){onNewCommUseIbvs(!mUseIbvs);}
	toadlet::egg::Collection<int> getVisionParams();
	void setVisionParams(toadlet::egg::Collection<int> p);
	bool pcIsConnected(){return mCommManager.pcIsConnected();}

	// Observer_AngularListener
	void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel);

	// for CommManagerListener
	void onNewCommTimeSync(int time);
	void onNewCommLogTransfer();
	void onNewCommLogMask(uint32 mask);
	void onNewCommLogClear();

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

	Mutex mMutex_cntl, mMutex_observer, mMutex_vision, mMutex_vicon;
	
	void run();
	void transmitDataUDP();
	void transmitImage();

	QuadLogger mQuadLogger;

	VisionProcessor mVisionProcessor;

	bool mUseIbvs;

	TNT::Array2D<int> getCpuUsage();

	int mNumCpuCores;
}; // class Rover

} // namespace Quadrotor
} // namespace ICSL

#endif
