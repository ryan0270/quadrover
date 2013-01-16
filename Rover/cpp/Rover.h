#ifndef CHADPHONE_H
#define CHADPHONE_H
#include <fstream>
#include <sstream>

#include <toadlet/egg.h>
#include <toadlet/flick.h>
using toadlet::int64;
using toadlet::uint64;
using toadlet::egg::String;

#include <android/asset_manager.h>
#include <android/asset_manager_jni.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Observer_Translational.h"
#include "TNT/tnt.h"
#include "TNT_Utils.h"
#include "ICSL/constants.h"
#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"
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
				 public VisionProcessorListener,
//				 public TranslationControllerListener,
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
	void setAssetManager(AAssetManager *mgr){mAssetManager = mgr;}

	// Observer_AngularListener
	void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel);

	// for CommManagerListener
	void onNewCommTimeSync(int time);
	void onNewCommLogTransfer();
	void onNewCommLogMask(uint32 mask);
	void onNewCommLogClear();
	void onNewCommStateVicon(toadlet::egg::Collection<float> const &data);
	void onCommConnectionLost();
	void onNewCommIbvsGains(toadlet::egg::Collection<float> const &gains);
	void onNewCommDesiredImageMoment(toadlet::egg::Collection<float> const &data);
	void onNewCommUseIbvs(bool useIbvs);

	// VisionProcessor callback
	void onImageProcessed(toadlet::egg::Collection<cv::Point2f> const &boxCenters, toadlet::egg::Collection<bool> const &boxFound, TNT::Array2D<double> const &imgAtt, TNT::Array2D<double> const &rotVel);
	void onImageLost(){onNewCommUseIbvs(false);}

protected:
	CommManager mCommManager;
	bool mRunCommPC, mRunnerIsDone;
	bool mDoInnerControl, mDoOuterControl;
	bool mDataIsSending, mImageIsSending;

	TranslationController mTranslationController;
	AttitudeThrustController mAttitudeThrustController;

	uint16 mThrottlePC, mThrottleIbvs;
	TNT::Array2D<double> mDesiredAtt, mDesAngularVelPC, mDesAngularVelIbvs;

	Observer_Angular mObsvAngular;
	Observer_Translational mObsvTranslational;
	TNT::Array2D<double> mCurAtt, mCurAngularVel;

	Time mLastGyroTime, mStartTime, mLastDataSendTime, mLastImageSendTime;

	TNT::Array2D<double> mRotViconToQuad, mRotQuadToPhone, mRotCamToPhone, mRotPhoneToCam, mRotViconToPhone;

	Mutex mMutex_cntl, mMutex_observer, mMutex_vision, mMutex_vicon;
	
	uint16 mLastMotorVal[4], mLastMotorValIbvs[4];
	Time mLastInnerLoopCntlTime, mLastOuterLoopCntlTime;

	uint32 mCntlCalcTimeUS;

	void run();
	void transmitDataUDP();
	void transmitImage();

	QuadLogger mQuadLogger;

	VisionProcessor mVisionProcessor;

	TNT::Array2D<double> mLastImageAtt, mLastImageRotVel;
	toadlet::egg::Collection<cv::Point2f> mBoxCenters;
	toadlet::egg::Collection<bool> mBoxFound;
	TNT::Array2D<double> mCurFeat;
	TNT::Array2D<double> mDesCentroid, mDesFeat;
	TNT::Array2D<double> mFeatErrInt;
	double mFocalLength;
	bool mUseIbvs, mFirstImageProcessed, mIbvsControlReady;
	TNT::Array2D<double> mGainImg;
	TNT::Array2D<double> mGainFlow, mGainFlowInt;
	TNT::Array2D<double> mGainAngularRateIBVS;
	TNT::Array2D<double> mGainFF;
	TNT::Array2D<double> mTorque2Cmd;
	double mGainAngleIBVS; // the stability proof right now only works for a scalar multiple ... not sure if I can do separate gains by channel
	double mGainDynamicIBVS;

	TNT::Array2D<double> mStateVicon;

	TNT::Array2D<double> mRotErr; // right now this is only used when visual servoing
	TNT::Array2D<double> mAttCmdOffset, mAttBias;
	TNT::Array2D<double> mFlow, mFlowErr_phone, mFlowErrInt;
	TNT::Array2D<double> mDesFlow; 
	TNT::Array2D<double> mCentroidErrInt;
	double mHeight, mRho;

	Collection<TNT::Array2D<double> > mBoxProjections;

	TNT::Array2D<double> mCentroid;
	double mMass;

	TNT::Array2D<double> mRotVelSum;
	int mRotVelSum_count;

	double mThrottleBase;
	
	void processImage();
//	static bool compareBlobsBySize(cv::KeyPoint p1, cv::KeyPoint p2)
//		{return (p1.size > p2.size);}
//	static cv::Point2f applyRollPitchCompensation(cv::Point2f const &p, float roll, float pitch, float focalLength);


	TNT::Array2D<int> getCpuUsage();

	int mNumCpuCores;

	AAssetManager *mAssetManager;
}; // class Rover

} // namespace Quadrotor
} // namespace ICSL

#endif
