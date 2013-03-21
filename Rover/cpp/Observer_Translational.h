#ifndef ICSL_OBSERVER_TRANSLATIONAL
#define ICSL_OBSERVER_TRANSLATIONAL
#include <memory>
#include <fstream>
#include <sched.h>

#include "toadlet/egg.h"

#include "TNT/tnt.h"

#include "constants.h"

#include "Time.h"
#include "CommManager.h"
#include "QuadLogger.h"
#include "Observer_Angular.h"
#include "AttitudeThrustControllerListener.h"
#include "SensorManager.h"
#include "VisionProcessor.h"
#include "Data.h"

namespace ICSL{
namespace Quadrotor{
class Observer_TranslationalListener
{
	public:
	Observer_TranslationalListener(){};
	virtual ~Observer_TranslationalListener(){};

	virtual void onObserver_TranslationalUpdated(TNT::Array2D<double> const &pos, TNT::Array2D<double> const &vel)=0;
};

class Observer_Translational : public toadlet::egg::Thread,
								public Observer_AngularListener,
								public CommManagerListener,
								public AttitudeThrustControllerListener,
								public SensorManagerListener,
								public VisionProcessorListener
{
	public:
	Observer_Translational();
	virtual ~Observer_Translational();

	void initialize();
	void run();
	void shutdown();

	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setRotViconToPhone(TNT::Array2D<double> const &rot){mRotViconToPhone.inject(rot);}
	void setMotorCmds(double const cmds[4]);

	double getBarometerHeight(){mMutex_meas.lock(); double temp = mBarometerHeightState[0][0]+mZeroHeight; mMutex_meas.unlock(); return temp;}

	void addListener(Observer_TranslationalListener *listener){mListeners.push_back(listener);}

	// from Observer_AngularListener
	void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel);

	// from CommManagerListener
	void onNewCommStateVicon(toadlet::egg::Collection<float> const &data);
	void onNewCommMass(float m);
	void onNewCommForceGain(float k);
	void onNewCommAttBias(float roll, float pitch, float yaw);
	void onNewCommAttBiasAdaptRate(toadlet::egg::Collection<float> const &rate);
	void onNewCommForceGainAdaptRate(float rate);
	void onNewCommKalmanMeasVar(toadlet::egg::Collection<float> const &var);
	void onNewCommKalmanDynVar(toadlet::egg::Collection<float> const &var);
	void onNewCommBarometerZeroHeight(float h);
	void onNewCommMotorOn(){mMotorOn = true;}
	void onNewCommMotorOff(){mMotorOn = false;}
	void onNewCommUseIbvs(bool useIbvs);

	// from MotorInterfaceListener
	void onAttitudeThrustControllerCmdsSent(double const cmds[4]);

	// for SensorManagerListener
	void onNewSensorUpdate(shared_ptr<Data> const &data);

	// for VisionProcessorListener
	void onImageProcessed(shared_ptr<ImageMatchData> const data);
	void onImageTargetFound(shared_ptr<ImageTargetFindData> const data);
	void onImageLost(){};

	protected:
	bool mRunning, mDone;
	bool mDoMeasUpdate;
	bool mNewViconPosAvailable, mNewCameraPosAvailable;
	bool mUseViconPos, mUseCameraPos;
	Time mStartTime;

	TNT::Array2D<double> mRotViconToPhone;
	QuadLogger *mQuadLogger;

	Collection<Observer_TranslationalListener*> mListeners;

	// for the translational Kalman Filter
	TNT::Array2D<double> mAkf, mAkf_T, mBkf, mCkf, mCkf_T;
	TNT::Array2D<double> mMeasCov, mPosMeasCov, mVelMeasCov; 
	TNT::Array2D<double> mDynCov, mErrCovKF;
	TNT::Array2D<double> mStateKF, mAttitude;
	TNT::Array2D<double> mAttBias, mAttBiasReset;
//	TNT::Array2D<double> mLastMeas;
	TNT::Array2D<double> mLastViconPos, mLastCameraPos;
	double mMass, mForceGainReset, mForceGain;
	Collection<double> mAttBiasAdaptRate;
	double mForceGainAdaptRate;

	toadlet::egg::Mutex mMutex_data, mMutex_att, mMutex_meas, mMutex_cmds, mMutex_phoneTempData;

	Time mLastPosReceiveTime, mLastBarometerMeasTime;
	Time mLastForceGainUpdateTime, mLastAttBiasUpdateTime;

	double mMotorCmds[4];

	void doTimeUpdateKF(TNT::Array2D<double> const &actuator, double dt);
	void doMeasUpdateKF(TNT::Array2D<double> const &meas);
	void doMeasUpdateKF_velOnly(TNT::Array2D<double> const &meas, TNT::Array2D<double> const &measCov);
	void doMeasUpdateKF_posOnly(TNT::Array2D<double> const &meas, TNT::Array2D<double> const &measCov);

	shared_ptr<DataPhoneTemp> mPhoneTempData;
	double mZeroHeight;
	TNT::Array2D<double> mBarometerHeightState;

	bool mNewImageResultsReady, mNewOpticFlowReady, mFlowCalcDone;
	toadlet::egg::Mutex mMutex_imageData;
	shared_ptr<ImageMatchData> mImageMatchData; 
	void calcOpticalFlow(shared_ptr<ImageMatchData> const img);

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	TNT::Array2D<double> mOpticFlowVel;
	Time mOpticFlowVelTime;

	bool mMotorOn;

	int mThreadPriority, mScheduler;

	list<shared_ptr<DataVector> > mStateDataBuffer, mAccelDataBuffer, mErrCovKFDataBuffer, mPosMeasDataBuffer;
	list<shared_ptr<Data> > mHeightDataBuffer;
//	list<TNT::Array2D<double> > mVelMeasBuffer; // This is only the vel meas from position updates
//	list<Time> mVelMeasTimeBuffer;

	bool mHaveFirstCameraPos;
	Time mLastCameraPosTime, mLastViconPosTime;
	TNT::Array2D<double> mLastViconVel, mLastCameraVel;

	bool mUseIbvs;

	TNT::Array2D<double> mViconCameraOffset;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
