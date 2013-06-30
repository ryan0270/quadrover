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
#define ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
#include "VelocityEstimator.h"
#undef ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY

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
								public VisionProcessorListener,
								public VelocityEstimatorListener
{
	public:
	Observer_Translational();
	virtual ~Observer_Translational();

	void initialize();
	void run();
	void shutdown();

	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void setStartTime(Time t);
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setRotViconToPhone(TNT::Array2D<double> const &rot){mRotViconToPhone.inject(rot);}
	void setMotorCmds(double const cmds[4]);

	double getBarometerHeight(){mMutex_meas.lock(); double temp = mBarometerHeightState[0][0]+mZeroHeight; mMutex_meas.unlock(); return temp;}

	void addListener(Observer_TranslationalListener *listener){mListeners.push_back(listener);}

	TNT::Array2D<double> estimateStateAtTime(Time const &t);
	TNT::Array2D<double> estimateErrCovAtTime(Time const &t);

	// from Observer_AngularListener
	void onObserver_AngularUpdated(shared_ptr<DataVector<double> > attData, shared_ptr<DataVector<double> > angularVelData);

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
	void onNewSensorUpdate(shared_ptr<IData> const &data);

	// for VisionProcessorListener
	void onImageProcessed(shared_ptr<ImageMatchData> const data);
	void onImageTargetFound(shared_ptr<ImageTargetFindData> const data);
	void onImageLost(){};

	// for VelocityEstimatorListener
	void onVelocityEstimator_newEstimate(shared_ptr<DataVector<double> > const &velData);

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
	TNT::Array2D<double> mCkf, mCkf_T;
	TNT::Array2D<double> mMeasCov, mPosMeasCov, mVelMeasCov; 
	TNT::Array2D<double> mDynCov, mErrCovKF;
	TNT::Array2D<double> mStateKF;
	TNT::Array2D<double> mAttBias, mAttBiasReset;
	TNT::Array2D<double> mLastViconPos, mLastCameraPos;
	double mMass, mForceGainReset, mForceGain;
	Collection<double> mAttBiasAdaptRate;
	double mForceGainAdaptRate;

	toadlet::egg::Mutex mMutex_data, mMutex_att, mMutex_meas, mMutex_cmds, mMutex_phoneTempData;
	toadlet::egg::Mutex mMutex_adaptation;

	Time mLastPosReceiveTime, mLastBarometerMeasTime;
	Time mLastForceGainUpdateTime, mLastAttBiasUpdateTime;

	static void doTimeUpdateKF(TNT::Array2D<double> const &accel, 
							   double const &dt,
							   TNT::Array2D<double> &state,
							   TNT::Array2D<double> &errCov,
							   TNT::Array2D<double> const &dynCov);
	static void doMeasUpdateKF_velOnly(TNT::Array2D<double> const &meas,
									   TNT::Array2D<double> const &measCov,
									   TNT::Array2D<double> &state,
									   TNT::Array2D<double> &errCov);
	static void doMeasUpdateKF_posOnly(TNT::Array2D<double> const &meas,
									   TNT::Array2D<double> const &measCov,
									   TNT::Array2D<double> &state,
									   TNT::Array2D<double> &errCov);

	shared_ptr<DataPhoneTemp<double> > mPhoneTempData;
	double mZeroHeight;
	TNT::Array2D<double> mBarometerHeightState;

	bool mNewImageResultsReady;
	toadlet::egg::Mutex mMutex_imageData;
	shared_ptr<ImageMatchData> mImageMatchData; 

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	bool mMotorOn;

	int mThreadPriority, mScheduler;

	vector<list<shared_ptr<Data<double> > > *> mDataBuffers;
	list<shared_ptr<DataVector<double> > > mStateBuffer, mErrCovKFBuffer, mViconPosBuffer, mCameraPosBuffer;
	list<shared_ptr<DataVector<double> > > mViconVelBuffer, mCameraVelBuffer, mOpticFlowVelBuffer;
	list<shared_ptr<Data<double> > > mHeightDataBuffer;
	list<shared_ptr<DataVector<double> > > mMotorCmdsBuffer, mThrustDirBuffer;
	list<shared_ptr<Data<double> > > mThrustBuffer;
	list<shared_ptr<IData> > mNewEventsBuffer;

	bool mHaveFirstCameraPos;
	Time mLastCameraPosTime, mLastViconPosTime;
	TNT::Array2D<double> mLastViconVel, mLastCameraVel;

	bool mUseIbvs;

	TNT::Array2D<double> mViconCameraOffset;

	Time applyData(shared_ptr<IData> const &data);
	void doForceGainAdaptation(TNT::Array2D<double> const &err);
	void doAttBiasAdaptation(TNT::Array2D<double> const &err);
};

} // namespace Quadrotor
} // namespace ICSL

#endif
