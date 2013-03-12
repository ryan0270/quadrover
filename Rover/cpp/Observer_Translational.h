#ifndef ICSL_OBSERVER_TRANSLATIONAL
#define ICSL_OBSERVER_TRANSLATIONAL
#include <memory>
#include <fstream>

#include "toadlet/egg.h"

#include "TNT/tnt.h"

#include "ICSL/constants.h"
#include "ICSL/SystemModel/ISystemModelAffine.h"
#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"

#include "Time.h"
#include "CommManager.h"
#include "QuadLogger.h"
#include "Observer_Angular.h"
#include "AttitudeThrustControllerListener.h"
#include "SensorManager.h"
#include "VisionProcessor.h"

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

	// from MotorInterfaceListener
	void onAttitudeThrustControllerCmdsSent(double const cmds[4]);

	// for SensorManagerListener
	void onNewSensorUpdate(shared_ptr<SensorData> const &data);

	// for VisionProcessorListener
	void onImageProcessed(shared_ptr<ImageMatchData> const data);
	void onImageLost(){};

	protected:
	bool mRunning, mDone;
	bool mDoMeasUpdate;
	bool mDoMeasUpdate_posOnly;
	Time mStartTime;

	Array2D<double> mRotViconToPhone;
	QuadLogger *mQuadLogger;

	Collection<Observer_TranslationalListener*> mListeners;

	// for the translational Kalman Filter
	TNT::Array2D<double> mAkf, mAkf_T, mBkf, mCkf, mCkf_T;
	TNT::Array2D<double> mMeasCov, mDynCov, mErrCovKF;
	TNT::Array2D<double> mStateKF, mAttitude;
	TNT::Array2D<double> mAttBias, mAttBiasReset;
	TNT::Array2D<double> mLastMeas;
	double mMass, mForceGainReset, mForceGain;
	Collection<double> mAttBiasAdaptRate;
	double mForceGainAdaptRate;

	toadlet::egg::Mutex mMutex_data, mMutex_att, mMutex_meas, mMutex_cmds, mMutex_phoneTempData;

	Time mLastMeasUpdateTime, mLastPosReceiveTime, mLastBarometerMeasTime;

	double mMotorCmds[4];

	void doTimeUpdateKF(TNT::Array2D<double> const &actuator, double dt);
	void doMeasUpdateKF(TNT::Array2D<double> const &meas);
	void doMeasUpdateKF_velOnly(TNT::Array2D<double> const &meas);
	void doMeasUpdateKF_posOnly(TNT::Array2D<double> const &meas);

	shared_ptr<SensorDataPhoneTemp> mPhoneTempData;
	double mZeroHeight;
	TNT::Array2D<double> mBarometerHeightState;

	bool mNewImageResultsReady, mNewOpticFlowReady, mFlowCalcDone;
	toadlet::egg::Mutex mMutex_imageData;
	shared_ptr<ImageMatchData> mImageMatchData; 
	void calcOpticalFlow(shared_ptr<ImageMatchData> const img);

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	TNT::Array2D<double> mOpticFlowVel;

	bool mMotorOn;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
