#ifndef ICSL_OBSERVER_TRANSLATIONAL
#define ICSL_OBSERVER_TRANSLATIONAL
#include <fstream>

#include <android/sensor.h>

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
								public SensorManagerListener
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

	// from MotorInterfaceListener
	void onAttitudeThrustControllerCmdsSent(double const cmds[4]);

	// for SensorManagerListener
	void onNewSensorUpdate(SensorData const *data);

	private:
	bool mRunning, mDone;
	bool mDoMeasUpdate;
	Time mStartTime;

	Array2D<double> mRotViconToPhone;
	QuadLogger *mQuadLogger;

	Collection<Observer_TranslationalListener*> mListeners;

	// for the translational Kalman Filter
	Array2D<double> mAkf, mAkf_T, mBkf, mCkf, mCkf_T;
	Array2D<double> mMeasCov, mDynCov, mErrCovKF, mGainKF;
	Array2D<double> mStateKF, mAttitude;
	TNT::Array2D<double> mAttBias, mAttBiasReset;
	TNT::Array2D<double> mLastMeas;
	double mMass, mForceGainReset, mForceGain;
	Collection<double> mAttBiasAdaptRate;
	double mForceGainAdaptRate;

	toadlet::egg::Mutex mMutex_data, mMutex_att, mMutex_meas, mMutex_cmds;

	Time mLastMeasUpdateTime, mLastPosReceiveTime;

	double mMotorCmds[4];

	void doTimeUpdateKF(TNT::Array2D<double> const &actuator, double dt);
	void doMeasUpdateKF(TNT::Array2D<double> const &meas);

	ASensorManager* mSensorManager;
	ASensorEventQueue* mSensorEventQueue;
	const ASensor* mPressureSensor;

	int getBatteryTemp();
	int getSecTemp();
	int getFuelgaugeTemp();
	int getTmuTemp();

};

} // namespace Quadrotor
} // namespace ICSL

#endif
