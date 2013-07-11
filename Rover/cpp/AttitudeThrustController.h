#ifndef ICSL_ATITTUDETHRUSTCONTROLLER
#define ICSL_ATITTUDETHRUSTCONTROLLER
#include <sched.h>
#include <thread>

#include "toadlet/egg.h"

#include "TNT/tnt.h"

#include "Time.h"
#include "CommManager.h"
#include "QuadLogger.h"
#include "TranslationController.h"
#include "Observer_Angular.h"
#include "MotorInterface.h"

namespace ICSL {
namespace Quadrotor{
using namespace toadlet::egg;

class AttitudeThrustControllerListener;
//class AttitudeThrustControllerListener
//{
//	AttitudeThrustControllerListener(){};
//	virtual ~AttitudeThrustControllerListener(){};
//
//	virtual void onAttitudeThrustControllerCmdsSent(double const cmds[4])=0;
//};

class AttitudeThrustController : public CommManagerListener,
								public TranslationControllerListener,
								public Observer_AngularListener
{
	public:
	AttitudeThrustController();
	virtual ~AttitudeThrustController();

	void initialize();
	void start(){ thread th(&AttitudeThrustController::run, this); th.detach(); }
	void run();
	void shutdown();

	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setMotorInterface(MotorInterface *mi){mMotorInterface = mi;}

	void calcControl();

	Collection<uint16> getLastMotorCmds(){mMutex_motorInterface.lock(); Collection<uint16> temp(mLastMotorCmds); mMutex_motorInterface.unlock(); return temp;}
	TNT::Array2D<double> getDesAttitude(){mMutex_data.lock(); TNT::Array2D<double> temp = mDesAtt.copy(); mMutex_data.unlock(); return temp;}

	void addListener(AttitudeThrustControllerListener* l){mListeners.push_back(l);}

	// for CommManagerListener
	void onNewCommForceGain(float k);
	void onNewCommTorqueGain(float k);
	void onCommConnectionLost();
	void onNewCommMotorOn();
	void onNewCommMotorOff();
	void onNewCommMotorTrim(int const trim[4]);
	void onNewCommMass(float m);
	void onNewCommAttitudeGains(toadlet::egg::Collection<float> const &gains);
	void onNewCommMotorArmLength(float l);

	// for TranslationControllerListener
	void onTranslationControllerAccelCmdUpdated(TNT::Array2D<double> const &accelCmd);

	// for Observer_AngularListener
	void onObserver_AngularUpdated(shared_ptr<DataVector<double> > attData, shared_ptr<DataVector<double> > angularVelData);

	protected:
	bool mRunning, mDone;
	bool mDoControl, mPcIsConnected;
	Time mStartTime, mLastControlTime;
	QuadLogger *mQuadLogger;

	double mForceScaling, mTorqueScaling;
	int mMotorTrim[4];
	Collection<uint16> mLastMotorCmds;

	MotorInterface *mMotorInterface;

	double mThrust, mMass, mMotorArmLength;;
	TNT::Array2D<double> mCurAngularVel;
	TNT::Array2D<double> mCurAtt, mDesAtt, mDesRotMat, mDesRotMat_T;
	TNT::Array2D<double> mGainAngle, mGainRate;

	Collection<AttitudeThrustControllerListener*> mListeners;

	toadlet::egg::Mutex mMutex_data, mMutex_motorInterface;

	int mThreadPriority, mScheduler;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
