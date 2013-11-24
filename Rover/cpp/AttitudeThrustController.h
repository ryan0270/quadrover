#ifndef ICSL_ATITTUDETHRUSTCONTROLLER
#define ICSL_ATITTUDETHRUSTCONTROLLER
#include <sched.h>
#include <thread>
#include <mutex>

#include "TNT/tnt.h"

#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"

#include "QuadLogger.h"
#include "Time.h"
#include "Listeners.h"
#include "MotorInterface.h"
#include "Rotation.h"

#include "toadlet/egg.h"

namespace ICSL {
namespace Quadrotor{

class AttitudeThrustController : public CommManagerListener,
								public TranslationControllerListener,
								public Observer_AngularListener,
								public MotorInterfaceListener
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

	Collection<uint16_t> getLastMotorCmds(){mMutex_motorInterface.lock(); Collection<uint16_t> temp(mLastMotorCmds); mMutex_motorInterface.unlock(); return temp;}
	TNT::Array2D<double> getDesAttitude();

	void addListener(AttitudeThrustControllerListener* l){mListeners.push_back(l);}

	// for CommManagerListener
	void onNewCommForceGain(float k);
	void onNewCommTorqueGain(float k);
	void onCommConnectionLost();
//	void onNewCommMotorOn();
	void onNewCommMotorOff();
	void onNewCommMotorTrim(const int trim[4]);
	void onNewCommMass(float m);
	void onNewCommAttitudeGains(const toadlet::egg::Collection<float> &gains);
	void onNewCommMotorArmLength(float l);

	// for TranslationControllerListener
	void onTranslationControllerAccelCmdUpdated(const TNT::Array2D<double> &accelCmd);

	// for Observer_AngularListener
	void onObserver_AngularUpdated(const shared_ptr<SO3Data<double>> &attData, const shared_ptr<DataVector<double>> &angularVelData);

	// for MotorInterfaceListener
	void onMotorWarmupDone();

	protected:
	bool mRunning, mDone;
	bool mDoControl, mPcIsConnected;
	Time mStartTime, mLastControlTime;
	QuadLogger *mQuadLogger;

	double mForceScaling, mTorqueScaling;
	int mMotorTrim[4];
	Collection<uint16_t> mLastMotorCmds;

	MotorInterface *mMotorInterface;

	double mThrust, mMass, mMotorArmLength;;
	TNT::Array2D<double> mCurAngularVel;
	TNT::Array2D<double> mGainAngle, mGainRate;

	Collection<AttitudeThrustControllerListener*> mListeners;

	std::mutex mMutex_data, mMutex_motorInterface;

	int mThreadPriority, mScheduler;

	TNT::Array2D<double> mDesAccel;

	SO3 mCurAtt, mDesAtt;
	SO3 mMotorPlaneBias;

	// For the reference model
	Array2D<double> mRefState;
//	double mRefDamping, mRefNaturalFreq;
	double mRefB, mRefC;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
