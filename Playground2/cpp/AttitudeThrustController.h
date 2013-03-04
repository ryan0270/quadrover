#ifndef ICSL_ATITTUDETHRUSTCONTROLLER
#define ICSL_ATITTUDETHRUSTCONTROLLER

#include "toadlet/egg.h"

#include "TNT/tnt.h"

#include "Time.h"
#include "CommManager.h"
#include "QuadLogger.h"
#include "TranslationController.h"
#include "Observer_Angular.h"
#include "MotorInterface.h"

using namespace toadlet::egg;

namespace ICSL {
namespace Quadrotor{
class AttitudeThrustControllerListener;
//class AttitudeThrustControllerListener
//{
//	AttitudeThrustControllerListener(){};
//	virtual ~AttitudeThrustControllerListener(){};
//
//	virtual void onAttitudeThrustControllerCmdsSent(double const cmds[4])=0;
//};

class AttitudeThrustController : public toadlet::egg::Thread,
									public CommManagerListener,
									public TranslationControllerListener,
									public Observer_AngularListener
{
	public:
	AttitudeThrustController();
	virtual ~AttitudeThrustController();

	void initialize();
	void run();
	void shutdown();

	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

	void calcControl();

	void enableMotors(bool enabled);
//	bool isMotorInterfaceConnected();
	// TODO: Need to adjust MotorInterface class so this can be a const pointer
	MotorInterface* getMotorInterface(){return &mMotorInterface;}

	Collection<uint8> getLastMotorCmds(){mMutex_motorInterface.lock(); Collection<uint8> temp(mLastMotorCmds); mMutex_motorInterface.unlock(); return temp;}
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
	void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel);

	protected:
	bool mRunning, mDone;
	bool mDoControl, mPcIsConnected;
	Time mStartTime, mLastControlTime;
	QuadLogger *mQuadLogger;

	double mForceScaling, mTorqueScaling;
	int mMotorTrim[4];
	Collection<uint8> mLastMotorCmds;

	MotorInterface mMotorInterface;

	double mThrust, mMass, mMotorArmLength;;
	TNT::Array2D<double> mCurAngularVel;
	TNT::Array2D<double> mCurAtt, mDesAtt, mDesRotMat, mDesRotMat_T;
	TNT::Array2D<double> mGainAngle, mGainRate;

	Collection<AttitudeThrustControllerListener*> mListeners;

	toadlet::egg::Mutex mMutex_data, mMutex_motorInterface;
};

} // namespace Quadrotor
} // namespace ICSL

#endif