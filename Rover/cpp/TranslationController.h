#ifndef ICSL_TRANSLATIONALCONTROLLER
#define ICSL_TRANSLATIONALCONTROLLER
#include <sched.h>
#include <thread>

#include "toadlet/egg.h"

#include "TNT/tnt.h"

#include "constants.h"

#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"
#include "Time.h"
#include "CommManager.h"
#include "QuadLogger.h"
#include "Observer_Translational.h"
#include "MotorInterface.h"

namespace ICSL {
namespace Quadrotor {

class TranslationControllerListener
{
	public:
	TranslationControllerListener(){};
	virtual ~TranslationControllerListener(){};

	virtual void onTranslationControllerAccelCmdUpdated(TNT::Array2D<double> const &accelCmd)=0;
};

class TranslationController : 	public Observer_TranslationalListener,
								public CommManagerListener,
								public MotorInterfaceListener
{
	public:
	TranslationController();
	virtual ~TranslationController();

	void initialize();
	void start(){ thread th(&TranslationController::run, this); th.detach(); }
	void run();
	void shutdown();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	TNT::Array2D<double> const getDesiredState(){mMutex_data.lock(); TNT::Array2D<double> tempState = mDesState.copy(); mMutex_data.unlock(); return tempState;}
	TNT::Array2D<double> const getCurState(){mMutex_data.lock(); TNT::Array2D<double> tempState = mCurState.copy(); mMutex_data.unlock(); return tempState;}
	TNT::Array2D<double> const getErrorMemory(){mMutex_data.lock(); TNT::Array2D<double> tempInt = mErrInt.copy(); mMutex_data.unlock(); return tempInt;}

	void calcControl();
	void reset();

	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setRotViconToPhone(TNT::Array2D<double> const &rot){mRotViconToPhone.inject(rot);}
	void setDesPosAccel(TNT::Array2D<double> const &a);

	void addListener(TranslationControllerListener* listener){mListeners.push_back(listener);}

	// from CommManagerListener
	void onNewCommTransGains(toadlet::egg::Collection<float> const &gains);
	void onNewCommMass(float m){mMass = m;}
	void onNewCommDesState(toadlet::egg::Collection<float> const &data);
	void onNewCommSetDesiredPos();
	void onNewCommMotorOn(){reset();}
	void onNewCommMotorOff(){reset();}
	void onNewCommSendControlSystem(Collection<tbyte> const &buff);

	// for Observer_TranslationalListener
	void onObserver_TranslationalUpdated(TNT::Array2D<double> const &pos, TNT::Array2D<double> const &vel);

	// for MotorInterfaceListener
	void onMotorWarmupDone(){reset();Log::alert("Tran Controller Received motor warmup done");}

	protected:
	bool mRunning, mDone;
	bool mNewMeasAvailable;
	QuadLogger *mQuadLogger;
	TNT::Array2D<double> mCurState, mDesState, mDesPosAccel;
	TNT::Array2D<double> mGainP, mGainD, mGainI;
	TNT::Array2D<double> mErrInt, mErrIntLimit;
	TNT::Array2D<double> mRotViconToPhone;
	TNT::Array2D<double> mDesAccel;
	double mMass;

	toadlet::egg::Mutex mMutex_data, mMutex_state;

	Time mStartTime, mLastControlTime;

	Collection<TranslationControllerListener*> mListeners;

	static double constrain(double val, double minVal, double maxVal)
	{ return min(maxVal, max(minVal, val)); }

	TNT::Array2D<double> calcControlPID(TNT::Array2D<double> const &error, double dt);

	// for the Hinf controller
	ICSL::SystemModelLinear mCntlSys;
	TNT::Array2D<double> mGainCntlSys;
	TNT::Array2D<double> calcControlSystem(TNT::Array2D<double> const &error, double dt);

	int mThreadPriority, mScheduler;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
