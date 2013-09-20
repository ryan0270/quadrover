#ifndef ICSL_TRANSLATIONALCONTROLLER
#define ICSL_TRANSLATIONALCONTROLLER
#include <sched.h>
#include <thread>

#include "TNT/tnt.h"

#include "constants.h"

#include "QuadLogger.h"
#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"
#include "Time.h"
#include "CommManager.h"
#include "Observer_Translational.h"
#include "MotorInterface.h"
#include "Rotation.h"

#include "toadlet/egg.h"

namespace ICSL {
namespace Quadrotor {

class TranslationControllerListener
{
	public:
	TranslationControllerListener(){};
	virtual ~TranslationControllerListener(){};

	virtual void onTranslationControllerAccelCmdUpdated(const TNT::Array2D<double> &accelCmd)=0;
};

class TranslationController : 	public Observer_TranslationalListener,
								public CommManagerListener,
								public MotorInterfaceListener,
								public TargetFinderListener
{
	public:
	TranslationController();
	virtual ~TranslationController();

	void initialize();
	void start(){ thread th(&TranslationController::run, this); th.detach(); }
	void run();
	void shutdown();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	const TNT::Array2D<double> getDesiredState(){mMutex_data.lock(); TNT::Array2D<double> tempState = mDesState.copy(); mMutex_data.unlock(); return tempState;}
	const TNT::Array2D<double> getCurState(){mMutex_data.lock(); TNT::Array2D<double> tempState = mCurState.copy(); mMutex_data.unlock(); return tempState;}
	const TNT::Array2D<double> getErrorMemory(){mMutex_data.lock(); TNT::Array2D<double> tempInt = mErrInt.copy(); mMutex_data.unlock(); return tempInt;}

	void calcControl();
	void reset();

	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setRotViconToPhone(const TNT::Array2D<double> &rot){mRotViconToPhone.inject(rot);}
	void setDesPosAccel(const TNT::Array2D<double> &a);

	void addListener(TranslationControllerListener* listener){mListeners.push_back(listener);}

	// from CommManagerListener
	void onNewCommTransGains(const toadlet::egg::Collection<float> &gains);
	void onNewCommMass(float m){mMass = m;}
	void onNewCommDesState(const toadlet::egg::Collection<float> &data);
	void onNewCommSetDesiredPos();
	void onNewCommMotorOn(){reset();}
	void onNewCommMotorOff(){reset();}
	void onNewCommSendControlSystem(const Collection<tbyte> &buff);
	void onNewCommUseIbvs(bool useIbvs){mUseIbvs = useIbvs;}
	void onNewCommIbvsGains(const toadlet::egg::Collection<float> &posGains, const toadlet::egg::Collection<float> &VELgAINS);
	void onNewCommStateVicon(const toadlet::egg::Collection<float> &data);

	// for Observer_TranslationalListener
	void onObserver_TranslationalUpdated(const TNT::Array2D<double> &pos, const TNT::Array2D<double> &vel);

	// for MotorInterfaceListener
	void onMotorWarmupDone(){reset();Log::alert("Tran Controller Received motor warmup done");}
	
	// for TargetFinderListener
	void onTargetFound(const shared_ptr<ImageTargetFindData> &data);

	protected:
	bool mRunning, mDone;
	bool mNewMeasAvailable;
	bool mUseIbvs;
	QuadLogger *mQuadLogger;
	TNT::Array2D<double> mCurState, mDesState, mDesPosAccel;
	TNT::Array2D<double> mGainP, mGainD, mGainI;
	TNT::Array2D<double> mErrInt, mErrIntLimit;
	TNT::Array2D<double> mRotViconToPhone;
	TNT::Array2D<double> mDesAccel;
	double mMass;

	toadlet::egg::Mutex mMutex_data, mMutex_state;

	Time mStartTime, mLastControlTime;

	Time mLastTargetFindTime;
	std::mutex mMutex_targetFindTime;

	Collection<TranslationControllerListener*> mListeners;

	static double constrain(double val, double minVal, double maxVal)
	{ return min(maxVal, max(minVal, val)); }

	TNT::Array2D<double> calcControlPID(const TNT::Array2D<double> &error, double dt);

	// for the Hinf controller
	ICSL::SystemModelLinear mCntlSys;
	TNT::Array2D<double> mGainCntlSys;
	TNT::Array2D<double> calcControlSystem(const TNT::Array2D<double> &error, double dt);

	TNT::Array2D<double> calcControlIBVS(double dt);
	TNT::Array2D<double> mIbvsPosGains, mIbvsVelGains;
	std::mutex mMutex_gains;

	TNT::Array2D<double> mStateVicon;
	std::mutex mMutex_viconState;

	int mThreadPriority, mScheduler;

	shared_ptr<ImageTargetFindData> mTargetData;
	std::mutex mMutex_target;

	SO3 mRotPhoneToCam, mRotCamToPhone;

	enum class Controller
	{
		PID,
		SYSTEM,
		IBVS
	};
	Controller mLastController;
};

} // namespace Quadrotor
} // namespace ICSL

#endif
