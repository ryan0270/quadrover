#include "TranslationController.h"
#include "TNT_Utils.h"

using namespace toadlet;
using namespace ICSL::Constants;

namespace ICSL {
namespace Quadrotor {
	TranslationController::TranslationController() :
		mCurState(6,1,0.0),
		mDesState(6,1,0.0),
		mDesPosAccel(6,1,0.0),
		mGainP(3,1,0.0),
		mGainD(3,1,0.0),
		mGainI(3,1,0.0),
		mErrInt(3,1,0.0),
		mErrIntLimit(3,1,0.0),
		mRotViconToPhone(3,3,0.0),
		mAccelCmd(3,1,0.0),
		mDesAccel(3,1,0.0)
	{
		mRunning = false;
		mDone = true;
		mNewMeasAvailable = false;

		mRotViconToPhone.inject(createIdentity(3));
		mLastControlTime.setTimeMS(0);

		mQuadLogger = NULL;
		
		mMass = 0.850;

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
	}

	TranslationController::~TranslationController()
	{
	}

	void TranslationController::shutdown()
	{
		Log::alert("------------------------- TranslationController shutdown started  --------------------------------------------------");
		mRunning = false;
		System sys;
		while(!mDone)
			sys.msleep(10);
		Log::alert("------------------------- TranslationController shutdown done");
	}

	void TranslationController::initialize()
	{
	}

	void TranslationController::run()
	{
		mDone = false;
		mRunning = true;
		System sys;
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			if(mNewMeasAvailable)
			{
				calcControl();
			}
			sys.msleep(1);
		}

		mDone = true;
	}

	void TranslationController::calcControl()
	{
		double dt;
		if(mLastControlTime.getMS() == 0)
			dt = 0;
		else
			dt = mLastControlTime.getElapsedTimeUS()/1.0e6;
		mLastControlTime.setTime();

		mMutex_state.lock();
		Array2D<double> error = mCurState-mDesState;
		Array2D<double> desState = mDesState.copy();
		Array2D<double> curState = mCurState.copy();
		mMutex_state.unlock();

		Array2D<double> accelCmd;
		accelCmd = calcControlPID(error,dt);

		for(int i=0; i<mListeners.size(); i++)
			mListeners[i]->onTranslationControllerAccelCmdUpdated(accelCmd);
	
		// Logging
		if(mQuadLogger != NULL)
		{
			String s1 = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_DES_TRANS_STATE + "\t";
			for(int i=0; i<desState.dim1(); i++)
				s1 = s1+desState[i][0]+"\t";

			String s2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_CUR_TRANS_STATE+"\t";
			for(int i=0; i<curState.dim1(); i++)
				s2 = s2+curState[i][0]+"\t";

			mQuadLogger->addLine(s1,LOG_FLAG_STATE_DES);
			mQuadLogger->addLine(s2,LOG_FLAG_STATE);
		}

		mNewMeasAvailable = false;
	}

	Array2D<double> TranslationController::calcControlPID(Array2D<double> const &error,double dt)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
			mErrInt[i][0] = constrain(mErrInt[i][0]+dt*error[i][0],-mErrIntLimit[i][0],mErrIntLimit[i][0]);


		for(int i=0; i<3; i++)
			mAccelCmd[i][0] =	-mGainP[i][0]*error[i][0]
								-mGainD[i][0]*error[i+3][0]
								-mGainI[i][0]*mErrInt[i][0]
								+mDesAccel[i][0];
		mAccelCmd[2][0] += GRAVITY;
		Array2D<double> accelCmd= mAccelCmd.copy();
		mMutex_data.unlock();

		return accelCmd;
	}

	void TranslationController::reset()
	{
		mMutex_data.lock();
		for(int i=0; i<mErrInt.dim1(); i++)
			mErrInt[i][0] = 0;

		mMutex_data.unlock();
	}

	void TranslationController::onNewCommTransGains(Collection<float> const &gains)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
		{
			mGainP[i][0] = gains[i];
			mGainD[i][0] = gains[i+3];
			mGainI[i][0] = gains[i+6];
			mErrIntLimit[i][0] = gains[i+9];
		}
		mMutex_data.unlock();

		{
			String s = "New translation gains: ";
			for(int i=0; i<gains.size(); i++)
				s = s+gains[i]+"\t";
			Log::alert(s);
		}

		reset();
	}

	void TranslationController::onNewCommDesState(Collection<float> const &data)
	{
		Array2D<double> desPos(3,1), desVel(3,1), desAcc(3,1);
		for(int i=0; i<3; i++)
			desPos[i][0] = data[i+6];
		for(int i=0; i<3; i++)
			desVel[i][0] = data[i+9];
		for(int i=0; i<3; i++)
			desAcc[i][0] = data[i+12];

		desPos.inject(matmult(mRotViconToPhone,desPos));
		desVel.inject(matmult(mRotViconToPhone,desVel));
		desAcc.inject(matmult(mRotViconToPhone,desAcc));

		mMutex_state.lock();
		for(int i=0; i<3; i++)
			mDesState[i][0] = desPos[i][0];
		for(int i=3; i<5; i++)
			mDesState[i][0] = desVel[i-3][0];
		mDesAccel.inject(desAcc);
		mMutex_state.unlock();
	}

	void TranslationController::onNewCommSetDesiredPos()
	{
		mMutex_state.lock();
		for(int i=0; i<3; i++)
			mDesState[i][0] = mCurState[i][0];
		mMutex_state.unlock();
		Log::alert("Desired position set.");
	}

	void TranslationController::onNewCommMotorOn()
	{
		reset();
	}

	void TranslationController::onObserver_TranslationalUpdated(TNT::Array2D<double> const &pos, TNT::Array2D<double> const &vel)
	{
		mMutex_state.lock();
		for(int i=0; i<3; i++)
			mCurState[i][0] = pos[i][0];
		for(int i=3; i<6; i++)
			mCurState[i][0] = vel[i-3][0];
		mMutex_state.unlock();

		mNewMeasAvailable = true;
	}


} // namespace Quadrotor
} // namespace ICSL
