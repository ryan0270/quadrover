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
		mGainPID(6,1,0.0),
		mGainPIDInt(3,1,0.0),
		mGainCntlSys(6,1,5.0),
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

		mCntlType = CNTL_TRANSLATION_PID;
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
		switch(mCntlType)
		{
			case CNTL_TRANSLATION_PID:
				accelCmd = calcControlPID(error,dt);
				break;
			case CNTL_TRANSLATION_SYS:
				accelCmd = calcControlSystem(error,dt);
				break;
			default:
				Log::alert(String()+"TranslationController::calcControl -- Unknown controller: \t"+mCntlType);
				accelCmd = Array2D<double>(3,0,0.0);
				accelCmd[2][0] += GRAVITY;
				break;
		}

		for(int i=0; i<mListeners.size(); i++)
			mListeners[i]->onTranslationControllerAccelCmdUpdated(accelCmd);
	
		// Logging
		if(mQuadLogger != NULL)
		{
			String s1 = String()+mStartTime.getElapsedTimeMS() + "\t" + "-1011" + "\t";
			for(int i=0; i<desState.dim1(); i++)
				s1 = s1+desState[i][0]+"\t";

			String s2 = String()+mStartTime.getElapsedTimeMS()+"\t"+"-1012"+"\t";
			for(int i=0; i<curState.dim1(); i++)
				s2 = s2+curState[i][0]+"\t";

			mQuadLogger->addLine(s1,STATE_DES);
			mQuadLogger->addLine(s2,STATE);
		}

		mNewMeasAvailable = false;
	}

	Array2D<double> TranslationController::calcControlPID(Array2D<double> const &error,double dt)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
			mErrInt[i][0] = constrain(mErrInt[i][0]+dt*error[i][0],-mErrIntLimit[i][0],mErrIntLimit[i][0]);

		for(int i=0; i<3; i++)
			mAccelCmd[i][0] =	-mGainPID[i][0]*error[i][0]
								-mGainPID[i+3][0]*error[i+3][0]
								-mGainPIDInt[i][0]*mErrInt[i][0]
								+mDesAccel[i][0];
		mAccelCmd[2][0] += GRAVITY;
		Array2D<double> accelCmd= mAccelCmd.copy();
		mMutex_data.unlock();

		return accelCmd;
	}

	Array2D<double> TranslationController::calcControlSystem(Array2D<double> const &error, double dt)
	{
		mMutex_data.lock();
		Array2D<double> u = mGainCntlSys*error;
		Array2D<double> x = mCntlSys.simulateEuler(u,dt);
		mAccelCmd = matmult(mCntlSys.getC(),x) + matmult(mCntlSys.getD(),u);
		mAccelCmd += 1.0/mMass*mDesAccel;
		mAccelCmd[2][0] += GRAVITY;
		Array2D<double> accelCmd= mAccelCmd.copy();
// printArray(transpose(u),"u: \t");
// printArray(transpose(x),"x: \t");
// printArray(transpoes(m
		mMutex_data.unlock();

		return accelCmd;
	}

	void TranslationController::reset()
	{
		mMutex_data.lock();
		for(int i=0; i<mErrInt.dim1(); i++)
			mErrInt[i][0] = 0;

		mCntlSys.reset();
		mMutex_data.unlock();
	}

	void TranslationController::onNewCommPosControllerGains(float const gainP[12], float const gainI[12], float const gainILimit[12], float mass, float forceScaling)
	{
		mMutex_data.lock();
		for(int i=0; i<6; i++)
			mGainPID[i][0] = gainP[i+6];
		for(int i=0; i<3; i++)
			mGainPIDInt[i][0] = gainI[i+6];
		for(int i=0; i<3; i++)
			mErrIntLimit[i][0] = gainILimit[i+6];
		mMutex_data.unlock();

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

	void TranslationController::onNewCommMotorOn()
	{
		reset();
	}

	void TranslationController::onNewCommSendControlSystem(Collection<tbyte> const &buff)
	{
		mMutex_data.lock();
		mCntlSys.deserialize(buff);

		int numIn = mCntlSys.getNumInputs();
		int numState = mCntlSys.getNumStates();
		int numOut = mCntlSys.getNumOutputs();

		mCntlSys.setCurState(Array2D<double>(numState,1,0.0));
		mMutex_data.unlock();

		String str = String()+"TranslationController -- Received controller system: ";
		str = str +numIn+" inputs x "+numOut+" outputs x "+numState+" states";
		Log::alert(str);
	}

	void TranslationController::onNewCommControlSystemGains(Collection<float> const &gains)
	{
		mMutex_data.lock();
		for(int i=0; i<6; i++)
			mGainCntlSys[i][0] = gains[i];
		mMutex_data.unlock();

		String s = "Received cntl sys gains: \t";
		for(int i=0; i<6; i++)
			s = s+gains[i]+"\t";
		Log::alert(s);
	}

	void TranslationController::onNewCommControlType(uint16 cntlType)
	{
		mMutex_data.lock();
		mCntlType = cntlType;
		if(cntlType == CNTL_TRANSLATION_SYS && mCntlSys.isInitialized() == false)
		{
			Log::alert("TranslationController -- Control system is not initialized.");
			mCntlType = CNTL_TRANSLATION_PID;
		}
		Log::alert(String()+"Control type set to "+mCntlType);
		reset();
		mMutex_data.unlock();

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
