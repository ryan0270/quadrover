#include "TranslationController.h"
#include "TNT_Utils.h"

namespace ICSL {
namespace Quadrotor {
//using namespace ICSL::Constants;
//using namespace TNT;
	TranslationController::TranslationController() :
		mCurState(6,1,0.0),
		mDesState(6,1,0.0),
		mDesPosAccel(6,1,0.0),
		mGainP(3,1,0.0),
		mGainD(3,1,0.0),
		mGainI(3,1,0.0),
		mErrInt(3,1,0.0),
		mErrIntLimit(3,1,0.0),
		mGainCntlSys(6,1,1.0),
		mRotViconToPhone(3,3,0.0),
		mDesAccel(3,1,0.0)
	{
		mRunning = false;
		mDone = true;
		mNewMeasAvailable = false;

		mRotViconToPhone.inject(createIdentity((double)3));
		mLastControlTime.setTimeMS(0);

		mQuadLogger = NULL;
		
		mMass = 0.850;

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mUseIbvs = false;

		mLastTargetFindTime.setTimeMS(0);

		mTargetData = NULL;

		mRotCamToPhone = SO3( matmult(createRotMat(2,-0.5*(double)PI),
								      createRotMat(0,(double)PI)) );
		mRotPhoneToCam = mRotCamToPhone.inv();
	}

	TranslationController::~TranslationController()
	{
	}

	void TranslationController::shutdown()
	{
		Log::alert("------------------------- TranslationController shutdown started  --------------------------------------------------");
		mRunning = false;
		while(!mDone)
			System::msleep(10);
		Log::alert("------------------------- TranslationController shutdown done");
	}

	void TranslationController::initialize()
	{
	}

	void TranslationController::run()
	{
		mDone = false;
		mRunning = true;
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			if(mNewMeasAvailable)
				calcControl();

			System::msleep(1);
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

		mMutex_targetFindTime.lock();
		Time lastTargetFindTime(mLastTargetFindTime);
		mMutex_targetFindTime.unlock();
		if(mUseIbvs && lastTargetFindTime.getElapsedTimeMS() < 1.0e3)
			accelCmd = calcControlIBVS(dt);
		else
			accelCmd = calcControlSystem(error,dt);
//		accelCmd = calcControlPID(error,dt);

		for(int i=0; i<mListeners.size(); i++)
			mListeners[i]->onTranslationControllerAccelCmdUpdated(accelCmd);
	
		// Logging
		if(mQuadLogger != NULL)
		{
			String logString;
			for(int i=0; i<accelCmd.dim1(); i++)
				logString = logString+accelCmd[i][0]+"\t";
			mQuadLogger->addEntry(LOG_ID_ACCEL_CMD,logString, LOG_FLAG_STATE_DES);
		}

		mNewMeasAvailable = false;
	}

	Array2D<double> TranslationController::calcControlPID(const Array2D<double> &error,double dt)
	{
		Array2D<double> accelCmd(3,1);
		mMutex_data.lock();
		for(int i=0; i<3; i++)
			mErrInt[i][0] = constrain(mErrInt[i][0]+dt*error[i][0],-mErrIntLimit[i][0],mErrIntLimit[i][0]);


		Array2D<double> tempI(3,1,0.0);
		if(mDesState[2][0] > 0.1) // don't build up the integrator when we're sitting still
			tempI.inject(mGainI);
		for(int i=0; i<3; i++)
			accelCmd[i][0] =	-mGainP[i][0]*error[i][0]
								-mGainD[i][0]*error[i+3][0]
								-tempI[i][0]*mErrInt[i][0]
								+mDesAccel[i][0];
		accelCmd[2][0] += GRAVITY;
		mMutex_data.unlock();

		return accelCmd;
	}

	Array2D<double> TranslationController::calcControlSystem(const Array2D<double> &error, double dt)
	{
		Array2D<double> accelCmd(3,1);
		if(mCntlSys.isInitialized())
		{
			mMutex_data.lock();
			Array2D<double> u = mGainCntlSys*error;
			Array2D<double> x = mCntlSys.simulateEuler(u,dt);
			accelCmd = matmult(mCntlSys.getC(),x) + matmult(mCntlSys.getD(),u);
			accelCmd += mDesAccel;
			mMutex_data.unlock();
		}
		else
		{ accelCmd[0][0] = accelCmd[1][0] = accelCmd[2][0] = 0; }

		accelCmd[2][0] += GRAVITY;

		return accelCmd;
	}

	Array2D<double> TranslationController::calcControlIBVS(double dt)
	{
		mMutex_target.lock();
		shared_ptr<ImageTargetFindData> targetData = mTargetData;
		mMutex_target.unlock();

		vector<cv::Point2f> imgPoints;
		for(int i=0; i<targetData->target->squareData.size(); i++)
			for(int j=0; j<targetData->target->squareData[i]->contour.size(); j++)
				imgPoints.push_back(targetData->target->squareData[i]->contour[j]);

		double f = targetData->imageData->focalLength;
		double cx = targetData->imageData->centerX;
		double cy = targetData->imageData->centerY;
		vector<Array2D<double>> spherePoints;
		Array2D<double> p(3,1), moment(3,1,0.0);
		for(int i=0; i<imgPoints.size(); i++)
		{
			p[0][0] = (imgPoints[i].x - cx);
			p[1][0] = (imgPoints[i].y - cy);
			p[2][0] = f;

			// Get it on the unit sphere
			p.inject(1.0/norm2(p)*p);

			moment += p;
		}
		moment = 1.0/norm2(moment)*moment;
		moment = mRotCamToPhone*moment;

		SO3 att = targetData->imageData->att;
		Array2D<double> desDir(3,1);
		desDir[0][0] = 0;
		desDir[1][0] = 0;
		desDir[2][0] = 1;
		Array2D<double> desMoment = att.inv()*desDir;

		mMutex_state.lock();
		Array2D<double> visionErr = mCurState[2][0]*moment-mDesState[2][0]*desMoment;
		mMutex_state.unlock();

		Array2D<double> ibvsGainPos(3,1);
		ibvsGainPos[0][0] = 1;
		ibvsGainPos[1][0] = 1;
		ibvsGainPos[2][0] = 1;
		Array2D<double> desVel = ibvsGainPos*visionErr; // remember that * is element-wise

		String logString;
		for(int i=0; i<desVel.dim1(); i++)
			logString = logString+desVel[i][0]+"\t";
		mQuadLogger->addEntry(LOG_ID_VEL_CMD, logString, LOG_FLAG_STATE_DES);

		Array2D<double> velErr(3,1);
		velErr[0][0] = mCurState[3][0]-(mDesState[3][0]+desVel[0][0]);
		velErr[1][0] = mCurState[4][0]-(mDesState[4][0]+desVel[1][0]);
		velErr[2][0] = mCurState[5][0]-(mDesState[5][0]+desVel[2][0]);

		Array2D<double> ibvsGainVel(3,1);
		ibvsGainVel[0][0] = 1;
		ibvsGainVel[1][0] = 1;
		ibvsGainVel[2][0] = 1;
		Array2D<double> accelCmd = ibvsGainVel*velErr;

		// Decay so if we go a long time without finding the image
		// we aren't trying to do too much
		mMutex_targetFindTime.lock();
		double t = mLastTargetFindTime.getElapsedTimeNS()/1.0e9;
		mMutex_targetFindTime.unlock();
		double decayRate = 1;
		accelCmd = exp(-decayRate*t)*accelCmd;

		accelCmd[2][0] += GRAVITY;
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

	void TranslationController::onNewCommTransGains(const Collection<float> &gains)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
		{
			mGainP[i][0] = gains[i];
			mGainD[i][0] = gains[i+3];
			mGainI[i][0] = gains[i+6];
			mErrIntLimit[i][0] = gains[i+9];

			mGainCntlSys[i][0] = gains[i];
			mGainCntlSys[i+3][0] = gains[i+3];
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

	void TranslationController::onNewCommDesState(const Collection<float> &data)
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

		reset();
	}

	void TranslationController::onNewCommSendControlSystem(const Collection<tbyte> &buff)
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

	void TranslationController::onObserver_TranslationalUpdated(const TNT::Array2D<double> &pos, const TNT::Array2D<double> &vel)
	{
		mMutex_state.lock();
		for(int i=0; i<3; i++)
			mCurState[i][0] = pos[i][0];
		for(int i=3; i<6; i++)
			mCurState[i][0] = vel[i-3][0];
		mMutex_state.unlock();

		mNewMeasAvailable = true;
	}

	void TranslationController::onTargetFound(const shared_ptr<ImageTargetFindData> &data)
	{
		if(data->target != NULL)
		{
			mMutex_target.lock();
			mTargetData = data;
			mMutex_target.unlock();

			mMutex_targetFindTime.lock();
			mLastTargetFindTime.setTime();
			mMutex_targetFindTime.unlock();
		}
	}

} // namespace Quadrotor
} // namespace ICSL
