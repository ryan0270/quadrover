#include "TranslationController.h"
#include "TNT_Utils.h"

namespace ICSL {
namespace Quadrotor {
using namespace ICSL::Constants;
using namespace TNT;
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
		mDesAccel(3,1,0.0),
		mIbvsPosGains(3,1,1.0),
		mIbvsVelGains(3,1,1.0),
		mStateVicon(12,1,0.0)
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
//			if(mNewMeasAvailable)
			{
				mNewMeasAvailable = false;
				calcControl();
			}

			System::msleep(20);
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
		mMutex_state.unlock();

		Array2D<double> accelCmd;

		mMutex_target.lock();
		int targetFindDeltaMS;
//		if(mTargetData == NULL)
//			targetFindDeltaMS = 999999;
//		else
//			targetFindDeltaMS = mTargetData->timestamp.getElapsedTimeMS();
		if(mTargetTranslationData == NULL)
			targetFindDeltaMS = 999999;
		else
			targetFindDeltaMS = mTargetTranslationData->timestamp.getElapsedTimeMS();
		mMutex_target.unlock();
		if(mUseIbvs && targetFindDeltaMS < 1.0e3)
		{
			// so when we loose the image we don't try jumping back
			mMutex_state.lock();
			mDesState[0][0] = mCurState[0][0];
			mDesState[1][0] = mCurState[1][0];
			mMutex_state.unlock();

//			accelCmd = calcControlIBVS();
			accelCmd = calcControlIBVS2();

			// fake the system controller so when we switch back to 
			// it the integrator is still valid
//			for(int i=0; i<error.dim1(); i++)
//				error[i][0] = 0;
//			calcControlSystem(error, dt);
		}
		else
		{
//			accelCmd = calcControlSystem(error,dt);
			accelCmd = calcControlPID(error,dt);
		}

		double accelLimit = 10;
		accelCmd[0][0] = min(accelLimit, max(-accelLimit, accelCmd[0][0]));
		accelCmd[1][0] = min(accelLimit, max(-accelLimit, accelCmd[1][0]));
		accelCmd[2][0] = min(GRAVITY+accelLimit, max(GRAVITY-accelLimit, accelCmd[2][0]));

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
	}

	Array2D<double> TranslationController::calcControlPID(const Array2D<double> &error,double dt)
	{
		Array2D<double> accelCmd(3,1);
		mMutex_data.lock();
		for(int i=0; i<3; i++)
			mErrInt[i][0] = constrain(mErrInt[i][0]+dt*error[i][0],-mErrIntLimit[i][0],mErrIntLimit[i][0]);

		Array2D<double> tempI(3,1,0.0);
		mMutex_state.lock();
		if(mDesState[2][0] > 0.2) // don't build up the integrator when we're sitting still
			tempI.inject(mGainI);
		mMutex_state.unlock();
		for(int i=0; i<3; i++)
			accelCmd[i][0] =	-mGainP[i][0]*error[i][0]
								-mGainD[i][0]*error[i+3][0]
								-tempI[i][0]*mErrInt[i][0]
								+mDesAccel[i][0];
		accelCmd[2][0] += GRAVITY;
		mMutex_data.unlock();

		mLastController = Controller::PID;
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

		mLastController = Controller::SYSTEM;
		return accelCmd;
	}

	Array2D<double> TranslationController::calcControlIBVS()
	{
Log::alert("TranslationController::calcControlIBVS -- Why am I here?");
		Array2D<double> accel(3,1,0.0);
		accel[2][0] = GRAVITY;
		return accel;
//		mMutex_target.lock();
//		shared_ptr<ImageTargetFindData> targetData = mTargetData;
//		mMutex_target.unlock();
//		
//		vector<cv::Point2f> imgPoints;
//		for(int i=0; i<targetData->target->squareData.size(); i++)
//			for(int j=0; j<targetData->target->squareData[i]->contour.size(); j++)
//				imgPoints.push_back(targetData->target->squareData[i]->contour[j]);
//
//		// move center from corner to middle
//		double cx = targetData->imageData->center.x;
//		double cy = targetData->imageData->center.y;
//		cv::Point2f center(cx,cy);
//		for(int i=0; i<imgPoints.size(); i++)
//			imgPoints[i] -= center;
//
//		// Predict the target's current position based on kinematics
//		double dt = targetData->timestamp.getElapsedTimeNS()/1.0e9;
//		double f = targetData->imageData->focalLength;
//		Array2D<double> vel(3,1);
//		double z;
//		if(mObsvTranslational == NULL)
//		{
//			Log::alert("crap, TranslationController's mObsvTranslational is null");
//			vel[0][0] = vel[1][0] = vel[2][0] = 0;
//			z = 1;
//		}
//		else
//		{
//			Array2D<double> imgState = mObsvTranslational->estimateStateAtTime(targetData->timestamp);
//			z = imgState[2][0];
//			vel[0][0] = imgState[3][0];
//			vel[1][0] = imgState[4][0];
//			vel[2][0] = imgState[5][0];
//			vel.inject(mRotPhoneToCam*vel);
//		}
//		Array2D<double> Lv(2,3), delta(2,1);
//		for(int i=0; i<imgPoints.size(); i++)
//		{
//			Lv[0][0] = -f; Lv[0][1] = 0;  Lv[0][2] = imgPoints[i].x;
//			Lv[1][0] = 0;  Lv[1][1] = -f; Lv[1][2] = imgPoints[i].y;
//
//			delta.inject(dt/z*matmult(Lv,vel));
//
//			imgPoints[i].x += delta[0][0];
//			imgPoints[i].y += delta[1][0];
//		}
//
//		// project onto unit sphere
//		vector<Array2D<double>> spherePoints;
//		Array2D<double> p(3,1), moment(3,1,0.0);
//		for(int i=0; i<imgPoints.size(); i++)
//		{
//			p[0][0] = imgPoints[i].x;
//			p[1][0] = imgPoints[i].y;
//			p[2][0] = f;
//
//			// Get it on the unit sphere
//			p.inject(1.0/norm2(p)*p);
//
//			moment += p;
//		}
//		moment = 1.0/norm2(moment)*moment;
//		moment = mRotCamToPhone*moment;
//
//		SO3 att = targetData->imageData->att;
//		Array2D<double> desDir(3,1);
//		desDir[0][0] = 0;
//		desDir[1][0] = 0;
//		desDir[2][0] = -524.0/2.0;;
//		desDir = 1.0/norm2(desDir)*desDir;
//		Array2D<double> desMoment = att.inv()*desDir;
//
//		mMutex_state.lock();
//		Array2D<double> curState = mCurState.copy();
//		Array2D<double> desState = mDesState.copy();
//		mMutex_state.unlock();
//		mMutex_gains.lock();
//		Array2D<double> posGains = mIbvsPosGains;
//		Array2D<double> velGains = mIbvsVelGains;
//		mMutex_gains.unlock();
//
////		Array2D<double> visionErr = curState[2][0]*moment-desState[2][0]*desMoment;
//		Array2D<double> visionErr = curState[2][0]*(moment-desMoment);
//
//		Array2D<double> desVel = posGains*visionErr; // remember that * is element-wise
//
//		// use real height for z vel
//		desVel[2][0] = -mIbvsPosGains[2][0]*(curState[2][0]-desState[2][0]);
//
//		String logString;
//		for(int i=0; i<desVel.dim1(); i++)
//			logString = logString+desVel[i][0]+"\t";
//		mQuadLogger->addEntry(LOG_ID_VEL_CMD, logString, LOG_FLAG_STATE_DES);
//
//		Array2D<double> velErr(3,1);
//		velErr[0][0] = curState[3][0]-(desState[3][0]+desVel[0][0]);
//		velErr[1][0] = curState[4][0]-(desState[4][0]+desVel[1][0]);
//		velErr[2][0] = curState[5][0]-(desState[5][0]+desVel[2][0]);
//
//		Array2D<double> accelCmd = -1.0*velGains*velErr;
//
//		accelCmd[0][0] = min(2.0, max(-2.0, accelCmd[0][0]));
//		accelCmd[1][0] = min(2.0, max(-2.0, accelCmd[1][0]));
//
//		accelCmd[2][0] += GRAVITY;
//
//		// Assume the PID integrated err holds the necessary
//		// acceleration offset
//		mMutex_data.lock();
//		accelCmd[2][0] -= mGainI[2][0]*mErrInt[2][0];
//		mMutex_data.unlock();
//
////Log::alert("--------------------------------------------------");
////Log::alert(String()+"target center:\t"+targetData->target->meanCenter.x+"\t"+targetData->target->meanCenter.y);
////printArray("moment:\t",moment);
////printArray("desMoment:\t",desMoment);
////printArray("desMoment2:\t",desMoment2);
////printArray("desVel:\t",desVel);
////printArray("accelCmd:\t",accelCmd);
//
//		mLastController = Controller::IBVS;
//		return accelCmd;
	}

	Array2D<double> TranslationController::calcControlIBVS2()
	{
		mMutex_target.lock();
		shared_ptr<ImageTranslationData> xlateData= mTargetTranslationData;
		mMutex_target.unlock();
		
		// Predict the target's current position based on kinematics
		vector<cv::Point2f> points = xlateData->goodPoints;
		double dt = xlateData->timestamp.getElapsedTimeNS()/1.0e9;
		double f = xlateData->imageTargetFind2Data->imageData->focalLength;
		Array2D<double> vel(3,1);
		double z;
		if(mObsvTranslational == NULL)
		{
			Log::alert("crap, TranslationController's mObsvTranslational is null");
			vel[0][0] = vel[1][0] = vel[2][0] = 0;
			z = 1;
		}
		else
		{
			Array2D<double> imgState = mObsvTranslational->estimateStateAtTime(xlateData->timestamp);
			z = imgState[2][0];
			vel[0][0] = imgState[3][0];
			vel[1][0] = imgState[4][0];
			vel[2][0] = imgState[5][0];
			vel.inject(mRotPhoneToCam*vel);
		}
		Array2D<double> Lv(2,3), delta(2,1);
		for(int i=0; i<points.size(); i++)
		{
			Lv[0][0] = -f; Lv[0][1] = 0;  Lv[0][2] = points[i].x;
			Lv[1][0] = 0;  Lv[1][1] = -f; Lv[1][2] = points[i].y;

			delta.inject(dt/z*matmult(Lv,vel));

			points[i].x += delta[0][0];
			points[i].y += delta[1][0];
		}

		// project onto unit sphere
		// Remeber that points is already in phone coords
		vector<Array2D<double>> spherePoints;
		Array2D<double> p(3,1), moment(3,1,0.0);
		for(int i=0; i<points.size(); i++)
		{
			p[0][0] = points[i].x;
			p[1][0] = points[i].y;
			p[2][0] = -f;

			// Get it on the unit sphere
			p.inject(1.0/norm2(p)*p);

			moment += p;
		}
		moment = 1.0/norm2(moment)*moment;
//		moment = mRotCamToPhone*moment;

		Array2D<double> desDir(3,1);
		desDir[0][0] = 0;
		desDir[1][0] = 0;
		desDir[2][0] = -1;
		Array2D<double> desMoment = desDir;

		mMutex_state.lock();
		Array2D<double> curState = mCurState.copy();
		Array2D<double> desState = mDesState.copy();
		mMutex_state.unlock();
		mMutex_gains.lock();
		Array2D<double> posGains = mIbvsPosGains;
		Array2D<double> velGains = mIbvsVelGains;
		mMutex_gains.unlock();

//		Array2D<double> visionErr = curState[2][0]*moment-desState[2][0]*desMoment;
		Array2D<double> visionErr = curState[2][0]*(moment-desMoment);

		Array2D<double> desVel = posGains*visionErr; // remember that * is element-wise

		// use real height for z vel
		desVel[2][0] = -mIbvsPosGains[2][0]*(curState[2][0]-desState[2][0]);

		String logString;
		for(int i=0; i<desVel.dim1(); i++)
			logString = logString+desVel[i][0]+"\t";
		mQuadLogger->addEntry(LOG_ID_VEL_CMD, logString, LOG_FLAG_STATE_DES);

		Array2D<double> velErr(3,1);
		velErr[0][0] = curState[3][0]-(desState[3][0]+desVel[0][0]);
		velErr[1][0] = curState[4][0]-(desState[4][0]+desVel[1][0]);
		velErr[2][0] = curState[5][0]-(desState[5][0]+desVel[2][0]);

		Array2D<double> accelCmd = -1.0*velGains*velErr;

		accelCmd[0][0] = min(2.0, max(-2.0, accelCmd[0][0]));
		accelCmd[1][0] = min(2.0, max(-2.0, accelCmd[1][0]));

		accelCmd[2][0] += GRAVITY;

		// Assume the PID integrated err holds the necessary
		// acceleration offset
		mMutex_data.lock();
		accelCmd[2][0] -= mGainI[2][0]*mErrInt[2][0];
		mMutex_data.unlock();

//Log::alert("--------------------------------------------------");
//Log::alert(String()+"target center:\t"+targetData->target->meanCenter.x+"\t"+targetData->target->meanCenter.y);
//printArray("moment:\t",moment);
//printArray("desMoment:\t",desMoment);
//printArray("desMoment2:\t",desMoment2);
//printArray("desVel:\t",desVel);
//printArray("accelCmd:\t",accelCmd);

		mLastController = Controller::IBVS;
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

	void TranslationController::onNewCommIbvsGains(const Collection<float> &posGains, const Collection<float> &velGains)
	{
		mMutex_gains.lock();
		for(int i=0; i<3; i++)
		{
			mIbvsPosGains[i][0] = posGains[i];
			mIbvsVelGains[i][0] = velGains[i];
		}
		printArray("IBVS pos gains updated:\t",mIbvsPosGains);
		printArray("IBVS vel gains updated:\t",mIbvsVelGains);
		mMutex_gains.unlock();
	}

	void TranslationController::onNewCommStateVicon(const Collection<float> &data)
	{
		mMutex_viconState.lock();
		if(mStateVicon.dim1() != data.size())
			mStateVicon = Array2D<double>(data.size(),1);

		for(int i=0; i<mStateVicon.dim1(); i++)
			mStateVicon[i][0] = data[i];
		mMutex_viconState.unlock();
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

	void TranslationController::onObserver_TranslationalImageProcessed(const shared_ptr<ImageTranslationData> &data)
	{
		mNewMeasAvailable = true;
		mMutex_target.lock();
		mTargetTranslationData = data;
		mMutex_target.unlock();
	}

	void TranslationController::onTargetFound(const shared_ptr<ImageTargetFindData> &data)
	{
		if(data->target != NULL)
		{
			mMutex_target.lock();
			mTargetData = data;
			mMutex_target.unlock();

			mNewMeasAvailable = true;
		}
	}

	void TranslationController::onTargetFound2(const shared_ptr<ImageTargetFind2Data> &data)
	{
		if(data == NULL || (data->repeatRegions.size() + data->newRegions.size()) == 0)
			return;

		mMutex_target.lock();
		mTarget2Data = data;
		mMutex_target.unlock();

		mNewMeasAvailable = true;
	}

} // namespace Quadrotor
} // namespace ICSL
