#include "Observer_Translational.h"
#include "TNT/jama_lu.h"
#include "TNT/jama_qr.h"

namespace ICSL{
namespace Quadrotor{
using namespace toadlet::egg;
using namespace ICSL::Constants;
	Observer_Translational::Observer_Translational() :
		mRotViconToPhone(3,3,0.0),
		mMeasCov(6,6,0.0),
		mPosMeasCov(3,3,0.0),
		mVelMeasCov(3,3,0.0),
		mDynCov(9,9,0.0),
		mErrCovKF(9,9,0.0),
		mStateKF(9,1,0.0),
		mLastViconPos(3,1,0.0),
		mLastCameraPos(3,1,0.0),
		mLastViconVel(3,1,0.0),
		mLastCameraVel(3,1,0.0),
		mViconCameraOffset(3,1,0.0),
		mAccelBiasReset(3,1,0.0)
	{
		mRunning = false;
		mDone = true;

		mLastPosReceiveTime.setTimeMS(0);
		
		mMeasCov[0][0] = mMeasCov[1][1] = mMeasCov[2][2] = 0.01*0.01;
		mMeasCov[3][3] = mMeasCov[4][4] = mMeasCov[5][5] = 0.3*0.3;
		mMeasCov[2][2] = 0.05*0.05;
		mMeasCov[5][5] = 0.5*0.5;
		mPosMeasCov = submat(mMeasCov,0,2,0,2);
		mVelMeasCov = submat(mMeasCov,3,5,3,5);
		mDynCov[0][0] = mDynCov[1][1] = mDynCov[2][2] = 0.1*0.1;
		mDynCov[3][3] = mDynCov[4][4] = mDynCov[5][5] = 0.5*0.5;
		mDynCov[6][6] = mDynCov[7][7] = mDynCov[8][8] = 0.5*0.5;
		mErrCovKF.inject(1e-4*createIdentity((double)9));

		mDoMeasUpdate = false;
		mNewViconPosAvailable = mNewCameraPosAvailable = false;

		mNewImageResultsReady = false;

//		mAccelData = NULL;
//		mNewAccelReady = false;

		mImageMatchData = NULL;

		mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
								 createRotMat(0,(double)PI));
		mRotPhoneToCam = transpose(mRotCamToPhone);

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mUseCameraPos = false;
		mUseViconPos = true;
		mHaveFirstCameraPos = false;

		mUseIbvs = false;
//		mUseIbvs = true;

		mAccelBiasReset[0][0] = 0.1;
		mAccelBiasReset[1][0] = 0;
		mAccelBiasReset[2][0] = -0.4;

		mStateKF[6][0] = mAccelBiasReset[0][0];
		mStateKF[7][0] = mAccelBiasReset[1][0];
		mStateKF[8][0] = mAccelBiasReset[2][0];

		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mStateBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mErrCovKFBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mViconPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mViconVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mCameraPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mCameraVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mOpticFlowVelBuffer));
//		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mHeightBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mMapHeightBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mMapVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mRawAccelDataBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double>>>*)(&mGravityDirDataBuffer));
	}

	Observer_Translational::~Observer_Translational()
	{
	}

	void Observer_Translational::shutdown()
	{
		Log::alert("------------------------- Observer_Translational shutdown started  --------------------------------------------------");
		mRunning = false;
		while(!mDone)
			System::msleep(10);

//		mNewAccelReady = false;
//		mAccelData = NULL;

		mImageMatchData = NULL;

		Log::alert("------------------------- Observer_Translational shutdown done");
	}

	void Observer_Translational::initialize()
	{
	}

	void Observer_Translational::run()
	{
		mDone = false;
		mRunning = true;

		Time lastUpdateTime;
		Array2D<double> measTemp(6,1);
		Array2D<double> accel(3,1,0.0), gravityDir(3,1,0.0);
		Array2D<double> pos(3,1),vel(3,1);
		double s1, s2, s3, c1, c2, c3;
		double dt;
		Array2D<double> errCov(12,1,0.0);

		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);

		Time loopTime;
		uint64 t;
		list<shared_ptr<IData>> events;
		float targetRate = 30; // hz
		float targetPeriodUS = 1.0f/targetRate*1.0e6;
		String logString;
		while(mRunning)
		{
			loopTime.setTime();

			mMutex_meas.lock();
			if(mHaveFirstCameraPos && mLastCameraPosTime.getElapsedTimeMS() > 1000)
			{
				mHaveFirstCameraPos = false;
				String s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_TARGET_LOST+"\t";
				mQuadLogger->addLine(s, LOG_FLAG_PC_UPDATES);
			}
			mMutex_meas.unlock();

			if(mUseIbvs && mHaveFirstCameraPos)
			{
				mUseViconPos = false;
				mUseCameraPos = true;
			}
			else
			{
				mUseViconPos = true;
				mUseCameraPos = false;
			}

shared_ptr<DataVector<double>> chadData(new DataVector<double>());
chadData->type = DATA_TYPE_VICON_POS;
chadData->data = Array2D<double>(3,1,0.0);
mNewEventsBuffer.push_back(chadData);

			// process new events
			events.clear();
			mMutex_meas.lock();
			mNewEventsBuffer.swap(events);
			mMutex_meas.unlock();
			events.sort(IData::timeSortPredicate);
			while(events.size() > 0)
			{
				lastUpdateTime.setTime( applyData(events.front()) );
				events.pop_front();
			}

			if(mGravityDirDataBuffer.size() >0 )
			{ mMutex_gravDir.lock(); gravityDir.inject(mGravityDirDataBuffer.back()->data); mMutex_gravDir.unlock(); }

			if(mRawAccelDataBuffer.size() > 0)
			{
				mMutex_accel.lock(); accel.inject(mRawAccelDataBuffer.back()->data); mMutex_accel.unlock();
				accel -= GRAVITY*gravityDir;
			}

			// time update since last processed event
			dt = lastUpdateTime.getElapsedTimeUS()/1.0e6;
			mMutex_data.lock();
			doTimeUpdateKF(accel, dt, mStateKF, mErrCovKF, mDynCov);
			mMutex_data.unlock();

			lastUpdateTime.setTime();

			// buffers
			mMutex_data.lock();
			shared_ptr<DataVector<double>> stateData = shared_ptr<DataVector<double>>(new DataVector<double>());
			stateData->type = DATA_TYPE_STATE_TRAN;
			stateData->data = mStateKF.copy();
			mStateBuffer.push_back(stateData);

			shared_ptr<DataVector<double>> errCovData = shared_ptr<DataVector<double>>(new DataVector<double>());
			errCovData->type = DATA_TYPE_KF_ERR_COV;
			errCovData->data = mErrCovKF.copy();
			mErrCovKFBuffer.push_back(errCovData);

			for(int i=0; i<mDataBuffers.size(); i++)
				while(mDataBuffers[i]->size() > 0 && mDataBuffers[i]->front()->timestamp.getElapsedTimeMS() > 1e3)
					mDataBuffers[i]->pop_front();

			for(int i=0; i<3; i++)
				pos[i][0] = mStateKF[i][0];
			for(int i=0; i<3; i++)
				vel[i][0] = mStateKF[i+3][0];
			for(int i=0; i<3; i++)
			{
				errCov[i][0] = mErrCovKF[i][i];
				errCov[i+3][0] = mErrCovKF[i][i+3];
				errCov[i+6][0] = mErrCovKF[i+3][i+3];
			}
			mMutex_data.unlock();

			{
				logString = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_KALMAN_ERR_COV+"\t";
				for(int i=0; i<errCov.dim1(); i++)
					logString= logString+errCov[i][0]+"\t";
				mMutex_logger.lock();
				mQuadLogger->addLine(logString,LOG_FLAG_STATE);
				mMutex_logger.unlock();
			}

{
logString = "state: ";
for(int i=0; i<mStateKF.dim1(); i++)
	logString = logString+mStateKF[i][0]+"\t";
Log::alert(logString);
}

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onObserver_TranslationalUpdated(pos, vel);
	
			t = loopTime.getElapsedTimeUS();
			if(t < targetPeriodUS)
				System::usleep(targetPeriodUS-t); // try to maintain a (roughly) constant update rate
		}

		mDone = true;
	}

	void Observer_Translational::doTimeUpdateKF(Array2D<double> const &accel, double const &dt, Array2D<double> &state, Array2D<double> &errCov, Array2D<double> const &dynCov)
	{
		for(int i=0; i<3; i++)
			state[i][0] += dt*state[i+3][0];
		for(int i=3; i<6; i++)
			state[i][0] += dt*(accel[i-3][0]-state[i+3][0]);
		// states 6-8 (bias states) are assumed constant

		// S = A*S*A'+V*dt*dt
		double dtSq = dt*dt;
		for(int i=0; i<3; i++)
		{
			errCov[i][i] += 2*errCov[i][i+3]*dt+(errCov[i+3][i+3]+dynCov[i][i])*dtSq;
			errCov[i][i+3] += (errCov[i+3][i+3]-errCov[i][i+6])*dt-errCov[i+3][i+6]*dtSq;
			errCov[i][i+6] += errCov[i+3][i+6]*dt;
			errCov[i+3][i] = errCov[i][i+3];
			errCov[i+6][i] = errCov[i][i+6];

			errCov[i+3][i+3] += -2*errCov[i+3][i+6]*dt+(errCov[i+6][i+6]+dynCov[i+3][i+3])*dtSq;
			errCov[i+3][i+6] -= errCov[i+6][i+6]*dt;
			errCov[i+6][i+3] = errCov[i+3][i+6];

			errCov[i+6][i+6] += dynCov[i+6][i+6]*dtSq;
		}
	}

	void Observer_Translational::doMeasUpdateKF_posOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
	{
		// K = S*C'*inv(C*S*C'+W)
		Array2D<double> gainKF(9,3,0.0);
		double den;
		for(int i=0; i<3; i++)
		{
			den = errCov[i][i]+measCov[i][i];
			gainKF[i][i] = errCov[i][i]/den;
			gainKF[i+3][i] = errCov[i][i+3]/den;
			gainKF[i+6][i] = errCov[i][i+6]/den;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err= meas-submat(state,0,2,0,0);
		for(int i=0; i<3; i++)
		{
			state[i][0] += gainKF[i][i]*err[i][0];
			state[i+3][0] += gainKF[i+3][i]*err[i][0];
			state[i+6][0] += gainKF[i+6][i]*err[i][0];
		}

		// S = (I-KC) S
		for(int i=0; i<3; i++)
		{
			errCov[i][i] -= gainKF[i][i]*errCov[i][i];
			errCov[i][i+3] -= gainKF[i][i]*errCov[i][i+3];
			errCov[i][i+6] -= gainKF[i][i]*errCov[i][i+6];
			errCov[i+3][i] = errCov[i][i+3];
			errCov[i+6][i] = errCov[i][i+6];

			errCov[i+3][i+3] -= gainKF[i+3][i]*errCov[i][i+3];
			errCov[i+3][i+6] -= gainKF[i+3][i]*errCov[i][i+6];
			errCov[i+6][i+3] = errCov[i+6][i+3];

			errCov[i+6][i+6] -= gainKF[i+6][i]*errCov[i][i+6];
		}
	}

	void Observer_Translational::doMeasUpdateKF_velOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
	{
		if(norm2(meas) > 10)
			return; // screwy measuremnt

		// K = S*C'*inv(C*S*C'+W)
		Array2D<double> gainKF(9,3,0.0);
		for(int i=0; i<3; i++)
		{
			double den = errCov[i+3][i+3]+measCov[i][i];
			gainKF[i][i] = errCov[i][i+3]/den;
			gainKF[i+3][i] = errCov[i+3][i+3]/den;
			gainKF[i+6][i] = errCov[i+3][i+6]/den;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-submat(state,3,5,0,0);
		for(int i=0; i<3; i++)
		{
			state[i][0] += gainKF[i][i]*err[i][0];
			state[i+3][0] += gainKF[i+3][i]*err[i][0];
			state[i+6][0] += gainKF[i+6][i]*err[i][0];
		}

		// S = (I-KC) S
		for(int i=0; i<3; i++)
		{
			errCov[i][i] -= gainKF[i][i]*errCov[i][i+3];
			errCov[i][i+3] -= gainKF[i][i]*errCov[i+3][i+3];
			errCov[i][i+6] -= gainKF[i][i]*errCov[i+3][i+6];
			errCov[i+3][i] = errCov[i][i+3];
			errCov[i+6][i] = errCov[i][i+6];

			errCov[i+3][i+3] -= gainKF[i+3][i]*errCov[i+3][i+3];
			errCov[i+3][i+6] -= gainKF[i+3][i]*errCov[i+3][i+6];
			errCov[i+6][i+3] = errCov[i+6][i+3];

			errCov[i+6][i+6] -= gainKF[i+6][i]*errCov[i+3][i+6];
		}

	}

	void Observer_Translational::doMeasUpdateKF_heightOnly(double const &meas, double const &measCov, Array2D<double> &state, Array2D<double> &errCov)
	{
		// K = S*C'*inv(C*S*C'+W)
		Array2D<double> gainKF(9,1,0.0);
		double den = errCov[2][2]+measCov;
		gainKF[2][0] = errCov[2][2]/den;
		gainKF[5][0] = errCov[2][5]/den;
		gainKF[8][0] = errCov[2][8]/den;

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		double err = meas-state[2][0];
		state[2][0] += gainKF[2][0]*err;
		state[5][0] += gainKF[5][0]*err;
		state[8][0] += gainKF[8][0]*err;

		// S = (I-KC) S
		errCov[2][2] -= gainKF[2][0]*errCov[2][2];
		errCov[2][5] -= gainKF[2][0]*errCov[2][5];
		errCov[2][8] -= gainKF[2][0]*errCov[2][8];
		errCov[5][2] = errCov[2][5];
		errCov[8][2] = errCov[2][8];

		errCov[5][5] -= gainKF[5][0]*errCov[2][5];
		errCov[5][8] -= gainKF[5][0]*errCov[2][8];
		errCov[8][5] = errCov[5][8];

		errCov[8][8] -= gainKF[8][0]*errCov[2][8];
	}

	void Observer_Translational::onObserver_AngularUpdated(shared_ptr<DataVector<double>> attData, shared_ptr<DataVector<double>> angularVelData)
	{
		Array2D<double> att = attData->data;
		double s1 = sin(att[2][0]); double c1 = cos(att[2][0]);
		double s2 = sin(att[1][0]); double c2 = cos(att[1][0]);
		double s3 = sin(att[0][0]); double c3 = cos(att[0][0]);

		// third column of orientation matrix, i.e. R*e3
//		Array2D<double> r(3,1);
//		r[0][0] = s1*s3+c1*c3*s2;
//		r[1][0] = c3*s1*s2-c1*s3;
//		r[2][0] = c2*c3;

		// gravity vector orientation, i.e. R'*e3
		Array2D<double> g(3,1);
		g[0][0] = -s2;
		g[1][0] = c2*s3;
		g[2][0] = c2*c3;

		shared_ptr<DataVector<double>> dirData(new DataVector<double>());
		dirData->type = DATA_TYPE_GRAVITY_DIR;
		dirData->timestamp = attData->timestamp;
		dirData->data = -1.0*g.copy();

		mMutex_meas.lock();
		mNewEventsBuffer.push_back(dirData);
		mMutex_meas.unlock();
	}

	void Observer_Translational::onNewCommStateVicon(Collection<float> const &data)
	{
		Time now;
		Array2D<double> pos(3,1);
		for(int i=0; i<3; i++)
			pos[i][0] = data[i+6];
		pos = matmult(mRotViconToPhone, pos);
		mMutex_meas.lock();
		Array2D<double> vel(3,1,0.0);
		double dt = mLastViconPosTime.getElapsedTimeUS()/1.0e6;
		mLastViconPosTime.setTime(now);
		if(dt < 0.2)
		{
			// This velocity ``measurement'' is very noisy, but it helps to correct some
			// of the bias error that occurs when the attitude estimate is wrong
			if(dt > 1.0e-3) // reduce the effect of noise
				vel.inject(1.0/dt*(pos-mLastViconPos));
			else
				for(int i=0; i<3; i++)
					vel[i][0] = 0;
		}

		shared_ptr<DataVector<double>> posData(new DataVector<double>() );
		posData->type = DATA_TYPE_VICON_POS;
		posData->timestamp.setTime(now);
		posData->data = pos.copy();

//		shared_ptr<DataVector<double>> velData(new DataVector<double>() );
//		velData->type = DATA_TYPE_VICON_VEL;
//		velData->timestamp.setTime(now);
//		velData->data = vel.copy();

//		shared_ptr<Data<double>> heightData = shared_ptr<Data<double>>(new Data<double>());
//		heightData->type = DATA_TYPE_VICON_HEIGHT;
//		heightData->timestamp.setTime(now);
//		heightData->data = pos[2][0];

		mMutex_data.lock();
		mLastViconPos.inject(pos);
		mLastViconVel.inject(vel);

		mNewEventsBuffer.push_back(posData);
//		mNewEventsBuffer.push_back(velData);
//		mHeightBuffer.push_back(heightData);

		mLastPosReceiveTime.setTime(now);
		mMutex_data.unlock();
		mMutex_meas.unlock();


		{
			String s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_RECEIVE_VICON+"\t";
			for(int i=0; i<data.size(); i++)
				s = s+data[i]+"\t";
			mMutex_logger.lock();
			mQuadLogger->addLine(s, LOG_FLAG_STATE);
			mMutex_logger.unlock();
		}

		mNewViconPosAvailable = true;
	}

	void Observer_Translational::onNewCommKalmanMeasVar(Collection<float> const &var)
	{
		mMutex_data.lock();
		for(int i=0; i<6; i++)
			mMeasCov[i][i] = var[i];
		String s = "Meas var update -- diag(mMeasCov): \t";
		for(int i=0; i<mMeasCov.dim1(); i++)
			s = s+mMeasCov[i][i]+"\t";
		mPosMeasCov = submat(mMeasCov,0,2,0,2);
		mVelMeasCov = submat(mMeasCov,3,5,3,5);
		mMutex_data.unlock();
		Log::alert(s);
	}

	void Observer_Translational::onNewCommKalmanDynVar(Collection<float> const &var)
	{
		mMutex_data.lock();
		for(int i=0; i<6; i++)
			mDynCov[i][i] = var[i];
		String s = "Dyn var update -- diag(mDynCov): \t";
		for(int i=0; i<mDynCov.dim1(); i++)
			s = s+mDynCov[i][i]+"\t";
		mMutex_data.unlock();
		Log::alert(s);
	}

//	void Observer_Translational::onAttitudeThrustControllerCmdsSent(double const cmds[4])
//	{
//		shared_ptr<DataVector<double>> data(new DataVector<double>());
//		data->type = DATA_TYPE_MOTOR_CMDS;
//		data->data = Array2D<double>(4,1,0.0);
//		for(int i=0; i<4; i++)
//			data->data[i][0] = cmds[i];
//
//		mMutex_cmds.lock();
//		mMotorCmdsBuffer.push_back(data);
//		mMutex_cmds.unlock();
//	}

	void Observer_Translational::onNewSensorUpdate(shared_ptr<IData> const &data)
	{
		switch(data->type)
		{
			case DATA_TYPE_ACCEL:
//				mMutex_accel.lock();
//				mAccelData = static_pointer_cast<DataVector<double>>(data);
//				mMutex_accel.unlock();
//				mNewAccelReady = true;
				
				shared_ptr<DataVector<double>> accelData(new DataVector<double>());
				accelData->type = DATA_TYPE_RAW_ACCEL;
				accelData->timestamp = data->timestamp;
				accelData->data = static_pointer_cast<DataVector<double>>(data)->dataCalibrated.copy();

				mMutex_meas.lock();
				mNewEventsBuffer.push_back(accelData);
				mMutex_meas.unlock();
				break;
		}
	}

	void Observer_Translational::onVelocityEstimator_newEstimate(shared_ptr<DataVector<double>> const &velData, shared_ptr<Data<double>> const &heightData)
	{
//		mMutex_meas.lock();
//		mNewEventsBuffer.push_back(velData);
//		mNewEventsBuffer.push_back(heightData);
//		mMutex_meas.unlock();
	}

	void Observer_Translational::onTargetFound(shared_ptr<ImageTargetFindData> const &data)
	{
		if(data->target == NULL)
			return;

		if(mUseIbvs)
		{
			if(!mHaveFirstCameraPos)
			{
				String s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_TARGET_ACQUIRED+"\t";
				mQuadLogger->addLine(s, LOG_FLAG_PC_UPDATES);
			}
			mHaveFirstCameraPos = true;
		}
		mLastCameraPosTime.setTime();

		// assuming 320x240 images
		double f = data->imageData->focalLength;
		double cx = data->imageData->centerX;
		double cy = data->imageData->centerY;

		// rotation compensation
		Array2D<double> att = data->imageData->att.copy();
//		Array2D<double> R = createRotMat_ZYX(att[2][0], att[1][0], att[0][0]);
		Array2D<double> R = createIdentity(3.0);
//		Array2D<double> R_T = transpose(R);

		Array2D<double> p(3,1);
		p[0][0] = -(data->target->meanCenter.x - cx);
		p[1][0] = -(data->target->meanCenter.y - cy);
		p[2][0] = -f;
		p = matmult(R, matmult(mRotCamToPhone, p));

		// Now estimate the pos
		double avgLength = 0;
		for(int i=0; i<data->target->squareData[0]->lineLengths.size(); i++)
			avgLength += data->target->squareData[0]->lineLengths[i];
		avgLength /= data->target->squareData[0]->lineLengths.size();

		
		double nomLength =  0.21;
		Array2D<double> pos(3,1);
		pos[2][0] = nomLength/avgLength*f;
		pos[0][0] = p[0][0]/f*abs(pos[2][0]);
		pos[1][0] = p[1][0]/f*abs(pos[2][0]);

		Array2D<double> viconOffset(3,1);
		viconOffset[0][0] = 0.750;
		viconOffset[1][0] = 0.691;
		viconOffset[2][0] = 0.087;

		pos = pos+viconOffset;

		shared_ptr<DataVector<double>> posData(new DataVector<double>());
		posData->type = DATA_TYPE_CAMERA_POS;
		posData->timestamp.setTime(data->imageData->timestamp);
		posData->data = pos.copy();

		mMutex_meas.lock();
		mNewEventsBuffer.push_back(posData);
		mMutex_meas.unlock();

		String str = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_TARGET_ESTIMATED_POS+"\t";
		for(int i=0; i<pos.dim1(); i++)
			str = str+pos[i][0]+"\t";
		mMutex_logger.lock();
		mQuadLogger->addLine(str, LOG_FLAG_CAM_RESULTS);
		mMutex_logger.unlock();
	}

	void Observer_Translational::onNewCommUseIbvs(bool useIbvs)
	{
		mUseIbvs = useIbvs;
		String s;
		if(useIbvs)
			s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_IBVS_ENABLED+"\t";
		else
		{
			s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_IBVS_DISABLED+"\t";
			mHaveFirstCameraPos = false;
		}
		mMutex_logger.lock();
		mQuadLogger->addLine(s, LOG_FLAG_PC_UPDATES);
		mMutex_logger.unlock();
	}

	
	Time Observer_Translational::applyData(shared_ptr<IData> const &data)
	{
		Time dataTime = data->timestamp;

		mMutex_data.lock();
		// apply data at the correct point in time
		mStateKF.inject( IData::interpolate(dataTime, mStateBuffer));
		mErrCovKF.inject( IData::interpolate(dataTime, mErrCovKFBuffer));
		switch(data->type)
		{
			case DATA_TYPE_RAW_ACCEL:
				mRawAccelDataBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
				break;
			case DATA_TYPE_GRAVITY_DIR:
				mGravityDirDataBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
				break;
			case DATA_TYPE_VICON_POS:
				mViconPosBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
				if(mUseViconPos)
					doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<double>>(data)->data, mPosMeasCov, mStateKF, mErrCovKF);
				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_CAMERA_POS:
				mCameraPosBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
				if(mUseCameraPos)
					doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<double>>(data)->data, mPosMeasCov, mStateKF, mErrCovKF);
				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_VICON_VEL:
				mViconVelBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
//				if(mUseViconPos)
//					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double>>(data)->data, 100*mVelMeasCov, mStateKF, mErrCovKF);
//				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_CAMERA_VEL:
				mCameraVelBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
//				if(mUseCameraPos)
//					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double>>(data)->data, 100*mVelMeasCov, mStateKF, mErrCovKF);
//				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_OPTIC_FLOW_VEL:
				mOpticFlowVelBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
				doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double>>(data)->data, mVelMeasCov, mStateKF, mErrCovKF);
				break;
//			case DATA_TYPE_THRUST_DIR:
//				mThrustDirBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
				break;
			case DATA_TYPE_MAP_VEL:
				mMapVelBuffer.push_back(static_pointer_cast<DataVector<double>>(data));
				doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double>>(data)->data, mVelMeasCov, mStateKF, mErrCovKF);
				break;
			case DATA_TYPE_MAP_HEIGHT:
				{
				mMapHeightBuffer.push_back(static_pointer_cast<Data<double>>(data));
double mHeightMeasCov = 0.1*0.1;
				doMeasUpdateKF_heightOnly(static_pointer_cast<Data<double>>(data)->data, mHeightMeasCov, mStateKF, mErrCovKF);
				}
				break;
			default:
				Log::alert(String()+"Observer_Translational::applyData() --> Unknown data type: "+data->type);
				mMutex_data.unlock();
				return dataTime;
		}

		if(mRawAccelDataBuffer.size() == 0 || mGravityDirDataBuffer.size() == 0)
		{
			mMutex_data.unlock();
			return dataTime;
		}

		// now we have to rebuild the state and errcov history
		list<shared_ptr<DataVector<double>>>::iterator posIter, velIter, opticFlowVelIter;
		list<shared_ptr<DataVector<double>>>::iterator rawAccelIter, gravDirIter;
		list<shared_ptr<DataVector<double>>> *posBuffer, *velBuffer;
		if(mUseViconPos)
		{
			posBuffer = &mViconPosBuffer;
//			velBuffer = &mViconVelBuffer;
		}
		else
		{
			posBuffer = &mCameraPosBuffer;
//			velBuffer = &mCameraVelBuffer;
		}
		velBuffer = &mMapVelBuffer;

		// These iters point to the last data point with iter->timestamp < dataTime 
		posIter = IData::findIndexReverse(dataTime, *posBuffer);
		velIter = IData::findIndexReverse(dataTime, *velBuffer);
		rawAccelIter = IData::findIndexReverse(dataTime, mRawAccelDataBuffer);
		gravDirIter = IData::findIndexReverse(dataTime, mGravityDirDataBuffer);
		opticFlowVelIter = IData::findIndexReverse(dataTime, mOpticFlowVelBuffer); 
		
		// construct a sorted list of all events
		list<shared_ptr<IData>> events;
		if(posIter != posBuffer->end()) 
			events.insert(events.end(), ++posIter, posBuffer->end());
		if(velIter != velBuffer->end())
			events.insert(events.end(), ++velIter, velBuffer->end());
		if(rawAccelIter != mRawAccelDataBuffer.end())
			events.insert(events.end(), ++rawAccelIter, mRawAccelDataBuffer.end());
		if(gravDirIter != mGravityDirDataBuffer.end())
			events.insert(events.end(), ++gravDirIter, mGravityDirDataBuffer.end());
		if(opticFlowVelIter != mOpticFlowVelBuffer.end())
			events.insert(events.end(), ++opticFlowVelIter, mOpticFlowVelBuffer.end());
		events.sort(IData::timeSortPredicate);

		if(events.size() == 0)
		{
			mMutex_data.unlock();
			return dataTime;
		}

		// clear out state and errcov information that we are about to replace
		IData::truncate(dataTime, mStateBuffer);
		IData::truncate(dataTime, mErrCovKFBuffer);

		// get accel vector at dataTime
		Array2D<double> accel(3,1), accelRaw(3,1), gravDir(3,1);
		if(gravDirIter != mGravityDirDataBuffer.end())
			gravDir.inject( (*gravDirIter)->data );
		else
			gravDir.inject( mGravityDirDataBuffer.back()->data);

		if(rawAccelIter != mRawAccelDataBuffer.end())
			accelRaw.inject((*rawAccelIter)->data);
		else
			accelRaw.inject(mRawAccelDataBuffer.back()->data);
		accel.inject(accelRaw - GRAVITY*gravDir);

		// apply events in order
		list<shared_ptr<IData>>::const_iterator eventIter = events.begin();
		Time lastUpdateTime(dataTime);
		double dt;
		shared_ptr<DataVector<double>> stateData, errCovData;
		while(eventIter != events.end())
		{
			dt = Time::calcDiffNS(lastUpdateTime, (*eventIter)->timestamp)/1.0e9;
			doTimeUpdateKF(accel, dt, mStateKF, mErrCovKF, mDynCov);

			switch((*eventIter)->type)
			{
				case DATA_TYPE_RAW_ACCEL:
					accelRaw.inject( static_pointer_cast<DataVector<double>>(*eventIter)->data);
					accel.inject(accelRaw+GRAVITY*gravDir);
					break;
				case DATA_TYPE_GRAVITY_DIR:
					gravDir.inject( static_pointer_cast<DataVector<double>>(*eventIter)->data );
					accel.inject(accelRaw+GRAVITY*gravDir);
					break;
				case DATA_TYPE_CAMERA_POS:
				case DATA_TYPE_VICON_POS:
					doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<double>>(*eventIter)->data, mPosMeasCov, mStateKF, mErrCovKF);
					break;
				case DATA_TYPE_VICON_VEL:
				case DATA_TYPE_CAMERA_VEL:
//					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double>>(*eventIter)->data, 100*mVelMeasCov, mStateKF, mErrCovKF);
					break;
				case DATA_TYPE_MAP_VEL:
					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double>>(*eventIter)->data, mVelMeasCov, mStateKF, mErrCovKF);
					break;
				case DATA_TYPE_MAP_HEIGHT:
					break;
				case DATA_TYPE_OPTIC_FLOW_VEL:
					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double>>(*eventIter)->data, mVelMeasCov, mStateKF, mErrCovKF);
					break;
				default:
					Log::alert(String()+"Observer_Translational::applyData() --> Unknown data type: "+data->type);
			}

			stateData = shared_ptr<DataVector<double>>(new DataVector<double>());
			stateData->type = DATA_TYPE_STATE_TRAN;
			stateData->timestamp.setTime( (*eventIter)->timestamp);
			stateData->data = mStateKF.copy();
			mStateBuffer.push_back(stateData);

			errCovData = shared_ptr<DataVector<double>>(new DataVector<double>());
			errCovData->type = DATA_TYPE_KF_ERR_COV;
			errCovData->timestamp.setTime( (*eventIter)->timestamp);
			errCovData->data = mErrCovKF.copy();
			mErrCovKFBuffer.push_back(errCovData);

			lastUpdateTime.setTime((*eventIter)->timestamp);
			eventIter++;
		}

		mMutex_data.unlock();

		return lastUpdateTime;
	}

	void Observer_Translational::setStartTime(Time t)
	{
		mMutex_data.lock(); mMutex_att.lock(); mMutex_meas.lock(); mMutex_cmds.lock(); mMutex_phoneTempData.lock();
		mStartTime.setTime(t);
		// for now I'll be lazy and just clearn everything out, assuming 
		// this only happens when nothing interesting is happening anyhow
		for(int i=0; i<mDataBuffers.size(); i++)
			mDataBuffers[i]->clear();

		mNewEventsBuffer.clear();

		mMutex_data.unlock(); mMutex_att.unlock(); mMutex_meas.unlock(); mMutex_cmds.unlock(); mMutex_phoneTempData.unlock();
	}

	Array2D<double> Observer_Translational::estimateStateAtTime(Time const &t)
	{
		mMutex_data.lock();
		Array2D<double> state = IData::interpolate(t, mStateBuffer);
		mMutex_data.unlock();

		return state;
	}

	Array2D<double> Observer_Translational::estimateErrCovAtTime(Time const &t)
	{
		Array2D<double> errCov;
		if(t < mErrCovKFBuffer.front()->timestamp)
			errCov = 10*createIdentity((double)6);
		else
		{
			mMutex_data.lock();
			errCov = IData::interpolate(t, mErrCovKFBuffer);
			mMutex_data.unlock();
		}

		return errCov;
	}
} // namespace Quadrotor
} // namespace ICSL
