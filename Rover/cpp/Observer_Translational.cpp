#include "Observer_Translational.h"
#include "TNT/jama_lu.h"
#include "TNT/jama_qr.h"

namespace ICSL{
namespace Quadrotor{
using namespace ICSL::Constants;
using namespace TNT;
using std::isnan;

	Observer_Translational::Observer_Translational() :
		mRotViconToPhone(3,3,0.0),
		mMeasCov(6,6,0.0),
		mPosMeasCov(3,3,0.0),
		mVelMeasCov(3,3,0.0),
		mDynCov(9,9,0.0),
		mErrCovKF(9,9,0.0),
		mStateKF(9,1,0.0),
		mViconCameraOffset(3,1,0.0)
	{
		mRunning = false;
		mDone = true;

		mMeasCov[0][0] = mMeasCov[1][1] = mMeasCov[2][2] = 0.01*0.01;
		mMeasCov[3][3] = mMeasCov[4][4] = mMeasCov[5][5] = 0.3*0.3;
		mMeasCov[2][2] = 0.05*0.05;
		mMeasCov[5][5] = 0.5*0.5;
		mPosMeasCov = submat(mMeasCov,0,2,0,2);
		mVelMeasCov = submat(mMeasCov,3,5,3,5);
		mDynCov[0][0] = mDynCov[1][1] = mDynCov[2][2] = 0.1*0.1*0;
		mDynCov[3][3] = mDynCov[4][4] = mDynCov[5][5] = 0.5*0.5*0;
		mDynCov[6][6] = mDynCov[7][7] = mDynCov[8][8] = 0;
		mErrCovKF.inject(1e-6*createIdentity((double)9));

		mDoMeasUpdate = false;
		mNewViconPosAvailable = mNewCameraPosAvailable = false;

		mRotCamToPhone = SO3( matmult(createRotMat(2,-0.5*(double)PI),
								      createRotMat(0,(double)PI)) );
		mRotPhoneToCam = mRotCamToPhone.inv();

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mUseCameraPos = false;
		mUseViconPos = true;
		mHaveFirstCameraPos = false;

		mUseIbvs = false;

		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mStateBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mErrCovKFBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mViconPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mViconVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mCameraPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mCameraVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mMapHeightBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mMapVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mRawAccelDataBuffer));
		mDataBuffers.push_back( (list<shared_ptr<IData>>*)(&mHeightDataBuffer));
		
		mHaveFirstVicon = false;

		mMAPHeightMeasCov = 0.1*0.1;

		mTargetNominalLength = 0.210;
		mViconCameraOffset[0][0] = 0;
		mViconCameraOffset[1][0] = 0;
		mViconCameraOffset[2][0] = 0;
		mIsViconCameraOffsetSet = false;

		mObsvAngular = NULL;
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
		Array2D<double> accel(3,1,0.0), gravityDir(3,1,0.0);
		gravityDir[0][0] = 0;
		gravityDir[1][0] = 0;
		gravityDir[2][0] = -1;
		Array2D<double> pos(3,1),vel(3,1);
		Array2D<double> errCov(18,1,0.0);
		double dt;

		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);

		Time loopTime;
		toadlet::uint64 t;
		list<shared_ptr<IData>> events;
		float targetRate = 100; // hz
		float targetPeriodUS = 1.0f/targetRate*1.0e6;
		String logString;
		SO3 att;
		while(mRunning)
		{
			loopTime.setTime();

			mMutex_posTime.lock();
			if(mHaveFirstCameraPos && mLastCameraPosTime.getElapsedTimeMS() > 1000)
			{
				mHaveFirstCameraPos = false;
				logString;
				mQuadLogger->addEntry(Time(), LOG_ID_TARGET_LOST, logString, LOG_FLAG_PC_UPDATES);
				mCameraVelBuffer.clear();
			}
			mMutex_posTime.unlock();

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

			// process new events
			events.clear();
			mMutex_events.lock();
			mNewEventsBuffer.swap(events);
			mMutex_events.unlock();

			mMutex_att.lock();
			att = mCurAtt;
			mMutex_att.unlock();

			events.sort(IData::timeSortPredicate);
			if(events.size() > 0 && events.front()->timestamp > lastUpdateTime)
			{
				// need to advance time to the first event
				if(mRawAccelDataBuffer.size() > 0)
				{
					mMutex_accel.lock(); accel.inject(mRawAccelDataBuffer.back()->data); mMutex_accel.unlock();
					accel += GRAVITY*gravityDir;
				}
				dt = Time::calcDiffNS(lastUpdateTime, events.front()->timestamp)/1.0e9;
				mMutex_kfData.lock();
				doTimeUpdateKF(accel, dt, mStateKF, mErrCovKF, mDynCov, att);
				lastUpdateTime.setTime(events.front()->timestamp);

				mMutex_kfData.unlock();
			}

			// now process the events
			if(events.size() > 0)
				lastUpdateTime.setTime( applyData(events) );

			if(mRawAccelDataBuffer.size() > 0)
			{
				mMutex_accel.lock(); accel.inject(mRawAccelDataBuffer.back()->data); mMutex_accel.unlock();
				accel += GRAVITY*gravityDir;
			}

			// time update since last processed event
			dt = lastUpdateTime.getElapsedTimeNS()/1.0e9;
			lastUpdateTime.setTime();
			mMutex_kfData.lock();
			doTimeUpdateKF(accel, dt, mStateKF, mErrCovKF, mDynCov, att);
			mMutex_kfData.unlock();

			// special handling if we aren't getting any position updates
			if( ((!mUseIbvs || !mHaveFirstCameraPos) && mLastViconPosTime.getElapsedTimeMS() > 1e3) )
			{
				mMutex_kfData.lock();
				for(int i=0; i<6; i++)
					mStateKF[i][0] = 0;
				mStateKF[6][0] = 0;
				mStateKF[7][0] = 0;
				mStateKF[8][0] = 0;

//				mErrCovKF.inject(0.01*mDynCov);
				mErrCovKF.inject(1e-6*createIdentity((double)9));
				mMutex_kfData.unlock();
			}

			// buffers
			shared_ptr<DataVector<double>> stateData = shared_ptr<DataVector<double>>(new DataVector<double>());
			stateData->type = DATA_TYPE_STATE_TRAN;
			stateData->timestamp.setTime(lastUpdateTime);
			shared_ptr<DataVector<double>> errCovData = shared_ptr<DataVector<double>>(new DataVector<double>());
			errCovData->type = DATA_TYPE_KF_ERR_COV;
			errCovData->timestamp.setTime(lastUpdateTime);

			mMutex_kfData.lock();
			stateData->data = mStateKF.copy();
			mStateBuffer.push_back(stateData);

			errCovData->data = mErrCovKF.copy();
			mErrCovKFBuffer.push_back(errCovData);

			for(int i=0; i<3; i++)
				pos[i][0] = mStateKF[i][0];
			for(int i=0; i<3; i++)
				vel[i][0] = mStateKF[i+3][0];
//			for(int i=0; i<3; i++)
//			{
//				errCov[i][0] = mErrCovKF[i][i];
//				errCov[i+3][0] = mErrCovKF[i][i+3];
//				errCov[i+6][0] = mErrCovKF[i][i+6];
//				errCov[i+9][0] = mErrCovKF[i+3][i+3];
//				errCov[i+12][0] = mErrCovKF[i+3][i+6];
//				errCov[i+15][0] = mErrCovKF[i+6][i+6];
//			}



			for(int i=0; i<mDataBuffers.size(); i++)
				while(mDataBuffers[i]->size() > 0 && mDataBuffers[i]->front()->timestamp.getElapsedTimeMS() > 0.5e3)
				{
//					if(mDataBuffers[i]->front()->type == DATA_TYPE_STATE_TRAN)
//					{
//						Array2D<double> v = static_pointer_cast<DataVector<double>>(mDataBuffers[i]->front())->data.copy();
//						logString = String();
//						for(int i=0; i<mStateKF.dim1(); i++)
//							logString = logString+v[i][0]+"\t";
//						mQuadLogger->addEntry(mDataBuffers[i]->front()->timestamp, LOG_ID_CUR_TRANS_STATE, logString,LOG_FLAG_STATE);
//					}

//					logString = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_KALMAN_ERR_COV+"\t";
//					for(int i=0; i<errCov.dim1(); i++)
//						logString= logString+errCov[i][0]+"\t";
//					mQuadLogger->addEntry(logString+"\n",LOG_FLAG_STATE);
				mDataBuffers[i]->pop_front();
				}

			{
				logString = String();
				for(int i=0; i<mStateKF.dim1(); i++)
					logString = logString+mStateKF[i][0]+"\t";
				mQuadLogger->addEntry(Time(),LOG_ID_CUR_TRANS_STATE, logString,LOG_FLAG_STATE);
			}

			mMutex_kfData.unlock();

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onObserver_TranslationalUpdated(pos, vel);

			t = loopTime.getElapsedTimeUS();
			if(t < targetPeriodUS)
				System::usleep(targetPeriodUS-t); // try to maintain a (roughly) constant update rate
		}

		mDone = true;
	}

	void Observer_Translational::doTimeUpdateKF(const Array2D<double> &accel,
												double dt,
												Array2D<double> &state,
												Array2D<double> &errCov,
												const Array2D<double> &dynCov,
												const SO3 &att)
	{
		if(dt > 1)
		{
			Log::alert(String()+"Translation observer time update large dt: " +dt);
			return;
		}

		if(dt == 0)
			return;

		if(dt < 0)
		{
			Log::alert(String()+"Why is dt < 0?  -- " + dt);
			return;
		}

		Array2D<double> bias = att*submat(state,6,8,0,0);
		
		for(int i=0; i<3; i++)
			state[i][0] += dt*state[i+3][0] + 0.5*dt*dt*(accel[i][0]-bias[i][0]);
		for(int i=3; i<6; i++)
			state[i][0] += dt*(accel[i-3][0]-bias[i-3][0]);
		// states 6-8 (bias states) are assumed constant

		// A = [I dt*I 	-0.5*dt^2*I;
		// 		0 I		-dt*I;
		// 		0 0		I;
		// S = A*S*A'+G*V*G'
		double dtSq = dt*dt;
		for(int i=0; i<3; i++)
		{
			errCov[i][i] += 2.0*errCov[i][i+3]*dt+-errCov[i][i+6]+errCov[i+3][i+3]*dtSq+
							    dynCov[i][i]*dtSq+dynCov[i+3][i+3]*dtSq*dtSq/4.0;
			errCov[i][i+3] += (errCov[i+3][i+3]-errCov[i][i+6])*dt-1.5*errCov[i+3][i+6]*dtSq+errCov[i+6][i+6]*dtSq*dt/2.0+
							  dynCov[i+3][i+3]*dtSq*dt/2.0;
			errCov[i][i+6] += errCov[i+3][i+6]*dt-errCov[i+6][i+6]*dtSq/2;
			errCov[i+3][i] = errCov[i][i+3];
			errCov[i+6][i] = errCov[i][i+6];

			errCov[i+3][i+3] += -2*errCov[i+3][i+6]*dt+errCov[i+6][i+6]*dtSq+dynCov[i+3][i+3]*dtSq;
			errCov[i+3][i+6] -= errCov[i+6][i+6]*dt;
			errCov[i+6][i+3] = errCov[i+3][i+6];

			errCov[i+6][i+6] += dynCov[i+6][i+6]*dtSq;
		}
	}

	void Observer_Translational::doMeasUpdateKF_posOnly(const Array2D<double> &meas,
														const Array2D<double> &measCov,
														Array2D<double> &state,
														Array2D<double> &errCov,
														const SO3 &att)
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

		// this will need to be rotation back to phone coords from inertial coords
		Array2D<double> biasDelta(3,1);
		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err= meas-submat(state,0,2,0,0);
		for(int i=0; i<3; i++)
		{
			state[i][0] += gainKF[i][i]*err[i][0];
			state[i+3][0] += gainKF[i+3][i]*err[i][0];
//			state[i+6][0] += gainKF[i+6][i]*err[i][0];
			biasDelta[i][0] = gainKF[i+6][i]*err[i][0];
		}
		biasDelta = att.inv()*biasDelta;
		for(int i=0; i<3; i++)
			state[i+6][0] += biasDelta[i][0];

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

	void Observer_Translational::doMeasUpdateKF_xyOnly(const Array2D<double> &meas,
													   const Array2D<double> &measCov,
													   Array2D<double> &state,
													   Array2D<double> &errCov,
													   const SO3 &att)
	{
		// K = S*C'*inv(C*S*C'+W)
		Array2D<double> gainKF(9,2,0.0);
		double den;
		for(int i=0; i<2; i++)
		{
			den = errCov[i][i]+measCov[i][i];
			gainKF[i][i] = errCov[i][i]/den;
			gainKF[i+3][i] = errCov[i][i+3]/den;
			gainKF[i+6][i] = errCov[i][i+6]/den;
		}

		// this will need to be rotation back to phone coords from inertial coords
		Array2D<double> biasDelta(3,1);
		biasDelta[2][0] = 0;
		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err= meas-submat(state,0,1,0,0);
		for(int i=0; i<2; i++)
		{
			state[i][0] += gainKF[i][i]*err[i][0];
			state[i+3][0] += gainKF[i+3][i]*err[i][0];
//			state[i+6][0] += gainKF[i+6][i]*err[i][0];
			biasDelta[i][0] = gainKF[i+6][i]*err[i][0];
		}
		biasDelta = att.inv()*biasDelta;
		for(int i=0; i<3; i++)
			state[i+6][0] += biasDelta[i][0];

		// S = (I-KC) S
		for(int i=0; i<2; i++)
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

	void Observer_Translational::doMeasUpdateKF_velOnly(const Array2D<double> &meas,
														const Array2D<double> &measCov,
														Array2D<double> &state,
														Array2D<double> &errCov,
														const SO3 &att)
	{
		if(norm2(meas) > 10)
			return; // screwy measuremnt

		// K = S*C'*inv(C*S*C'+W)
		Array2D<double> gainKF(9,3,0.0);
		double den;
		for(int i=0; i<3; i++)
		{
			den = errCov[i+3][i+3]+measCov[i][i];
			gainKF[i][i] = errCov[i][i+3]/den;
			gainKF[i+3][i] = errCov[i+3][i+3]/den;
			gainKF[i+6][i] = errCov[i+3][i+6]/den;
		}

		// this will need to be rotation back to phone coords from inertial coords
		Array2D<double> biasDelta(3,1);
		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-submat(state,3,5,0,0);
		for(int i=0; i<3; i++)
		{
			state[i][0] += gainKF[i][i]*err[i][0];
			state[i+3][0] += gainKF[i+3][i]*err[i][0];
//			state[i+6][0] += gainKF[i+6][i]*err[i][0];
			biasDelta[i][0] = gainKF[i+6][i]*err[i][0];
		}
		biasDelta = att.inv()*biasDelta;
		for(int i=0; i<3; i++)
			state[i+6][0] += biasDelta[i][0];

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

	void Observer_Translational::doMeasUpdateKF_heightOnly(double meas,
														   double measCov,
														   Array2D<double> &state,
														   Array2D<double> &errCov,
														   const SO3 &att)
	{
		// K = S*C'*inv(C*S*C'+W)
		Array2D<double> gainKF(9,1,0.0);
		double den = errCov[2][2]+measCov;
		gainKF[2][0] = errCov[2][2]/den;
		gainKF[5][0] = errCov[2][5]/den;
		gainKF[8][0] = errCov[2][8]/den;

		// this will need to be rotation back to phone coords from inertial coords
		Array2D<double> biasDelta(3,1);
		biasDelta[0][0] = biasDelta[1][0] = 0;
		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		double err = meas-state[2][0];
		state[2][0] += gainKF[2][0]*err;
		state[5][0] += gainKF[5][0]*err;
//		state[8][0] += gainKF[8][0]*err;
		biasDelta[2][0] = gainKF[8][0]*err;
		biasDelta = att.inv()*biasDelta;
		for(int i=0; i<3; i++)
			state[i+6][0] += biasDelta[i][0];

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

	void Observer_Translational::onObserver_AngularUpdated(const shared_ptr<SO3Data<double>> &attData, const shared_ptr<DataVector<double>> &angularVelData)
	{
//		Array2D<double> att = attData->data;
//		double s1 = sin(att[2][0]); double c1 = cos(att[2][0]);
//		double s2 = sin(att[1][0]); double c2 = cos(att[1][0]);
//		double s3 = sin(att[0][0]); double c3 = cos(att[0][0]);
//
//		// third column of orientation matrix, i.e. R*e3
//		//		Array2D<double> r(3,1);
//		//		r[0][0] = s1*s3+c1*c3*s2;
//		//		r[1][0] = c3*s1*s2-c1*s3;
//		//		r[2][0] = c2*c3;
//
//		// gravity vector orientation, i.e. R'*e3
//		Array2D<double> g(3,1);
//		g[0][0] = -s2;
//		g[1][0] = c2*s3;
//		g[2][0] = c2*c3;

		mMutex_att.lock();
		mCurAtt = attData->rotation;
		mMutex_att.unlock();

//		Array2D<double> e3(3,1);
//		e3[0][0] = e3[1][0] = 0;
//		e3[2][0] = 1;
//		Array2D<double> g = attData->rotation.inv()*e3;
//
//		shared_ptr<DataVector<double>> dirData(new DataVector<double>());
//		dirData->type = DATA_TYPE_GRAVITY_DIR;
//		dirData->timestamp = attData->timestamp;
//		dirData->data = -1.0*g.copy();
//
//		mMutex_events.lock();
//		mNewEventsBuffer.push_back(dirData);
//		mMutex_events.unlock();
	}

	void Observer_Translational::onNewCommStateVicon(const Collection<float> &data)
	{
		Time now;
		mMutex_posTime.lock();
		mLastViconPosTime.setTime(now);
		mMutex_posTime.unlock();

		Array2D<double> pos(3,1);
		for(int i=0; i<3; i++)
			pos[i][0] = data[i+6];
		pos = matmult(mRotViconToPhone, pos);

		if(!mHaveFirstVicon)
		{
			mMutex_kfData.lock();
			mStateKF[0][0] = pos[0][0];
			mStateKF[1][0] = pos[1][0];
			mStateKF[2][0] = pos[2][0];

			mStateBuffer.clear();
			mErrCovKFBuffer.clear();
			mMutex_kfData.unlock();
		}
		mHaveFirstVicon = true;

		shared_ptr<DataVector<double>> posData(new DataVector<double>() );
		posData->type = DATA_TYPE_VICON_POS;
		posData->timestamp.setTime(now);
		posData->data = pos.copy();

		mMutex_events.lock();
		mNewEventsBuffer.push_back(posData);
		mMutex_events.unlock();

		{
			String s = String();
			for(int i=0; i<data.size(); i++)
				s = s+data[i]+"\t";
			mQuadLogger->addEntry(Time(), LOG_ID_RECEIVE_VICON, s, LOG_FLAG_STATE);
		}

		mNewViconPosAvailable = true;
	}

	void Observer_Translational::onNewCommKalmanMeasVar(const Collection<float> &var)
	{
		assert(var.size() == mMeasCov.dim1());
		mMutex_kfData.lock();
		for(int i=0; i<mMeasCov.dim1(); i++)
			mMeasCov[i][i] = var[i];
		String s = "Meas var update -- diag(mMeasCov): \t";
		for(int i=0; i<mMeasCov.dim1(); i++)
			s = s+mMeasCov[i][i]+"\t";
		mPosMeasCov = submat(mMeasCov,0,2,0,2);
		mVelMeasCov = submat(mMeasCov,3,5,3,5);
		mMutex_kfData.unlock();
		Log::alert(s);
	}

	void Observer_Translational::onNewCommKalmanDynVar(const Collection<float> &var)
	{
		assert(var.size() == mDynCov.dim1());

		mMutex_kfData.lock();
//		// Zero out the err cov first. Depending on the new dyn cov
//		// it's possible that there were some off-diagonal entries
//		// that were non-zero before but should be zero down
//		// (i.e. if a one of the dyncov diagonals is zero)
//		for(int i=0; i<mErrCovKF.dim1(); i++)
//			for(int j=0; j<mErrCovKF.dim1(); j++)
//				mErrCovKF[i][j] = 0;
//		
		for(int i=0; i<mDynCov.dim1(); i++)
			mDynCov[i][i] = var[i];
		printArray("Dyn var update -- diag(mDynCov): \t", extractDiagonal(mDynCov));
		mMutex_kfData.unlock();
//		String s = "Dyn var update -- diag(mDynCov): \t";
//		for(int i=0; i<mDynCov.dim1(); i++)
//			s = s+mDynCov[i][i]+"\t";
//		mMutex_kfData.unlock();
//		Log::alert(s);
	}

	void Observer_Translational::onNewCommUseIbvs(bool useIbvs)
	{
		mUseIbvs = useIbvs;
		String s;
		if(useIbvs)
			mQuadLogger->addEntry(Time(), LOG_ID_IBVS_ENABLED, String(), LOG_FLAG_PC_UPDATES);
		else
		{
			mQuadLogger->addEntry(Time(), LOG_ID_IBVS_DISABLED, String(), LOG_FLAG_PC_UPDATES);
			mHaveFirstCameraPos = false;
		}
	}

	void Observer_Translational::onNewCommAccelBias(float xBias, float yBias, float zBias)
	{
//		mMutex_kfData.lock();
//for(int st=0; st<mStateKF.dim1(); st++)
//	if(isnan(mStateKF[st][0]))
//	{
//		printArray("state is nan before 5:\t",mStateKF);
//		break;
//	}
//		mAccelBiasReset[0][0] = xBias;
//		mAccelBiasReset[1][0] = yBias;
//		mAccelBiasReset[2][0] = zBias;
//
//		mStateKF[6][0] = mAccelBiasReset[0][0];
//		mStateKF[7][0] = mAccelBiasReset[1][0];
//		mStateKF[8][0] = mAccelBiasReset[2][0];
//
//		// also need to clear the buffers so we don't accidentally go back and use 
//		// one of the old values
//		mStateBuffer.clear();
//		mErrCovKFBuffer.clear();
//
//		shared_ptr<DataVector<double>> stateData = shared_ptr<DataVector<double>>(new DataVector<double>());
//		stateData->type = DATA_TYPE_STATE_TRAN;
//		stateData->data = mStateKF.copy();
//		mStateBuffer.push_back(stateData);
//
//		shared_ptr<DataVector<double>> errCovData = shared_ptr<DataVector<double>>(new DataVector<double>());
//		errCovData->type = DATA_TYPE_KF_ERR_COV;
//		errCovData->data = mErrCovKF.copy();
//		mErrCovKFBuffer.push_back(errCovData);
//for(int st=0; st<mStateKF.dim1(); st++)
//	if(isnan(mStateKF[st][0]))
//	{
//		printArray("state is nan after 5:\t",mStateKF);
//		break;
//	}
//		mMutex_kfData.unlock();
//
//		Log::alert(String()+"accel bias updated:\t"+xBias+"\t"+yBias+"\t"+zBias);
	}

	void Observer_Translational::onNewCommViconCameraOffset(float x, float y, float z)
	{
		mViconCameraOffset[0][0] = x;
		mViconCameraOffset[1][0] = y;
		mViconCameraOffset[2][0] = z;
		mIsViconCameraOffsetSet = true;
		Log::alert(String()+"Vicon--Camera offset updated:\t"+x+"\t"+y+"\t"+z);
	}

	void Observer_Translational::onNewCommTargetNominalLength(float length)
	{
		mTargetNominalLength = length;
		Log::alert(String()+"Target nominal length updated:\t"+length);
	}

	void Observer_Translational::onNewCommMAPHeightMeasCov(float cov)
	{
		mMAPHeightMeasCov = cov;
		Log::alert(String()+"MAP Height meas cov updated:\t"+cov);
	}

	void Observer_Translational::onNewSensorUpdate(const shared_ptr<IData> &data)
	{
		switch(data->type)
		{
			case DATA_TYPE_ACCEL:
				{
					Array2D<double> accel = static_pointer_cast<DataVector<double>>(data)->dataCalibrated.copy();

					SO3 att;
					if(mObsvAngular != NULL)
						att = mObsvAngular->estimateAttAtTime( data->timestamp );

					accel = att*accel;

					// The accelerometer is too noisy in z so, basically,
					// we are relying on the height measuremnts to 
					// keep velocity in check
					accel[2][0] = GRAVITY;


					shared_ptr<DataVector<double>> accelData(new DataVector<double>());
					accelData->type = DATA_TYPE_RAW_ACCEL;
					accelData->timestamp = data->timestamp;
					accelData->data = accel;

					mMutex_events.lock();
					mNewEventsBuffer.push_back(accelData);
					mMutex_events.unlock();
				}
				break;
			case DATA_TYPE_HEIGHT:
				{
					mMutex_kfData.lock();
					double curHeight = mStateKF[2][0];
					mMutex_kfData.unlock();
					const shared_ptr<HeightData<double>> d = static_pointer_cast<HeightData<double>>(data);

					if( abs(d->height-curHeight) < 0.1 )
					{
						mMutex_events.lock();
						mNewEventsBuffer.push_back( static_pointer_cast<HeightData<double>>(data) );
						mMutex_events.unlock();
					}
					else
					{
						// TODO: Better rejection algorithm
						Log::alert(String()+"Reject height -- curHeight: " + curHeight + "\t\tsonar height: "+d->height);
					}
				}
				break;
		}
	}

	void Observer_Translational::onVelocityEstimator_newEstimate(const shared_ptr<DataVector<double>> &velData,
																 const shared_ptr<Data<double>> &heightData)
	{
		bool tooLow = false;
		mMutex_kfData.lock();
		tooLow = mStateKF[2][0] < 0.3;
		mMutex_kfData.unlock();

		if(tooLow)
			return;

		mMutex_events.lock();
		mNewEventsBuffer.push_back(velData);
		mNewEventsBuffer.push_back(heightData);
		mMutex_events.unlock();
	}

	void Observer_Translational::onTargetFound(const shared_ptr<ImageTargetFindData> &data)
	{
		if(data->target == NULL)
			return;

		bool resetViconOffset = false;
		if(mUseIbvs)
		{
			if(!mHaveFirstCameraPos)
			{
				mQuadLogger->addEntry(Time(), LOG_ID_TARGET_ACQUIRED, String(), LOG_FLAG_PC_UPDATES);
				resetViconOffset = true;
			}
			mHaveFirstCameraPos = true;
		}
		mMutex_posTime.lock();
		mLastCameraPosTime.setTime();
		mMutex_posTime.unlock();

		double f = data->imageData->focalLength;
		double cx = data->imageData->centerX;
		double cy = data->imageData->centerY;

		// rotation compensation
		SO3 att = data->imageData->att;

		Array2D<double> p(3,1);
		p[0][0] = data->target->meanCenter.x - cx;
		p[1][0] = data->target->meanCenter.y - cy;
		p[2][0] = f;
		p = att*mRotCamToPhone*p;
		p = -1.0*p; // to get our position instead of the target's

		// Now estimate the pos
		double avgLength = 0;
		for(int i=0; i<data->target->squareData[0]->lineLengths.size(); i++)
			avgLength += data->target->squareData[0]->lineLengths[i];
		avgLength /= data->target->squareData[0]->lineLengths.size();

		mMutex_kfData.lock();
		const Array2D<double> state = IData::interpolate(data->timestamp, mStateBuffer);
		mMutex_kfData.unlock();

		double nomLength =  0.21;
		Array2D<double> pos(3,1);
//		pos[2][0] = mTargetNominalLength/avgLength*f;
		pos[2][0] = state[2][0]-mViconCameraOffset[2][0];
		pos[0][0] = p[0][0]/f*abs(pos[2][0]);
		pos[1][0] = p[1][0]/f*abs(pos[2][0]);

		if(resetViconOffset && !mIsViconCameraOffsetSet)
		{
			mViconCameraOffset = submat(state,0,2,0,0)-pos;
			printArray("Vicon offset set to: ", mViconCameraOffset);
			mIsViconCameraOffsetSet = true;
		}
		pos = pos+mViconCameraOffset;
		shared_ptr<DataVector<double>> posData(new DataVector<double>());
		posData->type = DATA_TYPE_CAMERA_POS;
		posData->timestamp.setTime(data->imageData->timestamp);
		posData->data = pos.copy();
		mMutex_events.lock();
		mNewEventsBuffer.push_back(posData);
		mMutex_events.unlock();

		String str = String();
		for(int i=0; i<pos.dim1(); i++)
			str = str+pos[i][0]+"\t";
		mQuadLogger->addEntry(LOG_ID_TARGET_ESTIMATED_POS, str, LOG_FLAG_CAM_RESULTS);
	}

	Time Observer_Translational::applyData(list<shared_ptr<IData>> &newEvents)
	{
		// assume newEvents is sorted
//		newEvents.sort(IData::timeSortPredicate);

		using std::isnan;
		// first go through and update all the buffers
		shared_ptr<IData> data;
		mMutex_kfData.lock();
		Time startTime;
		if(mStateBuffer.back()->timestamp > newEvents.front()->timestamp)
			startTime.setTime(newEvents.front()->timestamp);
		else
			startTime.setTime(mStateBuffer.back()->timestamp);
		while(newEvents.size() > 0)
		{
			data = newEvents.front();
			switch(data->type)
			{
				case DATA_TYPE_RAW_ACCEL:
					{
						shared_ptr<DataVector<double>> d = static_pointer_cast<DataVector<double>>(data);
						bool bad = false;
						for(int i=0; i<d->data.dim1(); i++)
						{
							if(isnan(d->data[i][0]))
							{
								printArray("accel is nan:\t",d->data);
								bad = true;
								break;
							}
						}
						if(!bad)
							mRawAccelDataBuffer.push_back(d);
					}
					break;
				case DATA_TYPE_VICON_POS:
					{
						shared_ptr<DataVector<double>> d = static_pointer_cast<DataVector<double>>(data);
						bool bad = false;
						for(int i=0; i<d->data.dim1(); i++)
						{
							if(isnan(d->data[i][0]))
							{
								printArray("vicon pos is nan:\t",d->data);
								bad = true;
								break;
							}
						}
						if(!bad)
							mViconPosBuffer.push_back(d);
					}
					break;
				case DATA_TYPE_CAMERA_POS:
					{
						shared_ptr<DataVector<double>> d = static_pointer_cast<DataVector<double>>(data);
						bool bad = false;
						for(int i=0; i<d->data.dim1(); i++)
						{
							if(isnan(d->data[i][0]))
							{
								printArray("camera pos is nan:\t",d->data);
								bad = true;
								break;
							}
						}
						if(!bad)
							mCameraPosBuffer.push_back(d);
					}
					break;
				case DATA_TYPE_MAP_VEL:
					{
						shared_ptr<DataVector<double>> d = static_pointer_cast<DataVector<double>>(data);
						bool bad = false;
						for(int i=0; i<d->data.dim1(); i++)
						{
							if(isnan(d->data[i][0]))
							{
								printArray("map vel is nan:\t",d->data);
								bad = true;
								break;
							}
						}
						if(!bad)
							mMapVelBuffer.push_back(d);
					}
					break;
				case DATA_TYPE_MAP_HEIGHT:
					{
						shared_ptr<Data<double>> d = static_pointer_cast<Data<double>>(data);
						bool bad = isnan(d->data);
						if(!bad)
							mMapHeightBuffer.push_back(d);
						else
							Log::alert("map height is bad");
					}
					break;
				case DATA_TYPE_HEIGHT:
					{
						shared_ptr<HeightData<double>> d = static_pointer_cast<HeightData<double>>(data);
						bool bad = isnan(d->height);
						if(!bad)
							mHeightDataBuffer.push_back(d);
						else
							Log::alert("height is bad");
					}
					break;
				default:
					Log::alert(String()+"Observer_Translational::applyData() --> Unknown data type: "+data->type);
					mMutex_kfData.unlock();
					return startTime;
			}

			newEvents.pop_front();
		}
		// To avoid bugs that keep popping up
		data = NULL;

		if(mRawAccelDataBuffer.size() == 0)
		{
			for(int i=0; i<6; i++)
				mStateKF[i][0] = 0;
			mStateKF[6][0] = 0;
			mStateKF[7][0] = 0;
			mStateKF[8][0] = 0;

//			mErrCovKF.inject(0.01*mDynCov);
			mErrCovKF.inject(1e-6*createIdentity((double)9));
			mMutex_kfData.unlock();
			return startTime;
		}

		// now we have to rebuild the state and errcov history
		list<shared_ptr<DataVector<double>>>::iterator posIter, velIter;
//		list<shared_ptr<DataVector<double>>>::iterator rawAccelIter;//, gravDirIter;
		list<shared_ptr<HeightData<double>>>::iterator heightIter;
		list<shared_ptr<Data<double>>>::iterator mapHeightIter;
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

		// These iters point to the last data point with iter->timestamp <startTime 
		posIter = IData::findIndexReverse(startTime, *posBuffer);
		velIter = IData::findIndexReverse(startTime, *velBuffer);
//		rawAccelIter = IData::findIndexReverse(startTime, mRawAccelDataBuffer);
		heightIter = IData::findIndexReverse(startTime, mHeightDataBuffer); 
		mapHeightIter = IData::findIndexReverse(startTime, mMapHeightBuffer); 
		
		// but I want the next one for these because
		// they are not measurements
//		if(rawAccelIter != mRawAccelDataBuffer.end()) rawAccelIter++;
		
		// construct a sorted list of all events
		list<shared_ptr<IData>> events;
		if(posIter != posBuffer->end()) 
			events.insert(events.end(), ++posIter, posBuffer->end());
		if(velIter != velBuffer->end())
			events.insert(events.end(), ++velIter, velBuffer->end());
//		if(rawAccelIter != mRawAccelDataBuffer.end())
//			events.insert(events.end(), ++rawAccelIter, mRawAccelDataBuffer.end());
		if(heightIter != mHeightDataBuffer.end())
			events.insert(events.end(), ++heightIter, mHeightDataBuffer.end());
		if(mapHeightIter != mMapHeightBuffer.end())
			events.insert(events.end(), ++mapHeightIter, mMapHeightBuffer.end());

		if(events.size() == 0)
		{
			// apply data at the correct point in time
			mStateKF.inject( IData::interpolate(startTime, mStateBuffer));
			mErrCovKF.inject( IData::interpolate(startTime, mErrCovKFBuffer));

			mMutex_kfData.unlock();
			return startTime;
		}

		// apply data at the correct point in time
		mStateKF.inject( IData::interpolate(startTime, mStateBuffer));
		mErrCovKF.inject( IData::interpolate(startTime, mErrCovKFBuffer));
		events.sort(IData::timeSortPredicate);

		// clear out state and errcov information that we are about to replace
		IData::truncate(startTime, mStateBuffer);
		IData::truncate(startTime, mErrCovKFBuffer);

		// get accel vector at startTime 
		Array2D<double> accel(3,1), accelRaw(3,1), gravDir(3,1);
		gravDir[0][0] = 0;
		gravDir[1][0] = 0;
		gravDir[2][0] = -1;

//		if(rawAccelIter != mRawAccelDataBuffer.end())
//			accelRaw.inject((*rawAccelIter)->data);
//		else
//			accelRaw.inject(mRawAccelDataBuffer.back()->data);
//		accel.inject(IData::interpolate(startTime,mRawAccelDataBuffer);
//		accel.inject(accelRaw + GRAVITY*gravDir);

		// apply events in order
		list<shared_ptr<IData>>::const_iterator eventIter = events.begin();
		Time lastUpdateTime(startTime);
		Time midTime;
		double dt;
		shared_ptr<DataVector<double>> stateData, errCovData;
		SO3 att;
		while(eventIter != events.end())
		{
			dt = Time::calcDiffNS(lastUpdateTime, (*eventIter)->timestamp)/1.0e9;
			midTime.setTime(lastUpdateTime);
			midTime.addTimeNS(dt*0.5e9);
			if(mObsvAngular != NULL)
				att = mObsvAngular->estimateAttAtTime(midTime);

			accelRaw.inject(IData::interpolate(midTime,mRawAccelDataBuffer));
			accel.inject(accelRaw+GRAVITY*gravDir);

			doTimeUpdateKF(accel, dt, mStateKF, mErrCovKF, mDynCov, att);

			switch((*eventIter)->type)
			{
				case DATA_TYPE_RAW_ACCEL:
					// this is now handled by the above interpolation
//					accelRaw.inject( static_pointer_cast<DataVector<double>>(*eventIter)->data);
//					accel.inject(accelRaw+GRAVITY*gravDir);
					break;
				case DATA_TYPE_CAMERA_POS:
					{
						shared_ptr<DataVector<double>> d = static_pointer_cast<DataVector<double>>(*eventIter);
						Array2D<double> meas(2,1);
						meas[0][0] = d->data[0][0];
						meas[1][0] = d->data[1][0];
						doMeasUpdateKF_xyOnly(meas, submat(mPosMeasCov,0,1,0,1), mStateKF, mErrCovKF, att);
					}
					break;
				case DATA_TYPE_VICON_POS:
					//doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<double>>(*eventIter)->data, mPosMeasCov, mStateKF, mErrCovKF);
					{
						shared_ptr<DataVector<double>> d = static_pointer_cast<DataVector<double>>(*eventIter);
						Array2D<double> meas(2,1);
						meas[0][0] = d->data[0][0];
						meas[1][0] = d->data[1][0];
						Array2D<double> measCov = 1e-4*createIdentity((double)2.0);
//						Array2D<double> measCov = submat(mPosMeasCov,0,1,0,1);
						doMeasUpdateKF_xyOnly(meas, measCov, mStateKF, mErrCovKF, att);
					}
					break;
				case DATA_TYPE_MAP_VEL:
					{
						shared_ptr<DataVector<double>> d = static_pointer_cast<DataVector<double>>(*eventIter);
						Array2D<double> vel = d->data;
						doMeasUpdateKF_velOnly(vel,mVelMeasCov, mStateKF, mErrCovKF, att);
					}
					break;
				case DATA_TYPE_MAP_HEIGHT:
					doMeasUpdateKF_heightOnly(static_pointer_cast<Data<double>>(*eventIter)->data, mMAPHeightMeasCov, mStateKF, mErrCovKF, att);
					break;
				case DATA_TYPE_HEIGHT:
					doMeasUpdateKF_heightOnly(static_pointer_cast<HeightData<double>>(*eventIter)->height, mPosMeasCov[2][2], mStateKF, mErrCovKF, att);
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

		mMutex_kfData.unlock();

		return lastUpdateTime;
	}

	void Observer_Translational::setStartTime(Time t)
	{
		mMutex_kfData.lock(); mMutex_events.lock();
		mStartTime.setTime(t);
		// for now I'll be lazy and just clear everything out, assuming 
		// this only happens when nothing interesting is happening anyhow
		for(int i=0; i<mDataBuffers.size(); i++)
			mDataBuffers[i]->clear();

		mNewEventsBuffer.clear();

		for(int i=0; i<6; i++)
			mStateKF[i][0] = 0;

		mMutex_kfData.unlock(); mMutex_events.unlock();
	}

	Array2D<double> Observer_Translational::estimateStateAtTime(const Time &t)
	{
		mMutex_kfData.lock();
		Array2D<double> state = IData::interpolate(t, mStateBuffer);
		mMutex_kfData.unlock();

		return state;
	}

	Array2D<double> Observer_Translational::estimateErrCovAtTime(const Time &t)
	{
		Array2D<double> errCov;
		mMutex_kfData.lock();
		if(t < mErrCovKFBuffer.front()->timestamp)
			errCov = 10*createIdentity((double)mErrCovKF.dim1());
		else
			errCov = IData::interpolate(t, mErrCovKFBuffer);
		mMutex_kfData.unlock();

		return errCov;
	}
} // namespace Quadrotor
} // namespace ICSL
