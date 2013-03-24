#include "Observer_Translational.h"
#include "TNT/jama_lu.h"
#include "TNT/jama_qr.h"

using namespace toadlet::egg;
using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{
	Observer_Translational::Observer_Translational() :
		mRotViconToPhone(3,3,0.0),
		mAkf(6,6,0.0),
		mAkf_T(6,6,0.0),
		mBkf(6,3,0.0),
		mCkf(6,6,0.0),
		mCkf_T(6,6,0.0),
		mMeasCov(6,6,0.0),
		mPosMeasCov(3,3,0.0),
		mVelMeasCov(3,3,0.0),
		mDynCov(6,6,0.0),
		mErrCovKF(6,6,0.0),
		mStateKF(6,1,0.0),
		mAttBias(3,1,0.0),
		mAttBiasReset(3,1,0.0),
		mAttBiasAdaptRate(3,0.0),
		mLastViconPos(3,1,0.0),
		mLastCameraPos(3,1,0.0),
		mBarometerHeightState(2,1,0.0),
		mLastViconVel(3,1,0.0),
		mLastCameraVel(3,1,0.0),
		mViconCameraOffset(3,1,0.0)
	{
		mRunning = false;
		mDone = true;

		mMass = 0.850;
		mForceGainReset = 0.0040;
		mForceGain = mForceGainReset;
		
		mLastForceGainUpdateTime.setTimeMS(0);
		mLastAttBiasUpdateTime.setTimeMS(0);
		mLastPosReceiveTime.setTimeMS(0);
		mLastBarometerMeasTime.setTimeMS(0);
		
		mAkf.inject(createIdentity(6));
		mCkf.inject(createIdentity(6));
		mMeasCov[0][0] = mMeasCov[1][1] = mMeasCov[2][2] = 0.01*0.01;
		mMeasCov[3][3] = mMeasCov[4][4] = mMeasCov[5][5] = 0.3*0.3;
		mMeasCov[2][2] = 0.05*0.05;
		mMeasCov[5][5] = 0.5*0.5;
		mPosMeasCov = submat(mMeasCov,0,2,0,2);
		mVelMeasCov = submat(mMeasCov,3,5,3,5);
mVelMeasCov = 1e-6*submat(mMeasCov,3,5,3,5);
		mDynCov.inject(0.02*0.02*createIdentity(6));
		mDynCov[5][5] *= 10;
		mErrCovKF.inject(1e-4*createIdentity(6));

		mAkf_T.inject(transpose(mAkf));
		mCkf_T.inject(transpose(mCkf));
		mForceGainAdaptRate = 0;

		mZeroHeight = 76;
		
		mDoMeasUpdate = false;
		mNewViconPosAvailable = mNewCameraPosAvailable = false;

		mNewImageResultsReady = false;

		mPhoneTempData = NULL;
		mImageMatchData = NULL;

		mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
								 createRotMat(0,(double)PI));
		mRotPhoneToCam = transpose(mRotCamToPhone);

		mFlowCalcDone = true;
		mNewOpticFlowReady = false;

		mMotorOn = false;

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mUseCameraPos = false;
		mUseViconPos = true;
		mHaveFirstCameraPos = false;

		mUseIbvs = false;

		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mStateBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mErrCovKFBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mViconPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mViconVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mCameraPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mCameraVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mOpticFlowVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mHeightDataBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mMotorCmdsBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mThrustDirBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data> >*)(&mThrustBuffer));

		mOpticFlowVel.type = DATA_TYPE_OPTIC_FLOW_VEL;
		mOpticFlowVel.data = Array2D<double>(3,1,0.0);
	}

	Observer_Translational::~Observer_Translational()
	{
	}

	void Observer_Translational::shutdown()
	{
		Log::alert("------------------------- Observer_Translational shutdown started  --------------------------------------------------");
		mRunning = false;
		System sys;
		while(!mDone)
			sys.msleep(10);

		mPhoneTempData = NULL;
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

		class : public Thread{
					public:
					void run(){imageMatchData->lock(); parent->calcOpticalFlow(imageMatchData); imageMatchData->unlock();}
					shared_ptr<ImageMatchData> imageMatchData;
					Observer_Translational *parent;
				} flowCalcThread;
		flowCalcThread.parent = this;

		Time lastUpdateTime;
		Array2D<double> measTemp(6,1);
		Array2D<double> r(3,1,0.0);
		Array2D<double> accel(3,1);
		Array2D<double> pos(3,1),vel(3,1);
		double s1, s2, s3, c1, c2, c3;
		double dt;
		Time lastBattTempTime;
		Array2D<double> flowVel(3,1,0.0);
		Array2D<double> errCov(12,1,0.0);

		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);

		Time loopTime;
		double thrust=0;
		while(mRunning)
		{
			loopTime.setTime();
			mMutex_cmds.lock();
			if(mMotorCmdsBuffer.size() > 0)
			{
				for(int i=0; i<4; i++)
					thrust += mForceGain*mMotorCmdsBuffer.back()->data[i][0];
			}
			else
				thrust = 0;
			mMutex_cmds.unlock();
			shared_ptr<Data> thrustData(new Data());
			thrustData->type = DATA_TYPE_THRUST;
			if(mMotorCmdsBuffer.size() > 10)
				thrustData->timestamp.setTime(mMotorCmdsBuffer.back()->timestamp);
			thrustData->data = thrust;
			mThrustBuffer.push_back(thrustData);

			mMutex_data.lock();
			if(mMotorOn)
			{
				mMutex_att.lock();
				if(mThrustDirBuffer.size() > 0)
					r.inject(mThrustDirBuffer.back()->data);
				mMutex_att.unlock();

				accel.inject(thrust/mMass*r);
				accel[2][0] -= GRAVITY;
			}
			else 
			{
				for(int i=0; i<accel.dim1(); i++)
					accel[i][0] = 0;
				mAttBias.inject(mAttBiasReset);
				mForceGain = mForceGainReset;
			}
			mMutex_data.unlock();

			dt = lastUpdateTime.getElapsedTimeUS()/1.0e6;
			mMutex_data.lock();
			for(int i=0; i<3; i++)
			{
				mAkf[i][i+3] = dt;
				mAkf_T[i+3][i] = dt;
				mBkf[i+3][i] = dt;
			}
			doTimeUpdateKF(accel, mAkf, mAkf_T, mBkf, mStateKF, mErrCovKF, mDynCov);
			mMutex_data.unlock();
			lastUpdateTime.setTime();

			mMutex_meas.lock();
			if(mHaveFirstCameraPos && mLastCameraPosTime.getElapsedTimeMS() > 500)
				mHaveFirstCameraPos = false;
			mMutex_meas.unlock();

//			if(mDoMeasUpdate)
//			{
//				mMutex_meas.lock(); measTemp.inject(mLastMeas); mMutex_meas.unlock();
//				mMutex_data.lock();
//				doMeasUpdateKF(measTemp);
//				mMutex_data.unlock();
//			}
			if(mNewOpticFlowReady)
			{
				mNewOpticFlowReady = false;
				mMutex_meas.lock();
				Array2D<double> vel = mOpticFlowVel.data.copy();
				Time imgTime = mOpticFlowVel.timestamp;
				mMutex_meas.unlock();

				mStateKF.inject( Data::interpolate(imgTime, mStateBuffer) );
				mErrCovKF.inject( Data::interpolate(imgTime, mErrCovKFBuffer) );

				// do measurement update
				doMeasUpdateKF_velOnly(vel, mVelMeasCov, mStateKF, mErrCovKF);

Array2D<double> beforeState = mStateKF.copy();
Array2D<double> beforeErrCov = mErrCovKF.copy();
rebuildState(imgTime);
Array2D<double> afterState = mStateKF.copy();
Array2D<double> afterErrCov = mErrCovKF.copy();
Log::alert("++++++++++++++++++++++++++++++++++++++++++++++++++");
Log::alert("++++++++++++++++++++++++++++++++++++++++++++++++++");
Log::alert(String()+"Backtrack time: "+imgTime.getElapsedTimeMS());
printArray("vel:\t", transpose(vel));
printArray("beforeState:\t", transpose(beforeState));
printArray("afterState:\t", transpose( afterState));
Log::alert("--------------------------------------------------");
printArray("beforeErrCov:\n", beforeErrCov);
printArray("afterErrCov:\n", afterErrCov);
mStateKF.inject(beforeState);
mErrCovKF.inject(beforeErrCov);

				// unwind the state back to the time when the picture was taken
				mMutex_data.lock();

//				mStateKF.inject( Data::interpolate(imgTime, mStateBuffer) );
//				mErrCovKF.inject( Data::interpolate(imgTime, mErrCovKFBuffer) );

				// do measurement update
//				doMeasUpdateKF_velOnly(vel, mVelMeasCov, mStateKF, mErrCovKF);

//				// now apply forward back to present time
//				while(mPosMeasBuffer.size() > 0 && mPosMeasBuffer.front()->timestamp < imgTime)
//					mPosMeasBuffer.pop_front();
////				while(mVelMeasTimeBuffer.front() < imgTime)
////				{
////					mVelMeasTimeBuffer.pop_front();
////					mVelMeasBuffer.pop_front();
////				}
//				while(mAccelDataBuffer.size() > 0 && mAccelDataBuffer.front()->timestamp < imgTime)
//					mAccelDataBuffer.pop_front();
//				
//				mStateBuffer.clear();
//				mErrCovKFBuffer.clear();
//				list<shared_ptr<DataVector> >::const_iterator accelIter = mAccelDataBuffer.begin();
//				list<shared_ptr<DataVector> >::const_iterator accelIterNext = mAccelDataBuffer.begin();
//				accelIterNext++;
//				list<shared_ptr<DataVector> >::const_iterator posIter = mPosMeasBuffer.begin();
////				list<Array2D<double> >::const_iterator velIter = mVelMeasBuffer.begin();
//				Array2D<double> accel(3,1), pos(3,1);// , vel(3,1);  -- vel is defined above
//				double dt;
//				while(accelIterNext != mAccelDataBuffer.end())
//				{
//					accel.inject((*accelIter)->data);
//					dt = Time::calcDiffUS( (*accelIter)->timestamp, (*accelIterNext)->timestamp)/1.0e6;
//
//					for(int i=0; i<3; i++)
//					{
//						mAkf[i][i+3] = dt;
//						mAkf_T[i+3][i] = dt;
//						mBkf[i+3][i] = dt;
//					}
//					doTimeUpdateKF(accel, mAkf, mAkf_T, mBkf, mStateKF, mErrCovKF, mDynCov);
//
//					// this is basically assuming that the accel data is sampled fast enough
//					// that the position measurement doesn't need to be interpolated before
//					// applying the position update
//					if( posIter != mPosMeasBuffer.end() && (*posIter)->timestamp > (*accelIter)->timestamp)
//					{
//						pos.inject( (*posIter)->data );
//						posIter++;
//
//						doMeasUpdateKF_posOnly(pos, mPosMeasCov, mStateKF, mErrCovKF);
//					}
////					if( velMeasTimeIter != mVelMeasTimeBuffer.end() && (*velMeasTimeIter) > (*accelIter)->timestamp)
////					{
////						vel.inject(*velIter);
////						velMeasTimeIter++;
////						velIter++;
////
////						doMeasUpdateKF_velOnly(vel, mVelMeasCov);
////					}
//
//
//					shared_ptr<DataVector> stateData = shared_ptr<DataVector>(new DataVector());
//					stateData->type = DATA_TYPE_STATE_TRAN;
//					stateData->data = mStateKF.copy();
//					stateData->timestamp.setTime( (*accelIter)->timestamp);
//					mStateBuffer.push_back(stateData);
//					shared_ptr<DataVector> errCovData = shared_ptr<DataVector>(new DataVector());
//					errCovData->type = DATA_TYPE_KF_ERR_COV;
//					errCovData->data = mErrCovKF.copy();
//					errCovData->timestamp.setTime( (*accelIter)->timestamp);
//					mErrCovKFBuffer.push_back(errCovData);
//
//					accelIter++;
//					accelIterNext++;
//				}

				mMutex_data.unlock();
			}

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

			// update bias and force scaling estimates
			if( (mNewViconPosAvailable && mUseViconPos) ||
				(mNewCameraPosAvailable && mUseCameraPos) )
			{
				mMutex_data.lock();
				if(mLastAttBiasUpdateTime.getMS() == 0)
					mLastAttBiasUpdateTime.setTime(); // this will effectively cause dt=0
				double dt = mLastAttBiasUpdateTime.getElapsedTimeUS()/1.0e6;
				mMutex_meas.lock();
				Array2D<double> pos;
				if(mUseViconPos)
					pos = mLastViconPos.copy();
				else
					pos = mLastCameraPos.copy();
				mMutex_meas.unlock();
				Array2D<double> err = pos-submat(mStateKF,0,2,0,0);
				if(dt < 0.1)
				{
					mAttBias[0][0] += mAttBiasAdaptRate[0]*dt*err[1][0];
					mAttBias[1][0] += mAttBiasAdaptRate[1]*dt*(-err[0][0]);
				}
				if(mLastForceGainUpdateTime.getMS() == 0)
					mLastForceGainUpdateTime.setTime();
				dt = mLastForceGainUpdateTime.getElapsedTimeUS()/1.0e6;
				if(pos[2][0] > 0.4 && dt < 0.1) // doing this too low runs into problems with ground effect
					mForceGain += mForceGainAdaptRate*dt*err[2][0];
				mLastAttBiasUpdateTime.setTime();
				mLastForceGainUpdateTime.setTime();

				{
					String str1 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_ATT_BIAS+"\t";
					for(int i=0; i<mAttBias.dim1(); i++)
						str1 = str1+mAttBias[i][0]+"\t";
					mQuadLogger->addLine(str1,LOG_FLAG_STATE);

					String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_FORCE_GAIN+"\t";
					str2 = str2+mForceGain+"\t";
					mQuadLogger->addLine(str2,LOG_FLAG_STATE);
				}
				mMutex_data.unlock();
			}

			if(mNewViconPosAvailable && mUseViconPos)
			{
				mNewViconPosAvailable = false;
				mMutex_meas.lock();
				Array2D<double> pos = mLastViconPos.copy();
				Array2D<double> vel = mLastViconVel.copy();
				mMutex_meas.unlock();

				mMutex_data.lock();
				doMeasUpdateKF_posOnly(pos, mPosMeasCov, mStateKF, mErrCovKF);
				doMeasUpdateKF_velOnly(vel, 100*mVelMeasCov, mStateKF, mErrCovKF);
				mMutex_data.unlock();

			}
			if(mNewCameraPosAvailable && mUseCameraPos)
			{
				mNewCameraPosAvailable = false;
				mMutex_meas.lock();
				Array2D<double> pos = mLastCameraPos.copy();
				Array2D<double> vel = mLastCameraVel.copy();
				mMutex_meas.unlock();

				mMutex_data.lock();
				doMeasUpdateKF_posOnly(pos, mPosMeasCov, mStateKF, mErrCovKF);
				doMeasUpdateKF_velOnly(vel, 100*mVelMeasCov, mStateKF, mErrCovKF);
				mMutex_data.unlock();
			}

			if(mNewImageResultsReady && mFlowCalcDone
//					&& mMotorOn // sometimes the blurry images when sitting close to the ground creates artificial matches
					)
			{
				// if we're here then the previous thread should already be finished
				flowCalcThread.imageMatchData = mImageMatchData;
				mFlowCalcDone = false;
				flowCalcThread.start();
				mNewImageResultsReady = false;
			}

			mMutex_data.lock();
			shared_ptr<DataVector> stateData = shared_ptr<DataVector>(new DataVector());
			stateData->type = DATA_TYPE_STATE_TRAN;
			stateData->data = mStateKF.copy();
			mStateBuffer.push_back(stateData);
			shared_ptr<DataVector> errCovData = shared_ptr<DataVector>(new DataVector());
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
				String str = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_KALMAN_ERR_COV+"\t";
				for(int i=0; i<errCov.dim1(); i++)
					str = str+errCov[i][0]+"\t";
				mQuadLogger->addLine(str,LOG_FLAG_STATE);
			}

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onObserver_TranslationalUpdated(pos, vel);

			uint64 t = loopTime.getElapsedTimeUS();
			if(t < 5e3)
				System::usleep(5e3-t); // maintain a (roughly) 200Hz update rate
		}

		mDone = true;
	}

	// See eqn 98 in the Feb 25, 2013 notes
	void  Observer_Translational::calcOpticalFlow(shared_ptr<ImageMatchData> const matchData)
	{
		if(matchData->featurePoints[0].size() < 5)
		{
			String str = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS+"\t";
			mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
			mFlowCalcDone = true;
			return;
		}

		double dt = matchData->dt;
		if(dt < 1e-3)
		{
			mFlowCalcDone = true;
			return;
		}
		mMutex_data.lock();
		Array2D<double> Sn = 300*300*createIdentity(2);
		Array2D<double> SnInv(2,2,0.0);
		SnInv[0][0] = 1.0/Sn[0][0]; SnInv[1][1] = 1.0/Sn[1][1];

		Array2D<double> mu_v1 = submat(mStateKF,3,5,0,0);
		Array2D<double> Sv = submat(mErrCovKF,3,5,3,5);
		JAMA::LU<double> SvLU(Sv);
		Array2D<double> SvInv1 = SvLU.solve(createIdentity(3));

		mOpticFlowVel.timestamp.setTime(matchData->imgData1->timestamp);
		Time img1Time = matchData->imgData1->timestamp;
		double z = Data::interpolate(img1Time, mHeightDataBuffer);
		z -= 0.060; // offset between markers and camera

		// Rotate prior velocity information to camera coords
		Array2D<double> mu_v = matmult(mRotPhoneToCam, mu_v1);
		Array2D<double> SvInv = matmult(mRotPhoneToCam, matmult(SvInv1, mRotCamToPhone));

		mMutex_data.unlock();
//Log::alert("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");

		Array2D<double> A(1,3,0.0);
		Array2D<double> B(3,3,0.0);
		Array2D<double> R1 = createRotMat_ZYX(matchData->imgData0->att[2][0], matchData->imgData0->att[1][0], matchData->imgData0->att[0][0]);
		Array2D<double> R2 = createRotMat_ZYX(matchData->imgData1->att[2][0], matchData->imgData1->att[1][0], matchData->imgData1->att[0][0]);
		Array2D<double> R1_T = transpose(R1);
		Array2D<double> R2_T = transpose(R2);
		Array2D<double> R = matmult(R1, transpose(R2));
		Array2D<double> q1a(3,1), q2a(3,1);
		Array2D<double> q1(2,1), q2(2,1);
		Array2D<double> Lv(2,3), Lv1(2,3), Lv2(2,3);
		Array2D<double> Lw(2,3), Lw1(2,3), Lw2(2,3);
		Array2D<double> angularVel(3,1,0.0);
		double f1= matchData->imgData0->focalLength;
		double f2 = matchData->imgData1->focalLength;
		double cx = matchData->imgData0->img->cols/2;
		double cy = matchData->imgData0->img->rows/2;
		Array2D<double> rotPoint(3,1);
		rotPoint[0][0] = 0;
		rotPoint[1][0] = 0;
		rotPoint[2][0] = 0;
		rotPoint = rotPoint*(f1/0.0037); // assumes f1=f2
		for(int i=0; i<matchData->featurePoints[0].size(); i++)
		{
			q1[0][0] = matchData->featurePoints[0][i].x-cx;
			q1[1][0] = matchData->featurePoints[0][i].y-cy;
			q2[0][0] = matchData->featurePoints[1][i].x-cx;
			q2[1][0] = matchData->featurePoints[1][i].y-cy;

			// Unrotate
			q1a[0][0] = q1[0][0];
			q1a[1][0] = q1[1][0];
			q1a[2][0] = f1;
			q2a[0][0] = q2[0][0];
			q2a[1][0] = q2[1][0];
			q2a[2][0] = f2;
//			// change q2 points to q1 attitude
//			q2a = matmult(R, q2a); 
			q1a = matmult(R1_T, q1a-rotPoint)+rotPoint;
			q2a = matmult(R2_T, q2a-rotPoint)+rotPoint;

			// project back on to the focal plane
			q1a = f1/q1a[2][0]*q1a;
			q2a = f2/q2a[2][0]*q2a;

			// back to 2d points
			q1[0][0] = q1a[0][0]; q1[1][0] = q1a[1][0];
			q2[0][0] = q2a[0][0]; q2[1][0] = q2a[1][0];

			// Velocity jacobian
			Lv1[0][0] = -f1; Lv1[0][1] = 0; Lv1[0][2] = q1[0][0];
			Lv1[1][0] = 0; Lv1[1][1] = -f1; Lv1[1][2] = q1[1][0];

			Lv2[0][0] = -f2; Lv2[0][1] = 0; Lv2[0][2] = q2[0][0];
			Lv2[1][0] = 0; Lv2[1][1] = -f2; Lv2[1][2] = q2[1][0];

			Lv = Lv1.copy();

//			Lw1[0][0] = q1[0][0]*q1[1][0]; Lw1[0][1] = -(1+pow(q1[0][0],2)); Lw1[0][2] = q1[1][0];
//			Lw1[1][0] = 1+pow(q1[1][0],2); Lw1[1][1] = -Lw1[0][0];			 Lw1[1][2] = -q1[0][0];
//			Lw1 = 1.0/matchData->imgData0->focalLength*Lw1;
//
//			Lw2[0][0] = q2[0][0]*q2[1][0]; Lw2[0][1] = -(1+pow(q2[0][0],2)); Lw2[0][2] = q2[1][0];
//			Lw2[1][0] = 1+pow(q2[1][0],2); Lw2[1][1] = -Lw2[0][0];			 Lw2[1][2] = -q2[0][0];
//			Lw2 = 1.0/matchData->imgData1->focalLength*Lw2;
//
//			Lw = Lw1.copy();

//			angularVel.inject(0.5*(matchData->imgData0->startAngularVel+matchData->imgData1->endAngularVel));

//			A += matmult(transpose(q2-q1-matmult(Lw,matmult(mRotPhoneToCam,angularVel))),matmult(SnInv, Lv));
			A += matmult(transpose(q2-q1),matmult(SnInv, Lv));
			B += matmult(transpose(Lv), matmult(SnInv, Lv));
		}
		int maxPoints = 50;
		int numPoints = matchData->featurePoints[0].size();
		double scale = min((double)maxPoints, (double)numPoints)/numPoints;
		A = scale*A;
		B = scale*B;
		Array2D<double> temp1 = (dt/z)*A+matmult(transpose(mu_v), SvInv);
		Array2D<double> temp2 = ((dt*dt)/(z*z))*B+SvInv;
		JAMA::LU<double> temp2_TQR(transpose(temp2));
		Array2D<double> vel1 = temp2_TQR.solve(transpose(temp1));

		JAMA::LU<double> B_TLU(transpose(B));
		Array2D<double> velLS1 = z/dt*B_TLU.solve(transpose(A)); // least squares

		// Finally, convert the velocity from camera to phone coords
		Array2D<double> vel = matmult(mRotCamToPhone, vel1);
		Array2D<double> velLS = matmult(mRotCamToPhone, velLS1);

		if(vel.dim1() == 3)
		{
			String str = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW+"\t";
			for(int i=0; i<vel.dim1(); i++)
				str = str+vel[i][0]+"\t";
			str = str+matchData->imgData0->timestamp.getElapsedTimeMS()+"\t";
			mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);

			mNewOpticFlowReady = true;

			mMutex_meas.lock();
			mOpticFlowVel.timestamp.setTime(img1Time);
			mOpticFlowVel.data.inject(vel);
			shared_ptr<DataVector> velData(new DataVector());
			velData->type = DATA_TYPE_OPTIC_FLOW_VEL;
			velData->timestamp.setTime(img1Time);
			velData->data = vel.copy();
			mOpticFlowVelBuffer.push_back(velData);
			mMutex_meas.unlock();
		}
		else
		{
			Log::alert("Why is the optical flow vel corrupted?");
Log::alert("++++++++++++++++++++++++++++++++++++++++++++++++++");
Log::alert(String()+"dt: "+dt);
Log::alert(String()+"z: "+z);
printArray("A:\n",A);
printArray("B:\n",B);
printArray("SvInv:\n",SvInv);
printArray("temp1:\n",temp1);
printArray("temp2:\n",temp2);
printArray("mu_v:\n",mu_v);
printArray("vel1:\n",vel1);
printArray("vel:\n",vel);
		}
		mFlowCalcDone = true;
	
		String str2 = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW_LS+"\t";
		for(int i=0; i<velLS.dim1(); i++)
			str2 = str2+velLS[i][0]+"\t";
		mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);

	}

	void Observer_Translational::setMotorCmds(double const cmds[4])
	{
		shared_ptr<DataVector> data(new DataVector());
		data->type = DATA_TYPE_MOTOR_CMDS;
		data->data = Array2D<double>(4,1,0.0);
		for(int i=0; i<4; i++)
			data->data[i][0] = cmds[i];

		mMutex_cmds.lock();
		mMotorCmdsBuffer.push_back(data);
		mMutex_cmds.unlock();
	}

	void Observer_Translational::doTimeUpdateKF(Array2D<double> const &actuator, Array2D<double> const &A, Array2D<double> const &B, Array2D<double> &state, Array2D<double> &errCov, Array2D<double> const &dynCov)
	{
		doTimeUpdateKF(actuator, A, transpose(A), B, state, errCov, dynCov);
	}

	void Observer_Translational::doTimeUpdateKF(Array2D<double> const &actuator, Array2D<double> const &A, Array2D<double> const &A_T,  Array2D<double> const &B, Array2D<double> &state, Array2D<double> &errCov, Array2D<double> const &dynCov)
	{
		state.inject(matmult(A,state)+matmult(B, actuator));
		errCov.inject(matmult(A, matmult(errCov, A_T)) + dynCov);
	}

	// This function needs to be called inside of a locked
	// mMutex_data block
//	void Observer_Translational::doMeasUpdateKF(TNT::Array2D<double> const &meas)
//	{
//		mDoMeasUpdate = false;
//		Array2D<double> m1_T = transpose(matmult(mErrCovKF, mCkf_T));
//		Array2D<double> m2_T = transpose(matmult(mCkf, matmult(mErrCovKF, mCkf_T)) + mMeasCov);
//
//		// I need to solve K = m1*inv(m2) which is the wrong order
//		// so solve K^T = inv(m2^T)*m1^T
//// The QR solver doesn't seem to give stable results. When I used it the resulting mErrCovKF at the end
//// of this function was no longer symmetric
////		JAMA::QR<double> m2QR(m2_T); 
////	 	Array2D<double> gainKF = transpose(m2QR.solve(m1_T));
//		JAMA::LU<double> m2LU(m2_T);
//		Array2D<double> gainKF = transpose(m2LU.solve(m1_T));
//
//		if(gainKF.dim1() == 0 || gainKF.dim2() == 0)
//		{
//			Log::alert("SystemControllerFeedbackLin::doMeasUpdateKF() -- Error computing Kalman gain");
//			return;
//		}
//
//		// \hat{x} = \hat{x} + K (meas - C \hat{x})
//		Array2D<double> err = meas-matmult(mCkf, mStateKF);
//		mStateKF += matmult(gainKF, err);
//
//		// S = (I-KC) S
//		mErrCovKF.inject(matmult(createIdentity(6)-matmult(gainKF, mCkf), mErrCovKF));
//
//		// this is to ensure that mErrCovKF always stays symmetric even after small rounding errors
//		mErrCovKF = 0.5*(mErrCovKF+transpose(mErrCovKF));
//
////		// update bias and force scaling estimates
////		if(mLastAttBiasUpdateTime.getMS() == 0)
////			mLastAttBiasUpdateTime.setTime(); // this will effectively cause dt=0
////		double dt = mLastAttBiasUpdateTime.getElapsedTimeUS()/1.0e6;
////		if(dt < 0.1)
////		{
////			mAttBias[0][0] += mAttBiasAdaptRate[0]*dt*err[1][0];
////			mAttBias[1][0] += mAttBiasAdaptRate[1]*dt*(-err[0][0]);
////		}
////		if(mLastForceGainUpdateTime.getMS() == 0)
////			mLastForceGainUpdateTime.setTime();
////		dt = mLastForceGainUpdateTime.getElapsedTimeUS()/1.0e6;
////		if(meas[2][0] > 0.4 && dt < 0.1) // doing this too low runs into problems with ground effect
////			mForceGain += mForceGainAdaptRate*dt*err[2][0];
////		mLastAttBiasUpdateTime.setTime();
////		mLastForceGainUpdateTime.setTime();
////
////		{
////			String str1 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_ATT_BIAS+"\t";
////			for(int i=0; i<mAttBias.dim1(); i++)
////				str1 = str1+mAttBias[i][0]+"\t";
////			mQuadLogger->addLine(str1,LOG_FLAG_STATE);
////
////			String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_FORCE_GAIN+"\t";
////			str2 = str2+mForceGain+"\t";
////			mQuadLogger->addLine(str2,LOG_FLAG_STATE);
////		}
//
//	}

	// This function needs to be called inside of a locked
	void Observer_Translational::doMeasUpdateKF_velOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
	{
		if(norm2(meas) > 10)
			return; // screwy measuremnt
		Array2D<double> C(3,6,0.0);
		C[0][3] = C[1][4] = C[2][5] = 1;
		Array2D<double> C_T = transpose(C);
		Array2D<double> m1_T = transpose(matmult(errCov, C_T));
		Array2D<double> m2_T = transpose(matmult(C, matmult(errCov, C_T)) + measCov);

		// I need to solve K = m1*inv(m2) which is the wrong order
		// so solve K^T = inv(m2^T)*m1^T
// The QR solver doesn't seem to give stable results. When I used it the resulting errCov at the end
// of this function was no longer symmetric
//		JAMA::QR<double> m2QR(m2_T); 
//	 	Array2D<double> gainKF = transpose(m2QR.solve(m1_T));
		JAMA::LU<double> m2LU(m2_T);
		Array2D<double> gainKF = transpose(m2LU.solve(m1_T));

		if(gainKF.dim1() == 0 || gainKF.dim2() == 0)
		{
			Log::alert("SystemControllerFeedbackLin::doMeasUpdateKF() -- Error computing Kalman gain");
			return;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-matmult(C, state);
		state += matmult(gainKF, err);

		// S = (I-KC) S
		errCov.inject(matmult(createIdentity(6)-matmult(gainKF, C), errCov));

		// this is to ensure that errCov always stays symmetric even after small rounding errors
		errCov = 0.5*(errCov+transpose(errCov));
	}

	// This function needs to be called inside of a locked
	void Observer_Translational::doMeasUpdateKF_posOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
	{
		Array2D<double> C(3,6,0.0);
		C[0][0] = C[1][1] = C[2][2] = 1;
		Array2D<double> C_T = transpose(C);
		Array2D<double> m1_T = transpose(matmult(errCov, C_T));
		Array2D<double> m2_T = transpose(matmult(C, matmult(errCov, C_T)) + measCov);

		// I need to solve K = m1*inv(m2) which is the wrong order
		// so solve K^T = inv(m2^T)*m1^T
// The QR solver doesn't seem to give stable results. When I used it the resulting errCov at the end
// of this function was no longer symmetric
//		JAMA::QR<double> m2QR(m2_T); 
//	 	Array2D<double> gainKF = transpose(m2QR.solve(m1_T));
		JAMA::LU<double> m2LU(m2_T);
		Array2D<double> gainKF = transpose(m2LU.solve(m1_T));

		if(gainKF.dim1() == 0 || gainKF.dim2() == 0)
		{
			Log::alert("SystemControllerFeedbackLin::doMeasUpdateKF() -- Error computing Kalman gain");
			return;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-matmult(C, state);
		state += matmult(gainKF, err);

		// S = (I-KC) S
		errCov.inject(matmult(createIdentity(6)-matmult(gainKF, C), errCov));

		// this is to ensure that errCov always stays symmetric even after small rounding errors
		errCov = 0.5*(errCov+transpose(errCov));
	}

	void Observer_Translational::onObserver_AngularUpdated(shared_ptr<DataVector> attData, shared_ptr<DataVector> angularVelData)
	{
		Array2D<double> att = attData->data;
		// third column of orientation matrix, i.e. R*e3
		double s1 = sin(att[2][0]); double c1 = cos(att[2][0]);
		double s2 = sin(att[1][0]-mAttBias[1][0]); double c2 = cos(att[1][0]-mAttBias[1][0]);
		double s3 = sin(att[0][0]-mAttBias[0][0]); double c3 = cos(att[0][0]-mAttBias[0][0]);
		Array2D<double> r(3,1);
		r[0][0] = s1*s3+c1*c3*s2;
		r[1][0] = c3*s1*s2-c1*s3;
		r[2][0] = c2*c3;

		shared_ptr<DataVector> dirData(new DataVector());
		dirData->type = DATA_TYPE_THRUST_DIR;
		dirData->timestamp = attData->timestamp;
		dirData->data = r.copy();

		mMutex_att.lock();
		mThrustDirBuffer.push_back(dirData);
		mMutex_att.unlock();
	}

	void Observer_Translational::onNewCommStateVicon(Collection<float> const &data)
	{
		Time now;
		Array2D<double> pos(3,1);
		for(int i=0; i<3; i++)
			pos[i][0] = data[i+6];
		pos = matmult(mRotViconToPhone, pos);
// ////////////////////// HACK ///////////////////////
// pos[0][0] -= 0.1;
// ////////////////////// HACK ///////////////////////
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

		shared_ptr<DataVector> posData(new DataVector);
		posData->type = DATA_TYPE_VICON_POS;
		posData->timestamp.setTime(now);
		posData->data = pos.copy();

		shared_ptr<DataVector> velData(new DataVector);
		velData->type = DATA_TYPE_VICON_VEL;
		velData->timestamp.setTime(now);
		velData->data = vel.copy();

		shared_ptr<Data> heightData = shared_ptr<Data>(new Data);
		heightData->type = DATA_TYPE_VICON_HEIGHT;
		heightData->timestamp.setTime(now);
		heightData->data = pos[2][0];

		mMutex_data.lock();
		mLastViconPos.inject(pos);
		mLastViconVel.inject(vel);

		mViconPosBuffer.push_back(posData);
		mViconVelBuffer.push_back(velData);
		mHeightDataBuffer.push_back(heightData);

		mLastPosReceiveTime.setTime(now);
		mMutex_data.unlock();
		mMutex_meas.unlock();


		{
			String s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_RECEIVE_VICON+"\t";
			for(int i=0; i<data.size(); i++)
				s = s+data[i]+"\t";
			mQuadLogger->addLine(s, LOG_FLAG_STATE);
		}

		mNewViconPosAvailable = true;
	}

	void Observer_Translational::onNewCommMass(float m)
	{
		mMutex_data.lock();
		mMass = m;
		mMutex_data.unlock();
	}

	void Observer_Translational::onNewCommForceGain(float k)
	{
		mMutex_data.lock();
		mForceGainReset = k;
		mForceGain = k;
		Log::alert(String()+"Force gain updated: \t"+mForceGain);
		mMutex_data.unlock();
	}

	void Observer_Translational::onNewCommAttBias(float roll, float pitch, float yaw)
	{
		mMutex_data.lock();
		mAttBiasReset[0][0] = roll;
		mAttBiasReset[1][0] = pitch;
		mAttBiasReset[2][0] = yaw;
		mAttBias.inject(mAttBiasReset);
		printArray("att bias: \t",transpose(mAttBias));
		mMutex_data.unlock();
	}

	void Observer_Translational::onNewCommAttBiasAdaptRate(Collection<float> const &rate)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
			mAttBiasAdaptRate[i] = rate[i];
		mMutex_data.unlock();
		{
			String s = "Att bias adapt rate updated: ";
			for(int i=0; i<rate.size(); i++)
				s = s+rate[i]+"\t";
			Log::alert(s);
		}
	}

	void Observer_Translational::onNewCommForceGainAdaptRate(float rate)
	{
		mMutex_data.lock();
		mForceGainAdaptRate = rate;
		Log::alert(String()+"force gain adapt rate: \t"+mForceGainAdaptRate);
		mMutex_data.unlock();
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

	void Observer_Translational::onNewCommBarometerZeroHeight(float h)
	{
		Log::alert(String()+"Barometer zero height set to " + h + " m");
		mMutex_meas.lock();
		mZeroHeight = h;
		mMutex_meas.unlock();
		return;
	}

	void Observer_Translational::onAttitudeThrustControllerCmdsSent(double const cmds[4])
	{
		shared_ptr<DataVector> data(new DataVector());
		data->type = DATA_TYPE_MOTOR_CMDS;
		data->data = Array2D<double>(4,1,0.0);
		for(int i=0; i<4; i++)
			data->data[i][0] = cmds[i];

		mMutex_cmds.lock();
		mMotorCmdsBuffer.push_back(data);
		mMutex_cmds.unlock();
	}

	void Observer_Translational::onNewSensorUpdate(shared_ptr<Data> const &data)
	{
		switch(data->type)
		{
			case DATA_TYPE_PRESSURE:
			{
				// equation taken from wikipedia
				double pressure = data->data;
				double Rstar = 8.31432; // N·m /(mol·K)
				double Tb = 288.15; // K
				double g0 = 9.80665; // m/s^2
				double M = 0.0289644; // kg/mol
				double Pb = 1013.25; // milliBar

				mMutex_phoneTempData.lock();
				if(mPhoneTempData == NULL) 
				{
					mMutex_phoneTempData.unlock();
					return;
				}
				mPhoneTempData->lock();
				float tmuTemp = mPhoneTempData->tmuTemp;
				mPhoneTempData->unlock();
				mMutex_phoneTempData.unlock();
				double k = (999.5-1000.0)/(45.0-37.0); // taken from experimental data
				double pressComp = pressure-k*(tmuTemp-37.0);

				mMutex_meas.lock();
				double h = -Rstar*Tb/g0/M*log(pressure/Pb);
				double hComp = -Rstar*Tb/g0/M*log(pressComp/Pb);

				if(mLastBarometerMeasTime.getMS() > 0)
				{
					double dt = mLastBarometerMeasTime.getElapsedTimeUS()/1.0e6;
					if(dt > 1.0e-3)
						mBarometerHeightState[1][0] = 1.0/dt*(h-mZeroHeight-mBarometerHeightState[0][0]);
					else
						mBarometerHeightState[1][0] = mBarometerHeightState[1][0];
				}
				else
					mBarometerHeightState[1][0] = 0;
				mBarometerHeightState[0][0]= h-mZeroHeight;
				mMutex_meas.unlock();

//				mDoMeasUpdate_zOnly = true;
				if(mQuadLogger != NULL)
				{
					String s = String() + mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_BAROMETER_HEIGHT+"\t" + h + "\t" + hComp;
					mQuadLogger->addLine(s,LOG_FLAG_PC_UPDATES);
				}
			}
			break;
			case DATA_TYPE_PHONE_TEMP:
			{
				mMutex_phoneTempData.lock();
				mPhoneTempData = static_pointer_cast<DataPhoneTemp>(data);
				mMutex_phoneTempData.unlock();
			}
			break;
		}
	}

	void Observer_Translational::onImageProcessed(shared_ptr<ImageMatchData> const data)
	{
		mMutex_imageData.lock();
//		data.copyTo(mImageData);
		mImageMatchData = data;
		mMutex_imageData.unlock();

		mNewImageResultsReady = true;
	}

	void Observer_Translational::onImageTargetFound(shared_ptr<ImageTargetFindData> const data)
	{
//		double nomLength = 0.111; // m
		double nomRadius = 0.097; // m
		double f = data->imgData->focalLength;

		Array2D<double> att = data->imgData->att.copy();
		Array2D<double> R = createRotMat_ZYX(att[2][0], att[1][0], att[0][0]);
		Array2D<double> R_T = transpose(R);

		float cx = data->imgData->img->cols/2.0;
		float cy = data->imgData->img->rows/2.0;
		Array2D<double> point(3,1);
		point[0][0] = -(data->circleBlobs[0].locationCorrected.x-cx);
		point[1][0] = -(data->circleBlobs[0].locationCorrected.y-cx);
		point[2][0] = -f;
		point = matmult(R_T, matmult(mRotCamToPhone, point));

		double radius = data->circleBlobs[0].radiusCorrected;
		double z = nomRadius*f/radius;
		double x = point[0][0]/f*abs(z);
		double y = point[1][0]/f*abs(z);

		Array2D<double> pos(3,1);
		pos[0][0] = x; 
		pos[1][0] = y; 
		pos[2][0] = z;
		mMutex_meas.lock();
///////////////// HACK ////////////////////
pos[2][0] = mLastViconPos[2][0];
///////////////// HACK ////////////////////
		bool doLog = false;
		mMutex_data.lock();
		if(!mHaveFirstCameraPos)
			mViconCameraOffset.inject(mLastViconPos-pos);
		pos = pos+mViconCameraOffset;
		if( !mHaveFirstCameraPos ||
			(mHaveFirstCameraPos && 
			 norm2(mLastCameraPos-pos) < 0.2) &&
			 norm2(submat(mStateKF,0,2,0,0)-pos) < 0.5
			)
		{
			mHaveFirstCameraPos = true;
			double dt = mLastCameraPosTime.getElapsedTimeUS()/1.0e6;
			if(dt < 0.2)
				mLastCameraVel.inject( 1.0/dt*(pos-mLastCameraPos));
			else
				for(int i=0; i<3; i++)
					mLastCameraVel[i][0] = 0;
			mLastCameraPos.inject(pos);
			mLastCameraPosTime.setTime();

			shared_ptr<DataVector> posData(new DataVector);
			posData->type = DATA_TYPE_CAMERA_POS;
			posData->timestamp.setTime(data->imgData->timestamp);
			posData->data = mLastCameraPos.copy();
			mCameraPosBuffer.push_back(posData);

			shared_ptr<DataVector> velData(new DataVector);
			velData->type = DATA_TYPE_CAMERA_VEL;
			velData->timestamp.setTime(data->imgData->timestamp);
			velData->data = mLastCameraVel.copy();
			mCameraVelBuffer.push_back(velData);

			doLog = true;
		}
		mMutex_data.unlock();
		
		mMutex_meas.unlock();

		if(doLog)
		{
			String s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_CAMERA_POS+"\t";
			for(int i=0; i<pos.dim1(); i++)
				s = s+pos[i][0]+"\t";
			mQuadLogger->addLine(s, LOG_FLAG_CAM_RESULTS);
		}

		mNewCameraPosAvailable = true;
	}

	void Observer_Translational::onNewCommUseIbvs(bool useIbvs)
	{
		mUseIbvs = useIbvs;
		String s;
		if(useIbvs)
			s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_IBVS_ENABLED+"\t";
		else
			s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_IBVS_DISABLED+"\t";
		mQuadLogger->addLine(s, LOG_FLAG_PC_UPDATES);
	}

	
	void Observer_Translational::rebuildState(Time const &startTime)
	{
		mMutex_data.lock();
		list<shared_ptr<DataVector> >::const_iterator posIter, velIter, thrustDirIter, opticFlowVelIter;
		list<shared_ptr<Data> >::const_iterator thrustIter;

		list<shared_ptr<DataVector> > *posBuffer, *velBuffer;
		if(mUseViconPos)
		{
			posBuffer = &mViconPosBuffer;
			velBuffer = &mViconVelBuffer;
		}
		else
		{
			posBuffer = &mCameraPosBuffer;
			velBuffer = &mCameraVelBuffer;
		}
		// These iters point to the last data point with iter->timestamp < startTime
		posIter = Data::findIndexReverse(startTime, *posBuffer);
		velIter = Data::findIndexReverse(startTime, *velBuffer);
		thrustDirIter = Data::findIndexReverse(startTime, mThrustDirBuffer);
		thrustIter = Data::findIndexReverse(startTime, mThrustBuffer);
		opticFlowVelIter = Data::findIndexReverse(startTime, mOpticFlowVelBuffer); 

		if( thrustIter == mThrustBuffer.end())
		{
			mMutex_data.unlock();
			Log::alert("Observer_Translational::rebuildState() -- I don't want to be here");
			return; // There might be something better I can do here, but hopefully this case is rare
		}
		
		// interpolate state and errcov to the thrust data time (which we'll use as our starting point)
		Time curTime( (*thrustIter)->timestamp);
		if(curTime < mStateBuffer.front()->timestamp)
		{
			mStateKF.inject(Array2D<double>(mStateKF.dim1(), mStateKF.dim2(), 0.0));
			mErrCovKF.inject(Array2D<double>(mErrCovKF.dim1(), mErrCovKF.dim2(), 0.0));
		}
		else
		{
			mStateKF.inject( Data::interpolate(curTime, mStateBuffer) );
			mErrCovKF.inject( Data::interpolate(curTime, mErrCovKFBuffer) );
		}

		// clear out state and errcov information that we are about to replace
		Data::truncate(curTime, mStateBuffer);
		Data::truncate(curTime, mErrCovKFBuffer);

		list<shared_ptr<DataVector> >::const_iterator thrustDirIterNext;
		thrustDirIterNext = thrustDirIter; thrustDirIter++;

		double thrust, dt;
		Array2D<double> thrustDir(3,1,0.0), posMeas(3,1,0.0), velMeas(3,1,0.0);
//		posIter++; // this should be the first measurement after curTime 
//		velIter++;
//		opticFlowVelIter++;
		Time nextTime;
		bool doTimeUpdate = false;
		Array2D<double> r(3,1), accel(3,1);
		while(	thrustIter != mThrustBuffer.end() || 
				thrustDirIter != mThrustDirBuffer.end() || 
				posIter != posBuffer->end() ||
				velIter != velBuffer->end())
		{
			doTimeUpdate = false;
			if(thrustIter != mThrustBuffer.end())
			{
				doTimeUpdate = true;
				thrust = (*thrustIter)->data;
				thrustIter++;

				// updates these iters so thrustDirIter is before thrustIter and thrustDirIterNext is after thrustIter
				while((*thrustDirIterNext)->timestamp < curTime && thrustDirIterNext != mThrustDirBuffer.end())
				{ thrustDirIter++; thrustDirIterNext++; }

				if(thrustDirIter == mThrustDirBuffer.end())
				{
					thrustDir[0][0] = 0;
					thrustDir[1][0] = 0;
					thrustDir[2][0] = 1;
				}
				else if(thrustDirIterNext == mThrustDirBuffer.end())
					thrustDir.inject((*thrustDirIter)->data);
				else
				{
				thrustDir.inject(Data::interpolate(curTime, **thrustDirIter, **thrustDirIterNext));
				}
			}
			else if(thrustDirIter != mThrustDirBuffer.end())
			{ // thrust will remain as set before
				doTimeUpdate = true;
				thrustDir.inject( (*thrustDirIter)->data);
				thrustDirIter++;
			}
			else if(opticFlowVelIter != mOpticFlowVelBuffer.end())
			{
				curTime.setTime( (*opticFlowVelIter)->timestamp);
				nextTime.setTime( (*opticFlowVelIter)->timestamp);
			}
			else if(velIter != velBuffer->end())
			{
				curTime.setTime( (*velIter)->timestamp);
				nextTime.setTime( (*velIter)->timestamp);
			}
			else if(posIter != posBuffer->end())
			{
				curTime.setTime( (*posIter)->timestamp );
				nextTime.setTime( (*posIter)->timestamp );
			}
			else
			{
				Log::alert("Rebuild state --> I shouldn't be here!!!!!!#!!!");
				continue; // I shouldn't ever get here
			}

			if(thrustIter != mThrustBuffer.end())
				nextTime.setTime((*thrustIter)->timestamp);
			else if(thrustDirIter != mThrustDirBuffer.end())
				nextTime.setTime((*thrustDirIter)->timestamp);

			if(doTimeUpdate)
			{
				if(thrust == 0)
				{
					accel[0][0] = accel[1][0] = accel[2][0] = 0;
				}
				else
				{
					accel.inject(thrust/mMass*thrustDir);
					accel[2][0] -= GRAVITY;
				}

				dt = Time::calcDiffNS( curTime, nextTime)/1.0e9;
				for(int i=0; i<3; i++)
				{
					mAkf[i][i+3] = dt;
					mAkf_T[i+3][i] = dt;
					mBkf[i+3][i] = dt;
				}
				doTimeUpdateKF(accel, mAkf, mAkf_T, mBkf, mStateKF, mErrCovKF, mDynCov);
//Log::alert(String()+"dt: "+dt);
//printArray("accel:\t",transpose(accel));
//printArray("mStateKF:\t",transpose(mStateKF));
			}

			if( posIter != posBuffer->end() && (*posIter)->timestamp <= curTime)
			{
				posMeas.inject( (*posIter)->data);
				doMeasUpdateKF_posOnly(posMeas, mPosMeasCov, mStateKF, mErrCovKF);

				posIter++;
			}
			if( velIter != velBuffer->end() && (*velIter)->timestamp <= curTime)
			{
				velMeas.inject( (*velIter)->data);
				doMeasUpdateKF_velOnly(velMeas, mVelMeasCov, mStateKF, mErrCovKF);

				velIter++;
			}
			if( opticFlowVelIter != mOpticFlowVelBuffer.end() && (*opticFlowVelIter)->timestamp <= curTime)
			{
				velMeas.inject( (*opticFlowVelIter)->data );
				doMeasUpdateKF_velOnly(velMeas, mVelMeasCov, mStateKF, mErrCovKF);

				opticFlowVelIter++;
			}

			shared_ptr<DataVector> stateData = shared_ptr<DataVector>(new DataVector());
			stateData->type = DATA_TYPE_STATE_TRAN;
			stateData->timestamp.setTime(nextTime);
			stateData->data = mStateKF.copy();
			mStateBuffer.push_back(stateData);

			shared_ptr<DataVector> errCovData = shared_ptr<DataVector>(new DataVector());
			errCovData->type = DATA_TYPE_KF_ERR_COV;
			errCovData->timestamp.setTime(nextTime);
			errCovData->data = mErrCovKF.copy();
			mErrCovKFBuffer.push_back(errCovData);

			curTime.setTime(nextTime);
		}

		mMutex_data.unlock();
	}
} // namespace Quadrotor
} // namespace ICSL
