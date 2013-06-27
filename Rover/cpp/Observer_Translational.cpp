#include "Observer_Translational.h"
#include "TNT/jama_lu.h"
#include "TNT/jama_qr.h"

using namespace toadlet::egg;
using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{
	Observer_Translational::Observer_Translational() :
		mRotViconToPhone(3,3,0.0),
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
		
		mCkf.inject(createIdentity((double)6));
		mMeasCov[0][0] = mMeasCov[1][1] = mMeasCov[2][2] = 0.01*0.01;
		mMeasCov[3][3] = mMeasCov[4][4] = mMeasCov[5][5] = 0.3*0.3;
		mMeasCov[2][2] = 0.05*0.05;
		mMeasCov[5][5] = 0.5*0.5;
		mPosMeasCov = submat(mMeasCov,0,2,0,2);
		mVelMeasCov = submat(mMeasCov,3,5,3,5);
		mDynCov.inject(0.02*0.02*createIdentity((double)6));
		mDynCov[5][5] *= 10;
		mErrCovKF.inject(1e-4*createIdentity((double)6));

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

		mMotorOn = false;

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mUseCameraPos = false;
		mUseViconPos = true;
		mHaveFirstCameraPos = false;

		mUseIbvs = false;

		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mStateBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mErrCovKFBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mViconPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mViconVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mCameraPosBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mCameraVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mOpticFlowVelBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mHeightDataBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mMotorCmdsBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mThrustDirBuffer));
		mDataBuffers.push_back( (list<shared_ptr<Data<double> > >*)(&mThrustBuffer));

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
		this->join();
//		while(!mDone)
//			System::msleep(10);

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
		uint64 t;
		double thrust=0;
		list<shared_ptr<IData> > events;
		while(mRunning)
		{
			loopTime.setTime();

			mMutex_meas.lock();
			if(mHaveFirstCameraPos && mLastCameraPosTime.getElapsedTimeMS() > 500)
				mHaveFirstCameraPos = false;
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

			// time update since last processed event
			mMutex_cmds.lock(); mMutex_adaptation.lock();
			thrust = 0;
			if(mMotorCmdsBuffer.size() > 0)
				for(int i=0; i<4; i++)
					thrust += mMotorCmdsBuffer.back()->data[i][0];
			else
				thrust = 0;
			thrust *= mForceGain;
			mMutex_adaptation.unlock(); mMutex_cmds.unlock();

			shared_ptr<Data<double> > thrustData(new Data<double>());
			thrustData->type = DATA_TYPE_THRUST;
			if(mMotorCmdsBuffer.size() > 0)
				thrustData->timestamp.setTime(mMotorCmdsBuffer.back()->timestamp);
			if(mMotorOn)
				thrustData->data = thrust;
			else
				thrustData->data = -1; // a bit of a hack
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
				mMutex_adaptation.lock();
				mAttBias.inject(mAttBiasReset);
				mForceGain = mForceGainReset;
				mMutex_adaptation.unlock();
			}
			mMutex_data.unlock();

			dt = lastUpdateTime.getElapsedTimeUS()/1.0e6;
			mMutex_data.lock();
			doTimeUpdateKF(accel, dt, mStateKF, mErrCovKF, mDynCov);
			mMutex_data.unlock();
			lastUpdateTime.setTime();

			// optical flow
			if(mNewImageResultsReady && mFlowCalcDone
//					&& mMotorOn // sometimes the blurry images when sitting close to the ground create artificial matches
					)
			{
				// if we're here then the previous thread should already be finished
				flowCalcThread.imageMatchData = mImageMatchData;
				mFlowCalcDone = false;
				flowCalcThread.start();
				mNewImageResultsReady = false;
			}

			// buffers
			mMutex_data.lock();
			shared_ptr<DataVector<double> > stateData = shared_ptr<DataVector<double> >(new DataVector<double>());
			stateData->type = DATA_TYPE_STATE_TRAN;
			stateData->data = mStateKF.copy();
			mStateBuffer.push_back(stateData);

			shared_ptr<DataVector<double> > errCovData = shared_ptr<DataVector<double> >(new DataVector<double>());
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
	
			t = loopTime.getElapsedTimeUS();
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

		double dt = Time::calcDiffNS( matchData->imgData0->timestamp, matchData->imgData1->timestamp)/1.0e9;
		if(dt < 1e-3)
		{
			mFlowCalcDone = true;
			return;
		}
		mMutex_data.lock();
		Array2D<double> Sn = 300*300*createIdentity((double)2);
		Array2D<double> SnInv(2,2,0.0);
		SnInv[0][0] = 1.0/Sn[0][0]; SnInv[1][1] = 1.0/Sn[1][1];

		Array2D<double> mu_v1 = submat(mStateKF,3,5,0,0);
		Array2D<double> Sv = submat(mErrCovKF,3,5,3,5);
		JAMA::LU<double> SvLU(Sv);
		Array2D<double> SvInv1 = SvLU.solve(createIdentity((double)3));

		mOpticFlowVel.timestamp.setTime(matchData->imgData1->timestamp);
		Time img1Time = matchData->imgData1->timestamp;
		double z = IData::interpolate(img1Time, mHeightDataBuffer);
		z -= 0.060; // offset between markers and camera

		// Rotate prior velocity information to camera coords
		Array2D<double> mu_v = matmult(mRotPhoneToCam, mu_v1);
		Array2D<double> SvInv = matmult(mRotPhoneToCam, matmult(SvInv1, mRotCamToPhone));

		mMutex_data.unlock();

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
			angularVel.inject(matchData->imgData0->angularVel);

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

			mMutex_meas.lock();
			mOpticFlowVel.timestamp.setTime(img1Time);
			mOpticFlowVel.data.inject(vel);
			shared_ptr<DataVector<double> > velData(new DataVector<double>());
			velData->type = DATA_TYPE_OPTIC_FLOW_VEL;
			velData->timestamp.setTime(img1Time);
			velData->data = vel.copy();
			mNewEventsBuffer.push_back(velData);
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
		shared_ptr<DataVector<double> > data(new DataVector<double>());
		data->type = DATA_TYPE_MOTOR_CMDS;
		data->data = Array2D<double>(4,1,0.0);
		for(int i=0; i<4; i++)
			data->data[i][0] = cmds[i];

		mMutex_cmds.lock();
		mMotorCmdsBuffer.push_back(data);
		mMutex_cmds.unlock();
	}

	void Observer_Translational::doTimeUpdateKF(Array2D<double> const &accel, double const &dt, Array2D<double> &state, Array2D<double> &errCov, Array2D<double> const &dynCov)
	{
		for(int i=0; i<3; i++)
			state[i][0] += dt*state[i+3][0];
		for(int i=3; i<6; i++)
			state[i][0] += dt*accel[i-3][0];
		for(int i=0; i<3; i++)
		{
			errCov[i][i] += 2*errCov[i][i+3]*dt+errCov[i+3][i+3]*dt*dt+dynCov[i][i];
			errCov[i][i+3] += errCov[i+3][i+3]*dt+dynCov[i][i+3];
			errCov[i+3][i] += errCov[i+3][i+3]*dt+dynCov[i+3][i];
		}
		for(int i=3; i<6; i++)
			errCov[i][i] += dynCov[i][i];
	}

	void Observer_Translational::doMeasUpdateKF_velOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
	{
		if(norm2(meas) > 10)
			return; // screwy measuremnt

		Array2D<double> gainKF(6,3,0.0);
		for(int i=0; i<3; i++)
		{
			gainKF[i][i] = errCov[i][i+3]/(errCov[i+3][i+3]+measCov[i][i]);
			gainKF[i+3][i] = errCov[i+3][i+3]/(errCov[i+3][i+3]+measCov[i][i]);
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-submat(state,3,5,0,0);
		for(int i=0; i<3; i++)
			state[i][0] += gainKF[i][i]*err[i][0];
		for(int i=3; i<6; i++)
			state[i][0] += gainKF[i][i-3]*err[i-3][0];

		// S = (I-KC) S
		for(int i=0; i<3; i++)
		{
			errCov[i][i] -= gainKF[i][i]*errCov[i][i+3];
			errCov[i][i+3] -= gainKF[i][i]*errCov[i+3][i+3];
			errCov[i+3][i] = errCov[i][i+3];
		}
		for(int i=3; i<6; i++)
			errCov[i][i] -= gainKF[i][i-3]*errCov[i][i];

	}

	void Observer_Translational::doMeasUpdateKF_posOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
	{
		Array2D<double> gainKF(6,3,0.0);
		for(int i=0; i<3; i++)
		{
			gainKF[i][i] = errCov[i][i]/(errCov[i][i]+measCov[i][i]);
			gainKF[i+3][i] = errCov[i][i+3]/(errCov[i][i]+measCov[i][i]);
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err2= meas-submat(state,0,2,0,0);
		for(int i=0; i<3; i++)
			state[i][0] += gainKF[i][i]*err2[i][0];
		for(int i=3; i<6; i++)
			state[i][0] += gainKF[i][i-3]*err2[i-3][0];

		// S = (I-KC) S
		for(int i=3; i<6; i++)
			errCov[i][i] -= gainKF[i][i-3]*errCov[i-3][i];
		for(int i=0; i<3; i++)
		{
			errCov[i][i] -= gainKF[i][i]*errCov[i][i];
			errCov[i][i+3] -= gainKF[i][i]*errCov[i][i+3];
			errCov[i+3][i] = errCov[i][i+3];
		}
	}

	void Observer_Translational::onObserver_AngularUpdated(shared_ptr<DataVector<double> > attData, shared_ptr<DataVector<double> > angularVelData)
	{
		Array2D<double> att = attData->data;
		// third column of orientation matrix, i.e. R*e3
		double s1 = sin(att[2][0]); double c1 = cos(att[2][0]);
		mMutex_adaptation.lock();
		double s2 = sin(att[1][0]-mAttBias[1][0]); double c2 = cos(att[1][0]-mAttBias[1][0]);
		double s3 = sin(att[0][0]-mAttBias[0][0]); double c3 = cos(att[0][0]-mAttBias[0][0]);
		mMutex_adaptation.unlock();
		Array2D<double> r(3,1);
		r[0][0] = s1*s3+c1*c3*s2;
		r[1][0] = c3*s1*s2-c1*s3;
		r[2][0] = c2*c3;

		shared_ptr<DataVector<double> > dirData(new DataVector<double>());
		dirData->type = DATA_TYPE_THRUST_DIR;
		dirData->timestamp = attData->timestamp;
		dirData->data = r.copy();

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

		shared_ptr<DataVector<double> > posData(new DataVector<double>() );
		posData->type = DATA_TYPE_VICON_POS;
		posData->timestamp.setTime(now);
		posData->data = pos.copy();

		shared_ptr<DataVector<double> > velData(new DataVector<double>() );
		velData->type = DATA_TYPE_VICON_VEL;
		velData->timestamp.setTime(now);
		velData->data = vel.copy();

		shared_ptr<Data<double> > heightData = shared_ptr<Data<double> >(new Data<double>());
		heightData->type = DATA_TYPE_VICON_HEIGHT;
		heightData->timestamp.setTime(now);
		heightData->data = pos[2][0];

		mMutex_data.lock();
		mLastViconPos.inject(pos);
		mLastViconVel.inject(vel);

		mNewEventsBuffer.push_back(posData);
		mNewEventsBuffer.push_back(velData);
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
		mMutex_adaptation.lock();
		mAttBiasReset[0][0] = roll;
		mAttBiasReset[1][0] = pitch;
		mAttBiasReset[2][0] = yaw;
		mAttBias.inject(mAttBiasReset);
		printArray("att bias: \t",transpose(mAttBias));
		mMutex_adaptation.unlock();
	}

	void Observer_Translational::onNewCommAttBiasAdaptRate(Collection<float> const &rate)
	{
		mMutex_adaptation.lock();
		for(int i=0; i<3; i++)
			mAttBiasAdaptRate[i] = rate[i];
		mMutex_adaptation.unlock();
		{
			String s = "Att bias adapt rate updated: ";
			for(int i=0; i<rate.size(); i++)
				s = s+rate[i]+"\t";
			Log::alert(s);
		}
	}

	void Observer_Translational::onNewCommForceGainAdaptRate(float rate)
	{
		mMutex_adaptation.lock();
		mForceGainAdaptRate = rate;
		Log::alert(String()+"force gain adapt rate: \t"+mForceGainAdaptRate);
		mMutex_adaptation.unlock();
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
		shared_ptr<DataVector<double> > data(new DataVector<double>());
		data->type = DATA_TYPE_MOTOR_CMDS;
		data->data = Array2D<double>(4,1,0.0);
		for(int i=0; i<4; i++)
			data->data[i][0] = cmds[i];

		mMutex_cmds.lock();
		mMotorCmdsBuffer.push_back(data);
		mMutex_cmds.unlock();
	}

	void Observer_Translational::onNewSensorUpdate(shared_ptr<IData> const &data)
	{
		switch(data->type)
		{
			case DATA_TYPE_PRESSURE:
			{
				// equation taken from wikipedia
				double pressure = static_pointer_cast<Data<double> >(data)->data;
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

//				if(mQuadLogger != NULL)
//				{
//					String s = String() + mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_BAROMETER_HEIGHT+"\t" + h + "\t" + hComp;
//					mQuadLogger->addLine(s,LOG_FLAG_PC_UPDATES);
//				}
			}
			break;
			case DATA_TYPE_PHONE_TEMP:
			{
				mMutex_phoneTempData.lock();
				mPhoneTempData = static_pointer_cast<DataPhoneTemp<double> >(data);
				mMutex_phoneTempData.unlock();
			}
			break;
		}
	}

	void Observer_Translational::onImageProcessed(shared_ptr<ImageMatchData> const data)
	{
		mMutex_imageData.lock();
		mImageMatchData = data;
		mMutex_imageData.unlock();

		mNewImageResultsReady = true;
	}

	void Observer_Translational::onImageTargetFound(shared_ptr<ImageTargetFindData> const data)
	{
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

			shared_ptr<DataVector<double> > posData(new DataVector<double>() );
			posData->type = DATA_TYPE_CAMERA_POS;
			posData->timestamp.setTime(data->imgData->timestamp);
			posData->data = mLastCameraPos.copy();
			mNewEventsBuffer.push_back(posData);

			shared_ptr<DataVector<double> > velData(new DataVector<double>() );
			velData->type = DATA_TYPE_CAMERA_VEL;
			velData->timestamp.setTime(data->imgData->timestamp);
			velData->data = mLastCameraVel.copy();
			mNewEventsBuffer.push_back(velData);

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

	
	Time Observer_Translational::applyData(shared_ptr<IData> const &data)
	{
		Time dataTime = data->timestamp;

		mMutex_data.lock();
		// apply data at the correct point in time
		mStateKF.inject( IData::interpolate(dataTime, mStateBuffer));
		mErrCovKF.inject( IData::interpolate(dataTime, mErrCovKFBuffer));
		switch(data->type)
		{
			case DATA_TYPE_VICON_POS:
				mViconPosBuffer.push_back(static_pointer_cast<DataVector<double> >(data));
				if(mUseViconPos)
				{
					if(static_pointer_cast<DataVector<double> >(data)->data[2][0] > 0.4)
					{ // doing this at low height gets confounded with ground effect forces
						Array2D<double> err = submat(mStateKF,0,2,0,0) - static_pointer_cast<DataVector<double> >(data)->data;
						doForceGainAdaptation(err);
					}
					doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<double> >(data)->data, mPosMeasCov, mStateKF, mErrCovKF);
				}
				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_CAMERA_POS:
				mCameraPosBuffer.push_back(static_pointer_cast<DataVector<double> >(data));
				if(mUseCameraPos)
				{
					Array2D<double> err = submat(mStateKF,0,2,0,0) - static_pointer_cast<DataVector<double> >(data)->data;
					doForceGainAdaptation(err);
					doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<double> >(data)->data, mPosMeasCov, mStateKF, mErrCovKF);
				}
				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_VICON_VEL:
				mViconVelBuffer.push_back(static_pointer_cast<DataVector<double> >(data));
				if(mUseViconPos)
					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double> >(data)->data, 100*mVelMeasCov, mStateKF, mErrCovKF);
				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_CAMERA_VEL:
				mCameraVelBuffer.push_back(static_pointer_cast<DataVector<double> >(data));
				if(mUseCameraPos)
					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double> >(data)->data, 100*mVelMeasCov, mStateKF, mErrCovKF);
				else
				{
					mMutex_data.unlock();
					return dataTime;
				}
				break;
			case DATA_TYPE_OPTIC_FLOW_VEL:
				mOpticFlowVelBuffer.push_back(static_pointer_cast<DataVector<double> >(data));
				doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double> >(data)->data, mVelMeasCov, mStateKF, mErrCovKF);
				break;
			case DATA_TYPE_THRUST_DIR:
				mThrustDirBuffer.push_back(static_pointer_cast<DataVector<double> >(data));
				break;
			default:
				Log::alert(String()+"Observer_Translational::applyData() --> Unknown data type: "+data->type);
		}

		if(mThrustBuffer.size() == 0)
		{
			mMutex_data.unlock();
			return dataTime;
		}

		// now we have to rebuild the state and errcov history
		list<shared_ptr<DataVector<double> > >::iterator posIter, velIter, thrustDirIter, opticFlowVelIter;
		list<shared_ptr<Data<double> > >::iterator thrustIter;

		list<shared_ptr<DataVector<double> > > *posBuffer, *velBuffer;
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

		// These iters point to the last data point with iter->timestamp < dataTime 
		posIter = IData::findIndexReverse(dataTime, *posBuffer);
		velIter = IData::findIndexReverse(dataTime, *velBuffer);
		thrustDirIter = IData::findIndexReverse(dataTime, mThrustDirBuffer);
		thrustIter = IData::findIndexReverse(dataTime, mThrustBuffer);
		opticFlowVelIter = IData::findIndexReverse(dataTime, mOpticFlowVelBuffer); 
		
		// get thrust and thrustDir values at dataTime
		double thrust;
		if(thrustIter != mThrustBuffer.end())
			thrust = (*thrustIter)->data;
		else
			thrust = mThrustBuffer.back()->data;
		Array2D<double> thrustDir(3,1);
		if(thrustDirIter != mThrustDirBuffer.end())
			thrustDir.inject( (*thrustDirIter)->data);
		else if(mThrustDirBuffer.size() > 0)
			thrustDir.inject( mThrustDirBuffer.back()->data);
		else
		{ thrustDir[0][0] = 0; thrustDir[1][0] = 0; thrustDir[2][0] = 1; }

		// construct a sorted list of all events
		list<shared_ptr<IData> > events;
		if(posIter != posBuffer->end()) 
			events.insert(events.end(), ++posIter, posBuffer->end());
		if(velIter != velBuffer->end())
			events.insert(events.end(), ++velIter, velBuffer->end());
		if(thrustDirIter != mThrustDirBuffer.end())
			events.insert(events.end(), ++thrustDirIter, mThrustDirBuffer.end());
		if(thrustIter != mThrustBuffer.end())
			events.insert(events.end(), ++thrustIter, mThrustBuffer.end());
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

		// apply events in order
		list<shared_ptr<IData> >::const_iterator eventIter = events.begin();
		Array2D<double> accel;
		if(thrust != -1)
			accel = thrust/mMass*thrustDir;
		else
			accel = Array2D<double>(3,1,0.0);
		Time lastUpdateTime(dataTime);
		double dt;
		shared_ptr<DataVector<double> > stateData, errCovData;
		while(eventIter != events.end())
		{
			dt = Time::calcDiffNS(lastUpdateTime, (*eventIter)->timestamp)/1.0e9;
			doTimeUpdateKF(accel, dt, mStateKF, mErrCovKF, mDynCov);

			switch((*eventIter)->type)
			{
				case DATA_TYPE_THRUST:
					thrust = static_pointer_cast<Data<double> >(*eventIter)->data;
					if(thrust != -1)
						accel.inject(thrust/mMass*thrustDir);
					else
						for(int i=0; i<3; i++)
							accel[i][0] = 0;
					break;
				case DATA_TYPE_THRUST_DIR:
					thrustDir.inject( static_pointer_cast<DataVector<double> >(*eventIter)->data);
					if(thrust != -1)
						accel.inject(thrust/mMass*thrustDir);
					else
						for(int i=0; i<3; i++)
							accel[i][0] = 0;
					break;
				case DATA_TYPE_VICON_POS:
				case DATA_TYPE_CAMERA_POS:
					doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<double> >(*eventIter)->data, mPosMeasCov, mStateKF, mErrCovKF);
					break;
				case DATA_TYPE_VICON_VEL:
				case DATA_TYPE_CAMERA_VEL:
					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double> >(*eventIter)->data, 100*mVelMeasCov, mStateKF, mErrCovKF);
					break;
				case DATA_TYPE_OPTIC_FLOW_VEL:
					doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<double> >(*eventIter)->data, mVelMeasCov, mStateKF, mErrCovKF);
					break;
				default:
					Log::alert(String()+"Observer_Translational::applyData() --> Unknown data type: "+data->type);
			}

			stateData = shared_ptr<DataVector<double> >(new DataVector<double>());
			stateData->type = DATA_TYPE_STATE_TRAN;
			stateData->timestamp.setTime( (*eventIter)->timestamp);
			stateData->data = mStateKF.copy();
			mStateBuffer.push_back(stateData);

			errCovData = shared_ptr<DataVector<double> >(new DataVector<double>());
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

	void Observer_Translational::doForceGainAdaptation(Array2D<double> const &err)
	{
		mMutex_adaptation.lock();
		if(mLastForceGainUpdateTime.getMS() == 0)
		{
			mLastForceGainUpdateTime.setTime();
			mMutex_adaptation.unlock();
			return;
		}
		double dt = mLastForceGainUpdateTime.getElapsedTimeUS()/1.0e6;
		if(dt < 0.1) // doing this too low runs into problems with ground effect
			mForceGain += mForceGainAdaptRate*dt*err[2][0];
		mLastForceGainUpdateTime.setTime();

		{
			String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_FORCE_GAIN+"\t";
			str2 = str2+mForceGain+"\t";
			mQuadLogger->addLine(str2,LOG_FLAG_STATE);
		}
		mMutex_adaptation.unlock();
	}

	void Observer_Translational::doAttBiasAdaptation(Array2D<double> const &err)
	{
		mMutex_adaptation.lock();
		if(mLastAttBiasUpdateTime.getMS() == 0)
		{
			mLastAttBiasUpdateTime.setTime();
			mMutex_adaptation.unlock();
			return;
		}
		double dt = mLastAttBiasUpdateTime.getElapsedTimeUS()/1.0e6;

		if(dt < 0.1)
		{
			mAttBias[0][0] += mAttBiasAdaptRate[0]*dt*err[1][0];
			mAttBias[1][0] += mAttBiasAdaptRate[1]*dt*(-err[0][0]);
		}

		{
			String str1 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_ATT_BIAS+"\t";
			for(int i=0; i<mAttBias.dim1(); i++)
				str1 = str1+mAttBias[i][0]+"\t";
			mQuadLogger->addLine(str1,LOG_FLAG_STATE);
		}
		mMutex_adaptation.unlock();
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

} // namespace Quadrotor
} // namespace ICSL
