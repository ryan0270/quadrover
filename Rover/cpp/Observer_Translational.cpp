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
		mDynCov(6,6,0.0),
		mErrCovKF(6,6,0.0),
		mStateKF(6,1,0.0),
		mAttBias(3,1,0.0),
		mAttBiasReset(3,1,0.0),
		mAttBiasAdaptRate(3,0.0),
		mAttitude(3,1,0.0),
		mLastMeas(6,1,0.0),
		mBarometerHeightState(2,1,0.0),
		mOpticFlowVel(3,1,0.0)
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
		mDynCov.inject(0.02*0.02*createIdentity(6));
		mDynCov[5][5] *= 10;
		mErrCovKF.inject(1e-4*createIdentity(6));

		mAkf_T.inject(transpose(mAkf));
		mCkf_T.inject(transpose(mCkf));
		mForceGainAdaptRate = 0;

		for(int i=0; i<4; i++)
			mMotorCmds[i] = 0;

		mZeroHeight = 76;
		
		mDoMeasUpdate = false;
		mDoMeasUpdate_posOnly = false;

		mNewImageResultsReady = false;

		mPhoneTempData = NULL;
		mImageMatchData = NULL;

		mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
								 createRotMat(0,(double)PI));
		mRotPhoneToCam = transpose(mRotCamToPhone);

		mFlowCalcDone = true;
		mNewOpticFlowReady = false;

		mMotorOn = false;
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

		System sys;
		Time lastUpdateTime;
		Array2D<double> measTemp(6,1);
		Array2D<double> r(3,1);
		Array2D<double> accel(3,1);
		Array2D<double> pos(3,1),vel(3,1);
		double s1, s2, s3, c1, c2, c3;
		double dt;
		Time lastBattTempTime;
		Array2D<double> flowVel(3,1,0.0);
		Array2D<double> errCov(12,1,0.0);
		while(mRunning)
		{
			double thrust = 0;
			mMutex_cmds.lock();
			for(int i=0; i<4; i++)
				thrust += mForceGain*mMotorCmds[i];
			mMutex_cmds.unlock();

			if(mMotorOn)
			{
				// third column of orientation matrix, i.e. R*e3
				mMutex_att.lock();
				s1 = sin(mAttitude[2][0]); c1 = cos(mAttitude[2][0]);
				s2 = sin(mAttitude[1][0]-mAttBias[1][0]); c2 = cos(mAttitude[1][0]-mAttBias[1][0]);
				s3 = sin(mAttitude[0][0]-mAttBias[0][0]); c3 = cos(mAttitude[0][0]-mAttBias[0][0]);
				mMutex_att.unlock();
				r[0][0] = s1*s3+c1*c3*s2;
				r[1][0] = c3*s1*s2-c1*s3;
				r[2][0] = c2*c3;

				mMutex_data.lock();
				accel.inject(thrust/mMass*r);
				mMutex_data.unlock();
				accel[2][0] -= GRAVITY;
			}
			else 
			{
				for(int i=0; i<accel.dim1(); i++)
					accel[i][0] = 0;
				mMutex_data.lock();
				mAttBias.inject(mAttBiasReset);
				mForceGain = mForceGainReset;
				mMutex_data.unlock();
			}

			dt = lastUpdateTime.getElapsedTimeUS()/1.0e6;
			doTimeUpdateKF(accel, dt);
			lastUpdateTime.setTime();


//			if(mDoMeasUpdate)
//			{
//				mMutex_meas.lock(); measTemp.inject(mLastMeas); mMutex_meas.unlock();
//				doMeasUpdateKF(measTemp);
//			}
			if(mNewOpticFlowReady)
			{
				mMutex_meas.lock();
				Array2D<double> vel = mOpticFlowVel.copy();
				mMutex_meas.unlock();

				doMeasUpdateKF_velOnly(vel);
			}
			if(mDoMeasUpdate_posOnly)
			{
				mMutex_meas.lock();
				Array2D<double> pos = submat(mLastMeas,0,2,0,0);
				mMutex_meas.unlock();

				doMeasUpdateKF_posOnly(pos);
			}

			if(mNewImageResultsReady && mFlowCalcDone
					&& mMotorOn)
			{
				// if we're here then the previous thread should already be finished
				flowCalcThread.imageMatchData = mImageMatchData;
				mFlowCalcDone = false;
				flowCalcThread.start();
				mNewImageResultsReady = false;
			}

			mMutex_data.lock();
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

			sys.msleep(5); // maintain a (roughly) 200Hz update rate
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

		mMutex_data.lock();
		double dt = matchData->dt;
		Array2D<double> mu_v = submat(mStateKF,3,5,0,0);
//		Array2D<double> Sn = 30*30*createIdentity(2);
		Array2D<double> Sn = 3e2*3e2*createIdentity(2);
		Array2D<double> SnInv(2,2,0.0);
		SnInv[0][0] = 1.0/Sn[0][0]; SnInv[1][1] = 1.0/Sn[1][1];

		Array2D<double> Sv = submat(mErrCovKF,3,5,3,5);
		JAMA::LU<double> SvLU(Sv);
		Array2D<double> SvInv = SvLU.solve(createIdentity(3));
		double z = mStateKF[2][0];
		z -= 0.060; // offset between markers and camera

//Log::alert("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
//printArray("mErrCovKF: \n", mErrCovKF);
		// Rotate prior velocity information to camera coords
		mu_v = matmult(mRotPhoneToCam, mu_v);
		SvInv = matmult(mRotPhoneToCam, matmult(SvInv, mRotCamToPhone));
		mMutex_data.unlock();

		double cx = matchData->imgData1->img->cols/2.0;
		double cy = matchData->imgData1->img->rows/2.0;
		Array2D<double> A(1,3,0.0);
		Array2D<double> B(3,3,0.0);
		// These rotation matrices are actually in phone coords rather than camera coords
		// but the transform to camera coords will get cancelled out because we 
		// unrotate and then rotate about by the same transform (in addition to the
		// attitude difference)
		Array2D<double> R1 = createRotMat_ZYX(matchData->imgData0->att[2][0], matchData->imgData0->att[1][0], matchData->imgData0->att[0][0]);
		Array2D<double> R2 = createRotMat_ZYX(matchData->imgData1->att[2][0], matchData->imgData1->att[1][0], matchData->imgData1->att[0][0]);
//		R1 = matmult(mRotPhoneToCam,R1);
//		R2 = matmult(mRotPhoneToCam,R2);
		Array2D<double> R1_T = transpose(R1);
		Array2D<double> R2_T = transpose(R2);
		Array2D<double> R = matmult(R1, transpose(R2));
		Array2D<double> q1a(3,1), q2a(3,1);
		Array2D<double> q1(2,1), q2(2,1);
		Array2D<double> Lv(2,3), Lv1(2,3), Lv2(2,3);
		Array2D<double> Lw(2,3), Lw1(2,3), Lw2(2,3);
		Array2D<double> angularVel(3,1,0.0);
		for(int i=0; i<matchData->featurePoints[0].size(); i++)
		{
			q1a[0][0] = matchData->featurePoints[0][i].x-cx;
			q1a[1][0] = matchData->featurePoints[0][i].y-cy;
			q1a[2][0] = matchData->imgData0->focalLength;
			q2a[0][0] = matchData->featurePoints[1][i].x-cx;
			q2a[1][0] = matchData->featurePoints[1][i].y-cy;
			q2a[2][0] = matchData->imgData1->focalLength;
//			// change q2 points to q1 attitude
//			q2a = matmult(R, q2a); 
			q1a = matmult(R1_T, q1a);
			q2a = matmult(R2_T, q2a);

			// project back onto the focal plane
			q1a = matchData->imgData0->focalLength/q1a[2][0]*q1a;
			q2a = matchData->imgData1->focalLength/q2a[2][0]*q2a;

			// back to 2d points
			q1[0][0] = q1a[0][0]; q1[1][0] = q1a[1][0];
			q2[0][0] = q2a[0][0]; q2[1][0] = q2a[1][0];

			// Velocity jacobian
			double f1= matchData->imgData0->focalLength;
			Lv1[0][0] = -f1; Lv1[0][1] = 0; Lv1[0][2] = q1[0][0];
			Lv1[1][0] = 0; Lv1[1][1] = -f1; Lv1[1][2] = q1[1][0];

			double f2 = matchData->imgData1->focalLength;
			Lv2[0][0] = -f2; Lv2[0][1] = 0; Lv2[0][2] = q2[0][0];
			Lv2[1][0] = 0; Lv2[1][1] = -f2; Lv2[1][2] = q2[1][0];

			Lv = Lv1.copy();

			Lw1[0][0] = q1[0][0]*q1[1][0]; Lw1[0][1] = -(1+pow(q1[0][0],2)); Lw1[0][2] = q1[1][0];
			Lw1[1][0] = 1+pow(q1[1][0],2); Lw1[1][1] = -Lw1[0][0];			 Lw1[1][2] = -q1[0][0];
			Lw1 = 1.0/matchData->imgData0->focalLength*Lw1;

			Lw2[0][0] = q2[0][0]*q2[1][0]; Lw2[0][1] = -(1+pow(q2[0][0],2)); Lw2[0][2] = q2[1][0];
			Lw2[1][0] = 1+pow(q2[1][0],2); Lw2[1][1] = -Lw2[0][0];			 Lw2[1][2] = -q2[0][0];
			Lw2 = 1.0/matchData->imgData1->focalLength*Lw2;

			Lw = Lw1.copy();

			angularVel.inject(0.5*(matchData->imgData0->startAngularVel+matchData->imgData1->endAngularVel));

//			A += matmult(transpose(q2-q1-matmult(Lw,matmult(mRotPhoneToCam,angularVel))),matmult(SnInv, Lv));
			A += matmult(transpose(q2-q1),matmult(SnInv, Lv));
			B += matmult(transpose(Lv), matmult(SnInv, Lv));
		}
		Array2D<double> temp1 = (dt/z)*A+matmult(transpose(mu_v), SvInv);
		Array2D<double> temp2 = ((dt*dt)/(z*z))*B+SvInv;
		JAMA::LU<double> temp2_TQR(transpose(temp2));
		Array2D<double> vel = temp2_TQR.solve(transpose(temp1));

		JAMA::LU<double> B_TLU(transpose(B));
		Array2D<double> velLS = z/dt*B_TLU.solve(transpose(A)); // least squares

		// Finally, convert the velocity from camera to phone coords
		vel = matmult(mRotCamToPhone, vel);
		velLS = matmult(mRotCamToPhone, velLS);

		String str = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW+"\t";
		for(int i=0; i<vel.dim1(); i++)
			str = str+vel[i][0]+"\t";
		mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
	
		String str2 = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW_LS+"\t";
		for(int i=0; i<velLS.dim1(); i++)
			str2 = str2+velLS[i][0]+"\t";
		mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);

		mFlowCalcDone = true;
		mNewOpticFlowReady = true;

		mMutex_meas.lock();
		mOpticFlowVel.inject(vel);
		mMutex_meas.unlock();
	}

	void Observer_Translational::setMotorCmds(double const cmds[4])
	{
		mMutex_cmds.lock();
		for(int i=0; i<4; i++)
			mMotorCmds[i] = cmds[i];
		mMutex_cmds.unlock();
	}

	void Observer_Translational::doTimeUpdateKF(TNT::Array2D<double> const &actuator, double dt)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
		{
			mAkf[i][i+3] = dt;
			mAkf_T[i+3][i] = dt;
			mBkf[i+3][i] = dt;
		}
		mStateKF.inject(matmult(mAkf,mStateKF)+matmult(mBkf, actuator));
		mErrCovKF.inject(matmult(mAkf, matmult(mErrCovKF, mAkf_T)) + mDynCov);
		mMutex_data.unlock();
	}

	void Observer_Translational::doMeasUpdateKF(TNT::Array2D<double> const &meas)
	{
		mMutex_data.lock();
		Array2D<double> m1_T = transpose(matmult(mErrCovKF, mCkf_T));
		Array2D<double> m2_T = transpose(matmult(mCkf, matmult(mErrCovKF, mCkf_T)) + mMeasCov);

		// I need to solve K = m1*inv(m2) which is the wrong order
		// so solve K^T = inv(m2^T)*m1^T
// The QR solver doesn't seem to give stable results. When I used it the resulting mErrCovKF at the end
// of this function was no longer symmetric
//		JAMA::QR<double> m2QR(m2_T); 
//	 	Array2D<double> gainKF = transpose(m2QR.solve(m1_T));
		JAMA::LU<double> m2LU(m2_T);
		Array2D<double> gainKF = transpose(m2LU.solve(m1_T));

		if(gainKF.dim1() == 0 || gainKF.dim2() == 0)
		{
			Log::alert("SystemControllerFeedbackLin::doMeasUpdateKF() -- Error computing Kalman gain");
			mMutex_data.unlock();
			return;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-matmult(mCkf, mStateKF);
		mStateKF += matmult(gainKF, err);

		// S = (I-KC) S
		mErrCovKF.inject(matmult(createIdentity(6)-matmult(gainKF, mCkf), mErrCovKF));

		// this is to ensure that mErrCovKF always stays symmetric even after small rounding errors
		mErrCovKF = 0.5*(mErrCovKF+transpose(mErrCovKF));

		// update bias and force scaling estimates
		if(mLastAttBiasUpdateTime.getMS() == 0)
			mLastAttBiasUpdateTime.setTime(); // this will effectively cause dt=0
		double dt = mLastAttBiasUpdateTime.getElapsedTimeUS()/1.0e6;
		mAttBias[0][0] += mAttBiasAdaptRate[0]*dt*err[1][0];
		mAttBias[1][0] += mAttBiasAdaptRate[1]*dt*(-err[0][0]);
		if(mLastForceGainUpdateTime.getMS() == 0)
			mLastForceGainUpdateTime.setTime();
		dt = mLastForceGainUpdateTime.getElapsedTimeUS()/1.0e6;
		if(meas[2][0] > 0.4) // doing this too low runs into problems with ground effect
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

		mDoMeasUpdate = false;
	}

	void Observer_Translational::doMeasUpdateKF_velOnly(TNT::Array2D<double> const &meas)
	{
		mNewOpticFlowReady = false;
		mMutex_data.lock();
		Array2D<double> measCov(3,3);
		measCov[0][0] = mMeasCov[3][3];
		measCov[0][1] = measCov[1][0] = mMeasCov[3][4];
		measCov[0][2] = measCov[2][0] = mMeasCov[3][5];
		measCov[1][1] = mMeasCov[4][4];
		measCov[1][2] = measCov[2][1] = mMeasCov[4][5];
//		measCov[2][2] = 10*mMeasCov[5][5]; // optical flow is especially bad in z
		Array2D<double> C(3,6,0.0);
		C[0][3] = C[1][4] = C[2][5] = 1;
		Array2D<double> C_T = transpose(C);
		Array2D<double> m1_T = transpose(matmult(mErrCovKF, C_T));
		Array2D<double> m2_T = transpose(matmult(C, matmult(mErrCovKF, C_T)) + measCov);

		// I need to solve K = m1*inv(m2) which is the wrong order
		// so solve K^T = inv(m2^T)*m1^T
// The QR solver doesn't seem to give stable results. When I used it the resulting mErrCovKF at the end
// of this function was no longer symmetric
//		JAMA::QR<double> m2QR(m2_T); 
//	 	Array2D<double> gainKF = transpose(m2QR.solve(m1_T));
		JAMA::LU<double> m2LU(m2_T);
		Array2D<double> gainKF = transpose(m2LU.solve(m1_T));

		if(gainKF.dim1() == 0 || gainKF.dim2() == 0)
		{
			Log::alert("SystemControllerFeedbackLin::doMeasUpdateKF() -- Error computing Kalman gain");
			mMutex_data.unlock();
			return;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-matmult(C, mStateKF);
		mStateKF += matmult(gainKF, err);

		// S = (I-KC) S
		mErrCovKF.inject(matmult(createIdentity(6)-matmult(gainKF, C), mErrCovKF));

		// this is to ensure that mErrCovKF always stays symmetric even after small rounding errors
		mErrCovKF = 0.5*(mErrCovKF+transpose(mErrCovKF));

		// update bias and force scaling estimates
		if(mLastAttBiasUpdateTime.getMS() == 0)
			mLastAttBiasUpdateTime.setTime(); // this will effectively cause dt=0
		double dt = min(0.05,mLastAttBiasUpdateTime.getElapsedTimeUS()/1.0e6);
		mAttBias[0][0] += mAttBiasAdaptRate[0]*dt*err[1][0];
		mAttBias[1][0] += mAttBiasAdaptRate[1]*dt*(-err[0][0]);
		mLastAttBiasUpdateTime.setTime();

		{
			String str1 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_ATT_BIAS+"\t";
			for(int i=0; i<mAttBias.dim1(); i++)
				str1 = str1+mAttBias[i][0]+"\t";
			mQuadLogger->addLine(str1,LOG_FLAG_STATE);
		}

		mMutex_data.unlock();
	}

	void Observer_Translational::doMeasUpdateKF_posOnly(TNT::Array2D<double> const &meas)
	{
		mDoMeasUpdate_posOnly = false;
		mMutex_data.lock();
		Array2D<double> measCov = submat(mMeasCov,0,2,0,2);
		Array2D<double> C(3,6,0.0);
		C[0][0] = C[1][1] = C[2][2] = 1;
		Array2D<double> C_T = transpose(C);
		Array2D<double> m1_T = transpose(matmult(mErrCovKF, C_T));
		Array2D<double> m2_T = transpose(matmult(C, matmult(mErrCovKF, C_T)) + measCov);

		// I need to solve K = m1*inv(m2) which is the wrong order
		// so solve K^T = inv(m2^T)*m1^T
// The QR solver doesn't seem to give stable results. When I used it the resulting mErrCovKF at the end
// of this function was no longer symmetric
//		JAMA::QR<double> m2QR(m2_T); 
//	 	Array2D<double> gainKF = transpose(m2QR.solve(m1_T));
		JAMA::LU<double> m2LU(m2_T);
		Array2D<double> gainKF = transpose(m2LU.solve(m1_T));

		if(gainKF.dim1() == 0 || gainKF.dim2() == 0)
		{
			Log::alert("SystemControllerFeedbackLin::doMeasUpdateKF() -- Error computing Kalman gain");
			mMutex_data.unlock();
			return;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-matmult(C, mStateKF);
		mStateKF += matmult(gainKF, err);

		// S = (I-KC) S
		mErrCovKF.inject(matmult(createIdentity(6)-matmult(gainKF, C), mErrCovKF));

		// this is to ensure that mErrCovKF always stays symmetric even after small rounding errors
		mErrCovKF = 0.5*(mErrCovKF+transpose(mErrCovKF));

		// update bias and force scaling estimates
		if(mLastForceGainUpdateTime.getMS() == 0)
			mLastForceGainUpdateTime.setTime(); // this will effectively cause dt=0
		double dt = min(0.40,mLastForceGainUpdateTime.getElapsedTimeUS()/1.0e6);
		mForceGain += mForceGainAdaptRate*err[2][0];
		mLastForceGainUpdateTime.setTime();
		{
			String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_OBSV_TRANS_FORCE_GAIN+"\t";
			str2 = str2+mForceGain+"\t";
			mQuadLogger->addLine(str2,LOG_FLAG_STATE);
		}

		mMutex_data.unlock();
	}

	void Observer_Translational::onObserver_AngularUpdated(Array2D<double> const &att, Array2D<double> const &angularVel)
	{
		mMutex_att.lock();
		mAttitude.inject(att);
		mMutex_att.unlock();
	}

	void Observer_Translational::onNewCommStateVicon(Collection<float> const &data)
	{
		Array2D<double> pos(3,1);
		for(int i=0; i<3; i++)
			pos[i][0] = data[i+6];
		pos = matmult(mRotViconToPhone, pos);
		mMutex_meas.lock();
		Array2D<double> vel(3,1,0.0);
		if(mLastPosReceiveTime.getMS() != 0)
		{
			// This velocity ``measurement'' is very noisy, but it helps to correct some
			// of the bias error that occurs when the attitude estimate is wrong
			double dt = mLastPosReceiveTime.getElapsedTimeUS()/1.0e6;
			if(dt > 1.0e-3) // reduce the effect of noise
				vel.inject(1.0/dt*(pos-submat(mLastMeas,0,2,0,0)));
			else
			{
				for(int i=0; i<3; i++)
					vel[i][0] = mLastMeas[i+3][0];
			}
		}
		for(int i=0; i<3; i++)
			mLastMeas[i][0] = pos[i][0];
		for(int i=3; i<6; i++)
			mLastMeas[i][0] = vel[i-3][0];
		mMutex_meas.unlock();
		mLastPosReceiveTime.setTime();

		{
			String s = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_RECEIVE_VICON+"\t";
			for(int i=0; i<data.size(); i++)
				s = s+data[i]+"\t";
			mQuadLogger->addLine(s, LOG_FLAG_STATE);
		}

//		mDoMeasUpdate = true;
		mDoMeasUpdate_posOnly = true;
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
		mMutex_cmds.lock();
		for(int i=0; i<4; i++)
			mMotorCmds[i] = cmds[i];
		mMutex_cmds.unlock();
	}

	void Observer_Translational::onNewSensorUpdate(shared_ptr<SensorData> const &data)
	{
		switch(data->type)
		{
			case SENSOR_DATA_TYPE_PRESSURE:
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
					return;
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
			case SENSOR_DATA_TYPE_PHONE_TEMP:
			{
				mMutex_phoneTempData.lock();
				mPhoneTempData = static_pointer_cast<SensorDataPhoneTemp>(data);
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
} // namespace Quadrotor
} // namespace ICSL
