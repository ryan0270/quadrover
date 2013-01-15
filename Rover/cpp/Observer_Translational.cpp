#include "Observer_Translational.h"
#include "TNT/jama_lu.h"

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
		mGainKF(6,6,0.0),
		mAttBias(3,1,0.0),
		mAttBiasReset(3,1,0.0),
		mAttitude(3,1,0.0),
		mLastMeas(6,1,0.0)
	{
		mRunning = false;
		mDone = true;

		mMass = 0.850;
		mForceScalingReset = 0.0040;
		mForceScaling = mForceScalingReset;
		
		mLastMeasUpdateTime.setTimeMS(0);
		mLastPosReceiveTime.setTimeMS(0);
		
		mAkf.inject(createIdentity(6));
		mCkf.inject(createIdentity(6));
		mMeasCov[0][0] = mMeasCov[1][1] = mMeasCov[2][2] = 0.01*0.01;
		mMeasCov[3][3] = mMeasCov[4][4] = mMeasCov[5][5] = 0.3*0.3;
		mDynCov.inject(0.02*0.02*createIdentity(6));
		mDynCov[5][5] *= 10;
		mErrCovKF.inject(1e-4*createIdentity(6));

		mAkf_T.inject(transpose(mAkf));
		mCkf_T.inject(transpose(mCkf));

		mGainAttBias = 0;
		mGainForceScaling = 0;

		for(int i=0; i<4; i++)
			mMotorCmds[i] = 0;
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
		Log::alert("------------------------- Observer_Translational shutdown done");
	}

	void Observer_Translational::initialize()
	{
	}

	void Observer_Translational::run()
	{

		mDone = false;
		mRunning = true;
		System sys;
		Time lastUpdateTime;
		Array2D<double> measTemp(6,1);
		Array2D<double> r(3,1);
		Array2D<double> accel(3,1);
		Array2D<double> pos(3,1),vel(3,1);
		double s1, s2, s3, c1, c2, c3;
		double dt;
		while(mRunning)
		{
			double thrust = 0;
			mMutex_cmds.lock();
			for(int i=0; i<4; i++)
				thrust += mForceScaling*mMotorCmds[i];
			mMutex_cmds.unlock();

			if(abs(thrust) >= 1e-6) 
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

				accel.inject(thrust/mMass*r);
				accel[2][0] -= GRAVITY;
			}
			else // assume thrust = 0 means the motors are off and we're sitting still on the ground
			{
				for(int i=0; i<accel.dim1(); i++)
					accel[i][0] = 0;
				mMutex_data.lock();
				mAttBias.inject(mAttBiasReset);
				mForceScaling = mForceScalingReset;
				mMutex_data.unlock();
			}

			dt = lastUpdateTime.getElapsedTimeUS()/1.0e6;
			doTimeUpdateKF(accel, dt);
			lastUpdateTime.setTime();


			if(mDoMeasUpdate)
			{
				mMutex_meas.lock(); measTemp.inject(mLastMeas); mMutex_meas.unlock();
				doMeasUpdateKF(measTemp);
			}

			mMutex_data.lock();
			for(int i=0; i<3; i++)
				pos[i][0] = mStateKF[i][0];
			for(int i=0; i<3; i++)
				vel[i][0] = mStateKF[i+3][0];
			mMutex_data.unlock();

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onObserver_TranslationalUpdated(pos, vel);

			sys.msleep(5); // maintain a (roughly) 200Hz update rate
		}

		mDone = true;
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
//	 	mGainKF.inject(transpose(m2QR.solve(m1_T)));
		JAMA::LU<double> m2LU(m2_T);
		mGainKF.inject(transpose(m2LU.solve(m1_T)));

		if(mGainKF.dim1() == 0 || mGainKF.dim2() == 0)
		{
			Log::alert("SystemControllerFeedbackLin::doMeasUpdateKF() -- Error computing Kalman gain");
			mMutex_data.unlock();
			return;
		}

		// \hat{x} = \hat{x} + K (meas - C \hat{x})
		Array2D<double> err = meas-matmult(mCkf, mStateKF);
		mStateKF += matmult(mGainKF, err);

		// S = (I-KC) S
		mErrCovKF.inject(matmult(createIdentity(6)-matmult(mGainKF, mCkf), mErrCovKF));

		// this is to ensure that mErrCovKF always stays symmetric even after small rounding errors
		mErrCovKF = 0.5*(mErrCovKF+transpose(mErrCovKF));

		// update bias and force scaling estimates
		if(mLastMeasUpdateTime.getMS() == 0)
			mLastMeasUpdateTime.setTime(); // this will effectively cause dt=0
		double dt = mLastMeasUpdateTime.getElapsedTimeUS()/1.0e6;
		mMutex_att.lock();
		double c = cos(mAttitude[2][0]);
		double s = sin(mAttitude[2][0]);
		mMutex_att.unlock();
		mAttBias[0][0] += mGainAttBias*dt*(c*err[1][0]-s*err[0][0]);
		mAttBias[1][0] += mGainAttBias*dt*(-s*err[1][0]-c*err[0][0]);
		mForceScaling += mGainForceScaling*err[2][0];
		mLastMeasUpdateTime.setTime();

		{
			String str1 = String()+mStartTime.getElapsedTimeMS()+"\t-710\t";
			for(int i=0; i<mAttBias.dim1(); i++)
				str1 = str1+mAttBias[i][0]+"\t";
			mQuadLogger->addLine(str1,STATE);

			String str2 = String()+mStartTime.getElapsedTimeMS()+"\t-711\t";
			str2 = str2+mForceScaling+"\t";
			mQuadLogger->addLine(str2,STATE);
		}
		mMutex_data.unlock();

		mDoMeasUpdate = false;
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
			if(dt > 1e-3) // reduce the effect of noise
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
// printArray(transpose(mLastMeas),"mLastMeas: \t");
		mMutex_meas.unlock();
		mLastPosReceiveTime.setTime();

		// use this to signal the run() thread to avoid tying up the 
		// CommManager thread calling this function
		mDoMeasUpdate = true;
	}

	void Observer_Translational::onNewCommMass(float m)
	{
		mMass = m;
	}

	void Observer_Translational::onNewCommForceScaling(float k)
	{
		mForceScalingReset = k;
		mForceScaling = k;
/////////////////////////////// HACK /////////////////////////////////
mForceScalingReset = 0.0035;
mForceScaling = 0.0035;
/////////////////////////////// HACK /////////////////////////////////

	}

	void Observer_Translational::onNewCommAttBias(float roll, float pitch, float yaw)
	{
		mMutex_data.lock();
		mAttBiasReset[0][0] = roll;
		mAttBiasReset[1][0] = pitch;
		mAttBiasReset[2][0] = yaw;
		mAttBias.inject(mAttBiasReset);
		printArray(transpose(mAttBias),"att bias: \t");
		mMutex_data.unlock();
	}

	void Observer_Translational::onNewCommAttBiasGain(float gain)
	{
		mMutex_data.lock();
		mGainAttBias = gain;
		Log::alert(String()+"att bias gain: \t"+mGainAttBias);
		mMutex_data.unlock();
	}

	void Observer_Translational::onNewCommForceScalingGain(float gain)
	{
		mMutex_data.lock();
		mGainForceScaling = gain;
		Log::alert(String()+"force scaling gain: \t"+mGainForceScaling);
		mMutex_data.unlock();
	}

	void Observer_Translational::onNewCommKalmanPosMeasStd(float std)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
			mMeasCov[i][i] = std*std;
		String s = "Pos meas update -- diag(mMeasCov): \t";
		for(int i=0; i<mMeasCov.dim1(); i++)
			s = s+mMeasCov[i][i]+"\t";
		mMutex_data.unlock();
		Log::alert(s);
	}

	void Observer_Translational::onNewCommKalmanVelMeasStd(float std)
	{
		mMutex_data.lock();
		for(int i=3; i<6; i++)
			mMeasCov[i][i] = std*std;
		String s = "Vel meas update -- diag(mMeasCov): \t";
		for(int i=0; i<mMeasCov.dim1(); i++)
			s = s+mMeasCov[i][i]+"\t";
		mMutex_data.unlock();
		Log::alert(s);
	}

	void Observer_Translational::onAttitudeThrustControllerCmdsSent(double const cmds[4])
	{
		mMutex_cmds.lock();
		for(int i=0; i<4; i++)
			mMotorCmds[i] = cmds[i];
		mMutex_cmds.unlock();
	}

} // namespace Quadrotor
} // namespace ICSL
