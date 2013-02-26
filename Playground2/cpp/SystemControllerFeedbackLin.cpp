#include "TNT/jama_lu.h"
//#include "TNT/jama_qr.h"

#include "TNT_Utils.h"

#include "SystemControllerFeedbackLin.h"

using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{
	using namespace std;
	using namespace ICSL;
	using namespace TNT;

	SystemControllerFeedbackLin::SystemControllerFeedbackLin() :
		mCurState(12,1,0.0),
		mDesiredState(12,1,0.0),
		mDesPosAccel(3,1,0.0),
		mGain(12,1,0.0),
		mGainInt(12,1,0.0),
		mErrInt(12,1,0.0),
		mErrIntLimit(12,1,0.0),
		mCurErrorRaw(12,1,0.0),
		mCurErrorFilt(12,1,0.0),
		mRotViconToPhone(3,3,0.0)
	{
		mMass = 0.850;
		mForceScaling = 0.0045;

		mLastControlTime.setTimeMS(0);

	}

	Array2D<double> SystemControllerFeedbackLin::calcControl(Array2D<double> const &curAct)
	{
		if(mLastControlTime.getMS() == 0)
		{
			mLastControlTime.setTime();
			return Array2D<double>(4,1,0.0);
		}
		double dt = mLastControlTime.getElapsedTimeUS()/1.0e6;
		mLastControlTime.setTime();

		mMutex_data.lock();
		Array2D<double> error = mCurState-mDesiredState;

		// restrict angle range
		if(error[0][0] > 180*DEG2RAD)  error[0][0] -= 360*DEG2RAD;
		if(error[0][0] < -180*DEG2RAD) error[0][0] += 360*DEG2RAD;
		if(error[1][0] > 180*DEG2RAD)  error[1][0] -= 360*DEG2RAD;
		if(error[1][0] < -180*DEG2RAD) error[1][0] += 360*DEG2RAD;
		if(error[2][0] > 180*DEG2RAD)  error[2][0] -= 360*DEG2RAD;
		if(error[2][0] < -180*DEG2RAD) error[2][0] += 360*DEG2RAD;
		
		// Integrators
		for(int i=0; i<mErrInt.dim1(); i++)
			mErrInt[i][0] = constrain(mErrInt[i][0] + dt*mGainInt[i][0]*error[i][0],-mErrIntLimit[i][0],mErrIntLimit[i][0]);

		// make these explicit so its easier to read
		double kPhi = mGain[0][0];
		double kTheta = mGain[1][0];
		double kPsi = mGain[2][0];
		double kx = mGain[6][0];
		double ky = mGain[7][0];
		double kz = mGain[8][0];
		double kxdot = mGain[9][0];
		double kydot = mGain[10][0];
		double kzdot = mGain[11][0];

		double sPhi = sin(mCurState[0][0]); double cPhi = cos(mCurState[0][0]);
		double sTheta = sin(mCurState[1][0]); double cTheta = cos(mCurState[1][0]);
		double sPsi = sin(mCurState[2][0]); double cPsi = cos(mCurState[2][0]);

		double uT = -mMass/cPhi/cTheta*(kz*error[8][0]+kzdot*error[11][0]+mErrInt[8][0]-GRAVITY);
		Array2D<double> Binv(3,3);
		Binv[0][0] = 1; Binv[0][1] = 0; 	Binv[0][2] = -sTheta;
		Binv[1][0] = 0; Binv[1][1] = cPhi; 	Binv[1][2] = sPhi*cTheta;
		Binv[2][0] = 0; Binv[2][1] = -sPhi;	Binv[2][2] = cPhi*cTheta;

		Array2D<double> s(3,1);
		s[0][0] = -kPhi*error[0][0]  -sPsi*(kx*error[6][0]+kxdot*error[9][0]+mErrInt[6][0]-mDesPosAccel[0][0])+cPsi*(ky*error[7][0]+kydot*error[10][0]+mErrInt[7][0]-mDesPosAccel[1][0]);
		s[1][0] = -kTheta*error[1][0]-cPsi*(kx*error[6][0]+kxdot*error[9][0]+mErrInt[6][0]-mDesPosAccel[0][0])-sPsi*(ky*error[7][0]+kydot*error[10][0]+mErrInt[7][0]-mDesPosAccel[1][0]);
		s[2][0] = -kPsi*error[2][0]-mErrInt[2][0];

		Array2D<double> w = matmult(Binv,s);
		
		Array2D<double> u(4, 1);
		u[0][0] = uT/mForceScaling/4.0;
		u[1][0] = w[0][0];
		u[2][0] = w[1][0];
		u[3][0] = w[2][0];

		mMutex_data.unlock();

		return u;
	}

	void SystemControllerFeedbackLin::setDesiredState(Array2D<double> const &x)
	{
		mMutex_data.lock();
		if(x.dim1() == mDesiredState.dim1() && x.dim2() == mDesiredState.dim2())
			mDesiredState.inject(x);
		else
			mDesiredState = x.copy();
		mMutex_data.unlock();
	}

	void SystemControllerFeedbackLin::setDesPosAccel(Array2D<double> const &a)
	{
		mMutex_data.lock();
		if(a.dim1() == mDesPosAccel.dim1() && a.dim2() == mDesPosAccel.dim2())
			mDesPosAccel.inject(a);
		else
			mDesPosAccel = a.copy();
		mMutex_data.unlock();
	}

	void SystemControllerFeedbackLin::onNewCommPosControllerGains(float const gainP[12], float const gainI[12], float const gainILimit[12], float mass, float forceScaling)
	{
		String str1, str2, str3;
		mMutex_data.lock();
		for(int i=0; i<12; i++)
		{
			mGain[i][0] = gainP[i];
			mGainInt[i][0] = gainI[i];
			mErrIntLimit[i][0] = gainILimit[i];

			str1 = str1+gainP[i]+"\t";
			str2 = str2+gainI[i]+"\t";
			str3 = str3+gainILimit[i]+"\t";
		}
		mMass = mass;
		mForceScaling = forceScaling;
		mMutex_data.unlock();

		Log::alert("     gainP: \t"+str1);
		Log::alert("     gainI: \t"+str2);
		Log::alert("gainILimit: \t"+str3);
		Log::alert(String()+"      mass: \t"+mass);
		Log::alert(String()+"forceScale: \t"+forceScaling);

		mQuadLogger->addLine(String()+" "+mStartTime.getElapsedTimeMS()+"-700"+str1, PC_UPDATES);
		mQuadLogger->addLine(String()+" "+mStartTime.getElapsedTimeMS()+"-701"+str2, PC_UPDATES);
		mQuadLogger->addLine(String()+" "+mStartTime.getElapsedTimeMS()+"-702"+str3, PC_UPDATES);
		mQuadLogger->addLine(String()+" "+mStartTime.getElapsedTimeMS()+"-703"+mass, PC_UPDATES);
		mQuadLogger->addLine(String()+" "+mStartTime.getElapsedTimeMS()+"-704"+forceScaling, PC_UPDATES);
	}

	void SystemControllerFeedbackLin::onObserver_AngularUpdated(Array2D<double> const &att, Array2D<double> const &angularVel)
	{
		mMutex_data.lock();
		for(int i=0; i<3; i++)
			mCurState[i][0] = att[i][0];
		for(int i=3; i<6; i++)
			mCurState[i][0] = angularVel[i-3][0];
		mMutex_data.unlock();
	}

	void SystemControllerFeedbackLin::onObserver_TranslationalUpdated(Array2D<double> const &pos, Array2D<double> const &vel)
	{
		mMutex_data.lock();
		for(int i=6; i<9; i++)
			mCurState[i][0] = pos[i-6][0];
		for(int i=9; i<12; i++)
			mCurState[i][0] = vel[i-9][0];
		mMutex_data.unlock();
	}
}
}
