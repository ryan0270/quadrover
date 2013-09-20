#include "AttitudeThrustController.h"
#include "AttitudeThrustControllerListener.h"

namespace ICSL {
namespace Quadrotor {
//using namespace TNT;
	AttitudeThrustController::AttitudeThrustController() :
		mCurAngularVel(3,1,0.0),
		mGainAngle(3,1,0.0),
		mGainRate(3,1,0.0),
		mDesAccel(3,1,0.0)
	{
		mRunning = false;
		mDone = true;
	
		mDoControl = false;
		mPcIsConnected = false;
	
		mLastControlTime.setTimeMS(0);

		mForceScaling = 3e-3;
		mTorqueScaling = 1e-3;

		mThrust = 0;
		mMass = 0;
		
		mMotorArmLength = 0.160;

		mMotorTrim[0] = mMotorTrim[1] = mMotorTrim[2] = mMotorTrim[3] = 0;

		mLastMotorCmds.push_back(0);
		mLastMotorCmds.push_back(0);
		mLastMotorCmds.push_back(0);
		mLastMotorCmds.push_back(0);

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mDesAccel[2][0] = GRAVITY;

		mMotorPlaneBias.setRotMat( matmult(createRotMat(1,-0.06), createRotMat(0,0.0) ) );
	}
	
	AttitudeThrustController::~AttitudeThrustController()
	{
	}
	
	void AttitudeThrustController::shutdown()
	{
		Log::alert("------------------------- AttitudeThrustController shutdown started  --------------------------------------------------");
		mRunning = false;
		while(!mDone)
			System::msleep(10);
	
		Log::alert("------------------------- AttitudeThrustController shutdown done");
	}
	
	void AttitudeThrustController::initialize()
	{
	}
	
	void AttitudeThrustController::run()
	{
		mDone = false;
		mRunning = true;
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			if(mDoControl)
				calcControl();
	
			System::msleep(1);
		}
	
		mDone = true;
	}
	
	void AttitudeThrustController::calcControl()
	{
		mMutex_data.lock();
		double n= norm2(mDesAccel);
		SO3 curMotorAtt = mCurAtt;
		Array2D<double> curEuler = curMotorAtt.getAnglesZYX();
		double c = cos(curEuler[2][0]); double s = sin(curEuler[2][0]);
		double x =  mDesAccel[0][0]*c+mDesAccel[1][0]*s;
		double y = -mDesAccel[0][0]*s+mDesAccel[1][0]*c;
		double desRoll  = -asin(y/n);
		double desPitch =  asin(x/cos(desRoll)/n);
		double desYaw = 0;

		mDesAtt.setRotMat(createRotMat_ZYX(desYaw,desPitch,desRoll));
		mDesAtt *= mMotorPlaneBias;
		mThrust = mMass*mDesAccel[2][0]/cos(curEuler[0][0])/cos(curEuler[1][0]);

		SO3 attErr = mDesAtt.inv()*curMotorAtt;
		Array2D<double> curStateAngular = stackVertical(curEuler,mCurAngularVel);
		mMutex_data.unlock();
	
		Array2D<double> rotMatErr = attErr.getRotMat();
		Array2D<double> rotMatErr_AS = rotMatErr-transpose(rotMatErr);
		Array2D<double> rotErr(3,1);
		rotErr[0][0] = rotMatErr_AS[2][1];
		rotErr[1][0] = rotMatErr_AS[0][2];
		rotErr[2][0] = rotMatErr_AS[1][0];
	
		mMutex_data.lock();
		// do I have an error in my controller derivation and previous implementation?
		// The orignal form isn't giving the right torque
		Array2D<double> torque = -1.0*mGainAngle*rotErr-mGainRate*mCurAngularVel;
//		Array2D<double> torque = mGainAngle*rotErr-mGainRate*mCurAngularVel;
		double cmdRoll = torque[0][0]/mForceScaling/mMotorArmLength/4.0;
		double cmdPitch = torque[1][0]/mForceScaling/mMotorArmLength/4.0;
		double cmdYaw = torque[2][0]/mTorqueScaling/4.0;
		double cmdThrust = mThrust/mForceScaling/4.0;
		mMutex_data.unlock();

		double cmds[4];
		cmds[0] = cmdThrust-cmdRoll-cmdPitch+cmdYaw;
		cmds[1] = cmdThrust-cmdRoll+cmdPitch-cmdYaw;
		cmds[2] = cmdThrust+cmdRoll+cmdPitch+cmdYaw;
		cmds[3] = cmdThrust+cmdRoll-cmdPitch-cmdYaw;
		for(int i=0; i<4; i++)
			cmds[i] = min((double)(1 << 11), max(0.0, cmds[i]));

		Collection<uint16> motorCmds(4);
		if(mPcIsConnected)
		{
			mMutex_data.lock();
			for(int i=0; i<4; i++)
				motorCmds[i] = (uint16)(cmds[i]+mMotorTrim[i]+0.5);
			mMutex_data.unlock();
		}
		else
			for(int i=0; i<4;i++)
				motorCmds[i] = 0;
		mMutex_motorInterface.lock();
		mMotorInterface->sendCommand(motorCmds);
		mLastMotorCmds = motorCmds; // should copy all the data
		mMutex_motorInterface.unlock();
	
		mDoControl = false;

		if(!mMotorInterface->isMotorsEnabled())
			for(int i=0; i<4; i++)
				cmds[i] = 0;
		for(int i=0; i<mListeners.size(); i++)
			mListeners[i]->onAttitudeThrustControllerCmdsSent(cmds);
	
		// Logging
		if(mQuadLogger != NULL)
		{
			String logStr;
			logStr = String();
			for(int i=0; i<4; i++)
				logStr= logStr+cmds[i] + "\t";
			mQuadLogger->addEntry(Time(),LOG_ID_MOTOR_CMDS,logStr,LOG_FLAG_MOTORS);

			logStr =String();
			logStr = logStr+desRoll+"\t"+desPitch+"\t"+desYaw+"\t";
			for(int i=0; i<3; i++)
				logStr = logStr+"0\t";
			mQuadLogger->addEntry(Time(),LOG_ID_DES_ATT,logStr,LOG_FLAG_STATE_DES);
		}
	}
	
	void AttitudeThrustController::onObserver_AngularUpdated(const shared_ptr<SO3Data<double>> &attData, const shared_ptr<DataVector<double>> &angularVelData)
	{
		mMutex_data.lock();
//		mCurAtt.inject(attData->rotation.getAnglesZYX());
		mCurAtt = attData->rotation;
		mCurAngularVel.inject(angularVelData->data);
		mMutex_data.unlock();
	
		mDoControl = true;
	}
	
	void AttitudeThrustController::onTranslationControllerAccelCmdUpdated(const Array2D<double> &accelCmd)
	{
		double n = norm2(accelCmd);
		if(abs(n) > 1e-3)
		{
			mMutex_data.lock();
			mDesAccel.inject(accelCmd);
			mMutex_data.unlock();
		}
		else
		{
			Log::alert("AttitudeThrustController -- Acceleration command was too small");
			mMutex_data.lock();
			mDesAccel[0][0] = mDesAccel[1][0] = 0;
			mDesAccel[1][0] = GRAVITY;
			mMutex_data.unlock();
		}
	}
	
	void AttitudeThrustController::onNewCommMotorOff()
	{
		mPcIsConnected = true;
		Log::alert(String("Turning motors off"));
		mMutex_motorInterface.lock();
		mMotorInterface->enableMotors(false);
		mMutex_motorInterface.unlock();
	}
	
	void AttitudeThrustController::onNewCommMotorOn()
	{
		mPcIsConnected = true;
		mMutex_motorInterface.lock();
		mMotorInterface->enableMotors(true);
		mMutex_motorInterface.unlock();
	}
	
	void AttitudeThrustController::onCommConnectionLost()
	{
		mPcIsConnected = false;
		mMutex_motorInterface.lock();
		mMotorInterface->enableMotors(false);
		mMutex_motorInterface.unlock();
	}
	
	void AttitudeThrustController::onNewCommForceGain(float k)
	{
		mPcIsConnected = true;
		mMutex_data.lock();
		mForceScaling = k;
		mMutex_data.unlock();
		Log::alert(String()+"Force scaling set to "+k);
	}

	void AttitudeThrustController::onNewCommTorqueGain(float k)
	{
		mPcIsConnected = true;
		mMutex_data.lock();
		mTorqueScaling = k;
		mMutex_data.unlock();
		Log::alert(String()+"Torque scaling set to "+k);
	}

	void AttitudeThrustController::onNewCommMotorTrim(const int trim[4])
	{
		mPcIsConnected = true;
		mMutex_data.lock();
		for(int i=0; i<4; i++)
			mMotorTrim[i] = trim[i];;
		{
			String s = "Motor trim updated: ";
			for(int i=0; i<4; i++)
				s = s+mMotorTrim[i]+"\t";
			Log::alert(s);
		}
		mMutex_data.unlock();
	}

	void AttitudeThrustController::onNewCommMass(float m)
	{
		mPcIsConnected = true;
		mMutex_data.lock();
		mMass = m;
		Log::alert(String()+"Mass updated to: "+mMass);
		mMutex_data.unlock();
	}

	void AttitudeThrustController::onNewCommAttitudeGains(const Collection<float> &gains)
	{
		mPcIsConnected = true;
		mMutex_data.lock();
		for(int i=0; i<3; i++)
		{
			mGainAngle[i][0] = gains[i];
			mGainRate[i][0] = gains[i+3];
		}
		mMutex_data.unlock();
		{
			String s = "Attitude control gains updated: ";
			for(int i=0; i<gains.size(); i++)
				s = s+gains[i]+"\t";
			Log::alert(s);
		}
	}

	void AttitudeThrustController::onNewCommMotorArmLength(float k)
	{
		mMutex_data.lock();
		mMotorArmLength = k;
		Log::alert(String()+"Motor arm length updated: "+mMotorArmLength);
		mMutex_data.unlock();
	}

	Array2D<double> AttitudeThrustController::getDesAttitude()
	{
		mMutex_data.lock();
		Array2D<double> temp = mDesAtt.getAnglesZYX();
		mMutex_data.unlock();

		return temp;
	}
} // namespace Quadrotor
} // namespace ICSL
