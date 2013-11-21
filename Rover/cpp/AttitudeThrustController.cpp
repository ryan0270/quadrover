#include "AttitudeThrustController.h"

namespace ICSL {
namespace Quadrotor {
using namespace TNT;
	AttitudeThrustController::AttitudeThrustController() :
		mCurAngularVel(3,1,0.0),
		mGainAngle(3,1,0.0),
		mGainRate(3,1,0.0),
		mDesAccel(3,1,0.0),
		mRefState(6,1,0.0)
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

		mMotorPlaneBias.setRotMat( matmult(createRotMat(1,-0.0), createRotMat(0,-0.0) ) );

		double refDamping = 1.2;
		double refNaturalFreq = 30;
		mRefB = refDamping*refNaturalFreq;
		mRefC = refNaturalFreq*refNaturalFreq;
//		Array2D<double> A(6,6,0.0);
//		A[0][3] = A[1][4] = A[2][5] = 1;
//		A[3][0] = A[4][1] = A[5][2] = -naturalFreq*naturalFreq;
//		A[3][3] = A[4][4] = A[5][5] = -2*damping*naturalFreq;
//
//		Array2D<double> B(6,3,0.0);
//		B[3][0] = B[4][1] = B[5][2] = naturalFreq*naturalFreq;
//
//		mRefSys.setA(A);
//		mRefSys.setB(B);
//		mRefSys.setC(createIdentity((double)6));
//		mRefSys.setD(Array2D<double>(6,3,0.0));
//		mRefSys.reset();
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
		double dt;
		if(mLastControlTime.getMS() == 0)
			dt = 0;
		else
			dt = mLastControlTime.getElapsedTimeNS()/1.0e9;
		mLastControlTime.setTime();

		mMutex_data.lock();
		double n= norm2(mDesAccel);
		SO3 curMotorAtt = mCurAtt;
		Array2D<double> curEuler = curMotorAtt.getAnglesZYX();
		double c = cos(curEuler[2][0]); double s = sin(curEuler[2][0]);
//c = 1; // don't try to compenstate for yaw error
		double x =  mDesAccel[0][0]*c+mDesAccel[1][0]*s;
		double y = -mDesAccel[0][0]*s+mDesAccel[1][0]*c;
		double desRoll  = -asin(y/n);
		double desPitch =  asin(x/cos(desRoll)/n);
		double desYaw = 0;

		mDesAtt.setRotMat(createRotMat_ZYX(desYaw,desPitch,desRoll));
//		mDesAtt *= mMotorPlaneBias;

		//TODO: this should be based on the reference model
		mThrust = mMass*mDesAccel[2][0]/cos(curEuler[0][0])/cos(curEuler[1][0]);

//		SO3 attErr = mDesAtt.inv()*curMotorAtt;
//		Array2D<double> accel(3,1,0.0);
//		Array2D<double> refRate(3,1,0.0);

		// Simulate the reference system
	  	for(int i=0; i<3; i++)
			mRefState[i][0] += dt*mRefState[i+3][0];

		Array2D<double> desVector(3,1,0.0);
		double desTheta = 0;
// TODO: this call for some reason seems to create excessive CPU load
		mDesAtt.getAngleAxis(desTheta, desVector);
		desVector = desTheta*desVector;
		Array2D<double> accel(3,1);
		for(int i=0; i<3; i++)
		{
			accel[i][0] =	-mRefB*mRefState[i+3][0]
							-mRefC*(mRefState[i][0]-desVector[i][0]);
			mRefState[i+3][0] += dt*accel[i][0];
		}

		Array2D<double> refVector = submat(mRefState,0,2,0,0);
		SO3_LieAlgebra lie(refVector);
		SO3 refAtt = lie.integrate(1);

//accel[0][0] = accel[1][0] = accel[2][0] = 0;
		SO3 attErr = refAtt.inv()*curMotorAtt;
		Array2D<double> refRate = submat(mRefState,3,5,0,0);
	
		Array2D<double> rotMatErr = attErr.getRotMat();
		Array2D<double> rotMatErr_AS = rotMatErr-transpose(rotMatErr);
		Array2D<double> rotErr(3,1);
		rotErr[0][0] = rotMatErr_AS[2][1];
		rotErr[1][0] = rotMatErr_AS[0][2];
		rotErr[2][0] = rotMatErr_AS[1][0];

		Array2D<double> inertia(3,1);
		inertia[0][0] = 0.005;
		inertia[1][0] = 0.005;
		inertia[2][0] = 0.020;
		Array2D<double> torque = -1.0*mGainAngle*rotErr-mGainRate*(mCurAngularVel-refRate)+inertia*accel;
		double cmdRoll = torque[0][0]/mForceScaling/mMotorArmLength/4.0;
		double cmdPitch = torque[1][0]/mForceScaling/mMotorArmLength/4.0;
		double cmdYaw = torque[2][0]/mTorqueScaling/4.0;
		double cmdThrust = mThrust/mForceScaling/4.0;
		mMutex_data.unlock();
		
		Collection<double> cmds(4);
		bool sane = false;
		int cnt = 0;
		while(!sane && cnt < 10)
		{
			cmds[0] = cmdThrust-cmdRoll-cmdPitch+cmdYaw;
			cmds[1] = cmdThrust-cmdRoll+cmdPitch-cmdYaw;
			cmds[2] = cmdThrust+cmdRoll+cmdPitch+cmdYaw;
			cmds[3] = cmdThrust+cmdRoll-cmdPitch-cmdYaw;

			double minCmd = 9999;
			double maxCmd = 0;
			for(int i=0; i<4; i++)
			{
				minCmd = min(minCmd, cmds[i]);
				maxCmd = max(maxCmd, cmds[i]);
			}

			if(minCmd >= 0 && maxCmd < 2048)
				sane = true;
			else
			{
				if( maxCmd-minCmd >= 2048 || (cmdThrust < 1600 && cmdThrust > 400))
				{
					// reduce torque
					double k = 0.9;
					cmdRoll *= k;
					cmdPitch *= k;
					cmdYaw *= k;
				}
				else
				{
					double diff;
					if(maxCmd >= 2048)
						diff = maxCmd-2048+10;
					else // minCmd < 0
						diff = minCmd-10;
					cmdThrust -= diff;
				}
			}

			cnt++;
		}

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
		mMotorInterface->setCommand(motorCmds);
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
			Collection<double> data(4);
			data[0] = cmdRoll;
			data[1] = cmdPitch;
			data[2] = cmdYaw;
			data[3] = cmdThrust;
			mQuadLogger->addEntry(LOG_ID_TORQUE_CMD, data, LOG_FLAG_MOTORS);

			mQuadLogger->addEntry(LOG_ID_MOTOR_CMDS, cmds, LOG_FLAG_MOTORS);

			data.clear();
			data.resize(3);
			data[0] = desRoll;
			data[1] = desPitch;
			data[2] = desYaw;
			mQuadLogger->addEntry(LOG_ID_DES_ATT,data,LOG_FLAG_STATE_DES);

			mQuadLogger->addEntry(LOG_ID_REF_ATTITUDE_SYSTEM_STATE, refAtt, refRate, LOG_FLAG_STATE_DES);
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

		for(int i=0; i<mRefState.dim1(); i++)
			mRefState[i][0] = 0;
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
