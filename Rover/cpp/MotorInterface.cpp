#include "MotorInterface.h"

namespace ICSL {
namespace Quadrotor{
using namespace toadlet::egg;
	MotorInterface::MotorInterface()
	{
		mRunning = false;
		mShutdown = true;

		mMotorsEnabled = false;
		mMotorCmds.push_back(0);
		mMotorCmds.push_back(0);
		mMotorCmds.push_back(0);
		mMotorCmds.push_back(0);

		mWaitingForConnection = true;

		mDoMotorWarmup = true;
	}

	MotorInterface::~MotorInterface()
	{
	}

	void MotorInterface::shutdown()
	{
		Log::alert("------------------------ MotorInterface shutdown started");
		mRunning = false;
//		this->join();
		while(!mShutdown)
			System::msleep(10);

		Log::alert("------------------------ MotorInterface shutdown done -------------------------");
	}

	void MotorInterface::initialize()
	{
//		mServerSocket = Socket::ptr(Socket::createTCPSocket());
//		mServerSocket->bind(45670);
//		mServerSocket->listen(1);
//		mServerSocket->setBlocking(false);
//
//		mSocket = Socket::ptr(mServerSocket->accept());

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

//		mMutex_socket.unlock();
	}

	// TODO: I can probably get rid of this
	// since comm has to go through java now
	void MotorInterface::run()
	{
		mShutdown = false;
		mRunning = true;

		// this just monitors the connection
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		Time lastSendTime;
		while(mRunning)
		{
//			if(!isConnected())
//			{
//				mWaitingForConnection = true;
//				mWaitingForConnection = false;
//				mSocket = Socket::ptr(mServerSocket->accept());
//				if(mSocket != NULL)
//					Log::alert("Connected to motors");
//			}
//			else if(!mMotorsEnabled && !mDoMotorWarmup)// && lastSendTime.getElapsedTimeMS() > 20)
//			{
// this shouldn't be necessary anymore
//				// this is just to keep the connection alive
//				Collection<uint16> cmds(4,0);
//				sendCommandForced(cmds);
//			}
//			else if(mDoMotorWarmup)
			if(mDoMotorWarmup)
			{
				mMutex_cmds.lock();
				mMotorCmds.resize(4);
				mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 100;
				mMutex_cmds.unlock();
				if(mMotorWarmupStartTime.getElapsedTimeMS() > 4e3)
				{
					mDoMotorWarmup = false;
					mMotorsEnabled = true;

					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onMotorWarmupDone();
					Log::alert("Warmup done");
				}
			}

			System::msleep(1);
		}

		mMutex_cmds.lock();
		mMotorCmds.resize(4);
		mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 0;
		mMutex_cmds.unlock();

		mShutdown = true;
	}

	void MotorInterface::setCommand(const Collection<uint16> &cmds)
	{
		if(!mMotorsEnabled)
			return;

		mMutex_cmds.lock();
		if(mMotorCmds.size() != cmds.size())
			mMotorCmds.resize(cmds.size());
		for(int i=0; i<cmds.size(); i++)
			mMotorCmds[i] = min((uint16)MAX_MOTOR_CMD, max((uint16)MIN_MOTOR_CMD, cmds[i]));
		mMutex_cmds.unlock();

//		mMutex_socket.lock(); mMutex_data.lock();
//		for(int i=0; i<cmds.size(); i++)
//			mMotorCmds[i] = min((uint16)MAX_MOTOR_CMD, max((uint16)MIN_MOTOR_CMD, cmds[i]));
//		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));
//		mMutex_data.unlock(); mMutex_socket.unlock();
//
//		if(result != 4*sizeof(uint16))
//		{
//			if(mSocket != NULL)
//				mSocket->close();
//
//			mSocket = NULL;
//			mMotorsEnabled = false;
//		}
	}

//	void MotorInterface::sendCommandForced(const Collection<uint16> &cmds)
//	{
//		if(!isConnected())
//			return;
//
//		mMutex_sendTime.lock();
//if(mLastSendTime.getElapsedTimeMS() > 10)
//	Log::alert(String()+"Last send time 2: " + mLastSendTime.getElapsedTimeMS());
//		mLastSendTime.setTime();
//		mMutex_sendTime.unlock();
//
//		mMutex_socket.lock(); mMutex_data.lock();
//
//		for(int i=0; i<cmds.size(); i++)
//			mMotorCmds[i] = min((uint16)MAX_MOTOR_CMD, max((uint16)MIN_MOTOR_CMD, cmds[i]));
////			mMotorCmds[i] = cmds[i];
//		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));
//
//		mMutex_data.unlock(); mMutex_socket.unlock();
//
//		if(result != 4*sizeof(uint16))
//		{
//			if(mSocket != NULL)
//				mSocket->close();
//
//			mSocket = NULL;
//			mMotorsEnabled = false;
//		}
//	}

	void MotorInterface::enableMotors(bool on)
	{
		// make sure we are always at a good starting point
		if(on)
		{
			mMutex_cmds.lock();
			mMotorCmds.resize(4);
			mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 100;
			mMutex_cmds.unlock();
			mDoMotorWarmup = true;
			mMotorWarmupStartTime.setTime();
			Log::alert("MotorInterface -- Motors enabled");
		}
		else
		{
			mMutex_cmds.lock();
			mMotorCmds.resize(4);
			mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 0;
			mMutex_cmds.unlock();
			mDoMotorWarmup = false;
			mMotorsEnabled = false;
			Log::alert("MotorInterface -- Motors disabled");
		}
	}

	vector<uint16> MotorInterface::getMotorCmds()
	{
		mMutex_cmds.lock();
		vector<uint16> temp = mMotorCmds;
		mMutex_cmds.unlock();

		return temp;
	}

} // namespace Quadrotor
} // namespace ICSL
