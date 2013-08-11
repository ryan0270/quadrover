#include "MotorInterface.h"

using namespace toadlet::egg;

namespace ICSL {
namespace Quadrotor{
	MotorInterface::MotorInterface()
	{
		mRunning = false;
		mShutdown = true;

		mMotorsEnabled = false;
		mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 0;

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
		mServerSocket = Socket::ptr(Socket::createTCPSocket());
		mServerSocket->bind(45670);
		mServerSocket->listen(1);
		mServerSocket->setBlocking(false);

		mSocket = Socket::ptr(mServerSocket->accept());

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mMutex_socket.unlock();
	}

	void MotorInterface::run()
	{
		mShutdown = false;
		mRunning = true;

		// this just monitors the connection
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			if(!isConnected())
			{
				mWaitingForConnection = true;
				mWaitingForConnection = false;
				mSocket = Socket::ptr(mServerSocket->accept());
				if(mSocket != NULL)
					Log::alert("Connected to motors");
			}
			else if(!mMotorsEnabled && !mDoMotorWarmup)
			{
				// this is just to keep the connection alive
				Collection<uint16> cmds(4,0);
				sendCommandForced(cmds);
			}
			else if(mDoMotorWarmup)
			{
				Collection<uint16> cmds(4,100);
				sendCommandForced(cmds);
				if(mMotorWarmupStartTime.getElapsedTimeMS() > 4e3)
				{
					mDoMotorWarmup = false;
					mMotorsEnabled = true;
					Log::alert("Warmup done");
				}
			}

			System::msleep(20);
		}

		Collection<uint16> cmds(4,0);
		sendCommandForced(cmds);
		mMutex_socket.lock();
		if(mSocket != NULL)
		{
//			mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));
			mSocket->close();
			mSocket = NULL;
		}
		mServerSocket->close();
		mServerSocket = NULL;
		mMutex_socket.unlock();

		mShutdown = true;
	}

	void MotorInterface::sendCommand(Collection<uint16> const &cmds)
	{
		if(!isConnected() || !mMotorsEnabled)
			return;

		mMutex_socket.lock(); mMutex_data.lock();

		for(int i=0; i<cmds.size(); i++)
			mMotorCmds[i] = min((uint16)MAX_MOTOR_CMD, max((uint16)MIN_MOTOR_CMD, cmds[i]));
		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));
		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint16))
		{
			if(mSocket != NULL)
				mSocket->close();

			mSocket = NULL;
			mMotorsEnabled = false;
		}
	}

	void MotorInterface::sendCommandForced(Collection<uint16> const &cmds)
	{
		if(!isConnected())
			return;

		mMutex_socket.lock(); mMutex_data.lock();

		for(int i=0; i<cmds.size(); i++)
			mMotorCmds[i] = min((uint16)MAX_MOTOR_CMD, max((uint16)MIN_MOTOR_CMD, cmds[i]));
//			mMotorCmds[i] = cmds[i];
		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));

		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint16))
		{
			if(mSocket != NULL)
				mSocket->close();

			mSocket = NULL;
			mMotorsEnabled = false;
		}
	}

	void MotorInterface::enableMotors(bool on)
	{
		// make sure we are always at a good starting point
		Collection<uint16> cmds(4,0);
		sendCommandForced(cmds);

		if(on)
		{
			mDoMotorWarmup = true;
			mMotorWarmupStartTime.setTime();
			Log::alert("MotorInterface -- Motors enabled");
		}
		else
		{
			mDoMotorWarmup = false;
			mMotorsEnabled = false;
			Log::alert("MotorInterface -- Motors disabled");
		}
	}

	// TODO: Need to make this function const
	bool MotorInterface::isConnected() const
	{
		return (mSocket != NULL) && !mWaitingForConnection;
	}
} // namespace Quadrotor
} // namespace ICSL
