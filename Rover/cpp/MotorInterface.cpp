#include "MotorInterface.h"

namespace ICSL {
namespace Quadrotor{
using namespace toadlet;
using namespace toadlet::egg;
	MotorInterface::MotorInterface()
	{
		mRunning = false;
		mShutdown = true;

		mMotorsEnabled = false;
		mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 0;

		mWaitingForConnection = true;

		mDoMotorWarmup = false;

		mServerSocketSend = NULL;
		mServerSocketReceive= NULL;
		mSocketSend = NULL;
		mSocketReceive = NULL;

		mLastSendTime.setTimeMS(0);
	}

	MotorInterface::~MotorInterface()
	{
	}

	void MotorInterface::shutdown()
	{
		Log::alert("------------------------ MotorInterface shutdown started");
		mRunning = false;
		while(!mShutdown)
			System::msleep(10);

		Log::alert("------------------------ MotorInterface shutdown done -------------------------");
	}

	void MotorInterface::initialize()
	{
		mServerSocketSend = Socket::ptr(Socket::createTCPSocket());
		mServerSocketSend->bind(45670);
		mServerSocketSend->listen(1);
		mServerSocketSend->setBlocking(false);
		mSocketSend = Socket::ptr(mServerSocketSend->accept());

		mServerSocketReceive = Socket::ptr(Socket::createTCPSocket());
		mServerSocketReceive->bind(45671);
		mServerSocketReceive->listen(1);
		mServerSocketReceive->setBlocking(false);
		mSocketReceive = Socket::ptr(mServerSocketReceive->accept());

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
		Time lastSendTime;
		while(mRunning)
		{
			mMutex_sendTime.lock();
			lastSendTime.setTime(mLastSendTime);
			mMutex_sendTime.unlock();
			if(!isConnectedSend())
			{
				mWaitingForConnection = true;
				mWaitingForConnection = false;
				mSocketSend = Socket::ptr(mServerSocketSend->accept());
				if(mSocketSend != NULL)
					Log::alert("Connected to phone-->arduino port");
			}
			else if(!mMotorsEnabled && !mDoMotorWarmup && lastSendTime.getElapsedTimeMS() > 20)
			{
				// this is just to keep the connection alive
				Collection<uint16_t> cmds(4,0);
				sendCommandForced(cmds);
			}
			else if(mDoMotorWarmup)
			{
				Collection<uint16_t> cmds(4,100);
				sendCommandForced(cmds);
				if(mMotorWarmupStartTime.getElapsedTimeMS() > 4e3)
				{
					mDoMotorWarmup = false;
					mMotorsEnabled = true;

					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onMotorWarmupDone();
					Log::alert("Warmup done");
				}
			}

			if(isConnectedReceive())
				pollTCP();
			else
			{
				mSocketReceive = Socket::ptr(mServerSocketReceive->accept());
				if(mSocketReceive != NULL)
				{
					Log::alert("Connected to arduino-->phone port");
				}
			}

			System::msleep(1);
		}

		Collection<uint16_t> cmds(4,0);
		sendCommandForced(cmds);
		mMutex_socket.lock();
		if(mSocketSend != NULL)
		{
			mSocketSend->close();
			mSocketSend = NULL;
		}
		if(mServerSocketSend != NULL)
			mServerSocketSend->close();
		mServerSocketSend = NULL;

		if(mSocketReceive != NULL)
		{
			mSocketReceive->close();
			mSocketReceive = NULL;
		}
		if(mServerSocketReceive != NULL)
			mServerSocketReceive->close();
		mServerSocketReceive = NULL;
		mMutex_socket.unlock();

		mShutdown = true;
	}

	void MotorInterface::sendCommand(const Collection<uint16_t> &cmds)
	{
		if(!isConnectedSend() || !mMotorsEnabled)
			return;

		mMutex_sendTime.lock();
		mLastSendTime.setTime();
		mMutex_sendTime.unlock();

		mMutex_socket.lock(); mMutex_data.lock();

		for(int i=0; i<cmds.size(); i++)
			mMotorCmds[i] = min((uint16_t)MAX_MOTOR_CMD, max((uint16_t)MIN_MOTOR_CMD, cmds[i]));
		int result = mSocketSend->send((tbyte*)mMotorCmds, 4*sizeof(uint16_t));
		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint16_t))
		{
			if(mSocketSend != NULL)
				mSocketSend->close();

			mSocketSend = NULL;
			mMotorsEnabled = false;
		}
	}

	void MotorInterface::sendCommandForced(const Collection<uint16_t> &cmds)
	{
		if(!isConnectedSend())
			return;

		mMutex_sendTime.lock();
		mLastSendTime.setTime();
		mMutex_sendTime.unlock();

		mMutex_socket.lock(); mMutex_data.lock();

		for(int i=0; i<cmds.size(); i++)
			mMotorCmds[i] = min((uint16_t)MAX_MOTOR_CMD, max((uint16_t)MIN_MOTOR_CMD, cmds[i]));
		int result = mSocketSend->send((tbyte*)mMotorCmds, 4*sizeof(uint16_t));

		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint16_t))
		{
			if(mSocketSend != NULL)
				mSocketSend->close();

			mSocketSend = NULL;
			mMotorsEnabled = false;
		}
	}

	void MotorInterface::enableMotors(bool on)
	{
		// make sure we are always at a good starting point
		Collection<uint16_t> cmds(4,0);
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

	bool MotorInterface::isConnectedSend() const
	{
		return (mSocketSend != NULL) && !mWaitingForConnection;
	}

	bool MotorInterface::isConnectedReceive() const
	{
		return mSocketReceive != NULL;
	}

	void MotorInterface::pollTCP()
	{
		if(!isConnectedReceive())
		{
			Log::alert("Not connected");
			return;
		}

		Time curTime;
		bool newPacketReady = mSocketReceive != NULL && mSocketReceive->pollRead(0);
		while(newPacketReady && mRunning)
		{
			uint8_t code = -1;
			bool resetSocket = false;
			int received = receiveTCP((tbyte*)&code, sizeof(code));// == sizeof(code);
			if(received == sizeof(code))
			{
//				{
//					String str = String() + "code: " + (int)code;
//					Log::alert(str);
//				}
				switch(code)
				{
					case COMM_ARDUINO_HEIGHT:
						uint16_t height;
						received = receiveTCP((tbyte*)&height, sizeof(height));
						if(received == sizeof(height))
						{
							shared_ptr<HeightData<double>> heightData(new HeightData<double>);
							heightData->timestamp.setTime(curTime);
							heightData->type = DATA_TYPE_HEIGHT;
							heightData->heightRaw = height/1000.0;
							heightData->height = height/1000.0;
							
							for(int i=0; i<mSonarListeners.size(); i++)
								mSonarListeners[i]->onNewSonar(heightData);
						}
						else
							resetSocket = true;
						break;
					default:
						Log::alert(String()+"Unknown arduino code: " + code);
						resetSocket = true;
				}
			}

			if(resetSocket)
			{
				Log::alert("Socket reset");
				mSocketReceive->close();
				mSocketReceive = NULL;
			}

			newPacketReady = mSocketReceive != NULL && mSocketReceive->pollRead(0);
		}
	}

	int MotorInterface::receiveTCP(tbyte* data, int size)
	{
		if(!isConnectedReceive())
			return 0;
		int received= mSocketReceive->receive(data, size);
		int delta = 1;
		while(delta > 0 && received < size)
		{
			delta = mSocketReceive->receive(data+received, size-received);
			received += delta;
		}

		return received;
	}
} // namespace Quadrotor
} // namespace ICSL
