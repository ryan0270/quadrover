#include "MotorInterface.h"

namespace ICSL {
namespace Quadrotor{
using namespace toadlet::egg;
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

					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onMotorWarmupDone();
					Log::alert("Warmup done");
				}
			}

			if(isConnected())
				pollTCP();

			System::msleep(1);
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

	void MotorInterface::sendCommand(const Collection<uint16> &cmds)
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

	void MotorInterface::sendCommandForced(const Collection<uint16> &cmds)
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

	bool MotorInterface::isConnected() const
	{
		return (mSocket != NULL) && !mWaitingForConnection;
	}

	void MotorInterface::pollTCP()
	{
		if(!isConnected())
		{
			Log::alert("Not connected");
			return;
		}

		Time curTime;
		bool newPacketReady = mSocket != NULL && mSocket->pollRead(0);
		while(newPacketReady && mRunning)
		{
			uint8_t code = -1;
			bool resetSocket = false;
//			bool received = receiveTCP((tbyte*)&code, sizeof(code));// == sizeof(code);
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
				mSocket->close();
				mSocket = NULL;
			}

			newPacketReady = mSocket != NULL && mSocket->pollRead(0);
		}
	}

	int MotorInterface::receiveTCP(tbyte* data, int size)
	{
		if(!isConnected())
			return 0;
		int received= mSocket->receive(data, size);
		int delta = 1;
		while(delta > 0 && received < size)
		{
			delta = mSocket->receive(data+received, size-received);
			received += delta;
		}

		return received;
	}
} // namespace Quadrotor
} // namespace ICSL
