#include "MotorInterface.h"

using namespace toadlet::egg;

namespace ICSL {
namespace Quadrotor{
	MotorInterface::MotorInterface()
	{
		mRunning = false;
		mShutdown = true;

		mMotorsEnabled = false;
		mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 1000;

		mWaitingForConnection = true;
	}

	MotorInterface::~MotorInterface()
	{
	}

	void MotorInterface::shutdown()
	{
		Log::alert("------------------------ MotorInterface shutdown started");
		mRunning = false;
		System sys;
		while(!mShutdown)
			sys.msleep(10);

		Log::alert("------------------------ MotorInterface shutdown done -------------------------");
	}

	void MotorInterface::initialize()
	{
		mServerSocket = Socket::ptr(Socket::createTCPSocket());
		mServerSocket->bind(4567);
		mServerSocket->listen(1);
		mServerSocket->setBlocking(false);

		mSocket = Socket::ptr(mServerSocket->accept());
	}

	void MotorInterface::run()
	{
		mShutdown = false;
		mRunning = true;

		// this just monitors the connection
		System sys;
		while(mRunning)
		{
			if(!isConnected())
			{
				mWaitingForConnection = true;
//				mServerSocket->setBlocking(true);
				mWaitingForConnection = false;
				mSocket = Socket::ptr(mServerSocket->accept());
				if(mSocket != NULL)
					Log::alert("Connected to motors");
			}
			else if(!mMotorsEnabled)
			{
				// this is just to keep the connection alive
				Collection<uint16> cmds(4,1000);
				cmds[0] = 1000;
				cmds[1] = 1001;
				cmds[2] = 1002;
				cmds[3] = 1003;
				sendCommandForced(cmds);
			}

			sys.msleep(50);
		}

		Collection<uint16> cmds(4,1000);
		sendCommandForced(cmds);
//		mMutex_data.lock();
//		mMotorCmds[0] = mMotorCmds[1] = mMotorCmds[2] = mMotorCmds[3] = 1000;
//		mMutex_data.unlock();
		mMutex_socket.lock();
		if(mSocket != NULL && mSocket->connected())
		{
			mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));
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
			mMotorCmds[i] = cmds[i];
		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));

		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint16))
		{
			if(mSocket != NULL && mSocket->connected())
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
			mMotorCmds[i] = cmds[i];
		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint16));

		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint16))
		{
			if(mSocket != NULL && mSocket->connected())
				mSocket->close();

			mSocket = NULL;
			mMotorsEnabled = false;
		}
	}

	void MotorInterface::enableMotors(bool on)
	{
		mMotorsEnabled = on;

		// make sure we are always at a good starting point
		Collection<uint16> cmds(4,1000);
		sendCommandForced(cmds);

		if(mMotorsEnabled)
			Log::alert("MotorInterface -- Motors enabled");
		else
			Log::alert("MotorInterface -- Motors disabled");
	}

	// TODO: Need to make this function const
	bool MotorInterface::isConnected()
	{
		if(mWaitingForConnection)
			return false;

		mMutex_socket.lock(); 
		bool temp = (mSocket != NULL && mSocket->connected()); 
		mMutex_socket.unlock(); 
		return temp;
	}

} // namespace Quadrotor
} // namespace ICSL
