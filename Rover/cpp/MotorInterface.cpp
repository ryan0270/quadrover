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
		mServerSocket->bind(45670);
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
				Collection<uint8> cmds(4,0);
				sendCommandForced(cmds);
			}

			sys.msleep(50);
		}

		Collection<uint8> cmds(4,0);
		sendCommandForced(cmds);
		mMutex_socket.lock();
		if(mSocket != NULL)
		{
			mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint8));
			mSocket->close();
			mSocket = NULL;
		}
		mServerSocket->close();
		mServerSocket = NULL;
		mMutex_socket.unlock();

		mShutdown = true;
	}

	void MotorInterface::sendCommand(Collection<uint8> const &cmds)
	{
		if(!isConnected() || !mMotorsEnabled)
			return;

		mMutex_socket.lock(); mMutex_data.lock();

		for(int i=0; i<cmds.size(); i++)
			mMotorCmds[i] = cmds[i];
		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint8));
		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint8))
		{
			if(mSocket != NULL)
				mSocket->close();

			mSocket = NULL;
			mMotorsEnabled = false;
		}
	}

	void MotorInterface::sendCommandForced(Collection<uint8> const &cmds)
	{
		if(!isConnected())
			return;

		mMutex_socket.lock(); mMutex_data.lock();

		for(int i=0; i<cmds.size(); i++)
			mMotorCmds[i] = cmds[i];
		int result = mSocket->send((tbyte*)mMotorCmds, 4*sizeof(uint8));

		mMutex_data.unlock(); mMutex_socket.unlock();

		if(result != 4*sizeof(uint8))
		{
			if(mSocket != NULL)
				mSocket->close();

			mSocket = NULL;
			mMotorsEnabled = false;
		}
	}

	void MotorInterface::enableMotors(bool on)
	{
		mMotorsEnabled = on;

		// make sure we are always at a good starting point
		Collection<uint8> cmds(4,0);
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
		bool temp = (mSocket != NULL); 
		mMutex_socket.unlock(); 
		return temp;
	}

} // namespace Quadrotor
} // namespace ICSL
