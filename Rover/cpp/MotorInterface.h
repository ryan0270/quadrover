#ifndef MOTORINTERFACE
#define MOTORINTERFACE
#include <sched.h>
#include <thread>
#include <mutex>

#include "toadlet/egg.h"

#include "Time.h"

namespace ICSL {
namespace Quadrotor{
using namespace std;
using namespace toadlet;

enum{ 
	MIN_MOTOR_CMD = 0, 
	MAX_MOTOR_CMD = 1<<11
};

class MotorInterfaceListener
{
	public:
	virtual void onMotorWarmupDone()=0;
};

class MotorInterface
{
	public:
	MotorInterface();
	virtual ~MotorInterface();

	void shutdown();
	void initialize();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	//TODO: Need to make this function const
	bool isConnected() const;

	void sendCommand(toadlet::egg::Collection<uint16> const &cmds);

	void enableMotors(bool on);
	bool isMotorsEnabled() const {return mMotorsEnabled;}

	void start(){ thread th(&MotorInterface::run, this); th.detach(); }
	void run();

	void addListener(MotorInterfaceListener *listener){mListeners.push_back(listener);}

	protected:
	toadlet::egg::Socket::ptr mServerSocket, mSocket;
	bool mRunning, mShutdown;
	bool mMotorsEnabled;
	bool mWaitingForConnection;
	uint16 mMotorCmds[4];

	std::mutex mMutex_data, mMutex_socket;

	// skips enabled/disabled checks
	void sendCommandForced(toadlet::egg::Collection<uint16> const &cmds);

	int mThreadPriority, mScheduler;

	bool mDoMotorWarmup;
	Time mMotorWarmupStartTime;

	toadlet::egg::Collection<MotorInterfaceListener*> mListeners;
};

} // namespace Quadrotor
} // namespace ICSL
#endif // MOTORINTERFACE
