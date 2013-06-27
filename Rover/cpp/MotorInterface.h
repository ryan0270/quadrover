#ifndef MOTORINTERFACE
#define MOTORINTERFACE
#include <sched.h>

#include "toadlet/egg.h"

#include "Time.h"

namespace ICSL {
namespace Quadrotor{

enum{ 
	MIN_MOTOR_CMD = 0, 
	MAX_MOTOR_CMD = 1<<11
};

class MotorInterface : public toadlet::egg::Thread
{
	public:
	MotorInterface();
	virtual ~MotorInterface();

	void shutdown();
	void initialize();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	//TODO: Need to make this function const
	bool isConnected();

	void sendCommand(toadlet::egg::Collection<uint16> const &cmds);

	void enableMotors(bool on);
	bool isMotorsEnabled() const {return mMotorsEnabled;}

	// from toadlet::egg::Thread
	void run();

	protected:
	toadlet::egg::Socket::ptr mServerSocket, mSocket;
	bool mRunning, mShutdown;
	bool mMotorsEnabled;
	bool mWaitingForConnection;
	uint16 mMotorCmds[4];

	toadlet::egg::Mutex mMutex_data, mMutex_socket;

	// skips enabled/disabled checks
	void sendCommandForced(toadlet::egg::Collection<uint16> const &cmds);

	int mThreadPriority, mScheduler;

	bool mDoMotorWarmup;
	Time mMotorWarmupStartTime;
};

} // namespace Quadrotor
} // namespace ICSL
#endif // MOTORINTERFACE
