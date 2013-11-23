#ifndef MOTORINTERFACE
#define MOTORINTERFACE
#include <sched.h>
#include <thread>
#include <mutex>
#include <memory>

#include "toadlet/egg.h"

#include "Time.h"
#include "Data.h"
#include "Listeners.h"

namespace ICSL {
namespace Quadrotor{
using namespace std;

enum{ 
	MIN_MOTOR_CMD = 0, 
	MAX_MOTOR_CMD = 1<<11
};

enum
{
	COMM_ARDUINO_HEIGHT=1,
};

class MotorInterface
{
	public:
	MotorInterface();
	virtual ~MotorInterface();

	void shutdown();
	void initialize();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	inline bool isConnectedSend() const;
	inline bool isConnectedReceive() const;

	void sendCommand(const toadlet::egg::Collection<uint16_t> &cmds);

	void enableMotors(bool on);
	bool isMotorsEnabled() const {return mMotorsEnabled;}

	void start(){ thread th(&MotorInterface::run, this); th.detach(); }
	void run();

	void addListener(MotorInterfaceListener *listener){mListeners.push_back(listener);}
	void addSonarListener(SonarListener *listener){mSonarListeners.push_back(listener);}

	void setStartTime(Time time){mStartTime.setTime(time);}

	protected:
	toadlet::egg::Socket::ptr mServerSocketSend, mSocketSend;
	toadlet::egg::Socket::ptr mServerSocketReceive, mSocketReceive;
	bool mRunning, mShutdown;
	bool mMotorsEnabled;
	bool mWaitingForConnection;
	uint16_t mMotorCmds[4];

	std::mutex mMutex_data, mMutex_socket;

	// skips enabled/disabled checks
	void sendCommandForced(const toadlet::egg::Collection<uint16_t> &cmds);

	int mThreadPriority, mScheduler;

	bool mDoMotorWarmup;
	Time mMotorWarmupStartTime;
	Time mStartTime;
	Time mLastSendTime;
	std::mutex mMutex_sendTime;

	toadlet::egg::Collection<MotorInterfaceListener*> mListeners;
	toadlet::egg::Collection<SonarListener*> mSonarListeners;

	void pollTCP();
	int receiveTCP(toadlet::tbyte* data, int size);
};

} // namespace Quadrotor
} // namespace ICSL
#endif // MOTORINTERFACE
