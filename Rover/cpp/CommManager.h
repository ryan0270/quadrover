#ifndef ICSL_COMM_MANAGER
#define ICSL_COMM_MANAGER
#include <sched.h>
#include <thread>
#include <mutex>

#include <vector>
#include <toadlet/egg.h>

#include "Common.h"
#include "Time.h"
#include "Listeners.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace toadlet;

class CommManager 
{
	public:
	explicit CommManager();
	virtual ~CommManager();

	void initialize();
	void shutdown();
	void start(){ thread th(&CommManager::run, this); th.detach(); }
	void run();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void addListener(CommManagerListener* listener){mListeners.push_back(listener);}
	void transmitUDP(Packet &pck);
	void transmitImageBuffer(uint32 numRows, uint32 numCols, uint32 numChannels, uint32 type, const vector<unsigned char> &buff);
	bool sendLogFile(const char* filename);
	bool pcIsConnected(){return mConnected;}

	protected:
	bool mConnected;
	std::mutex mMutex_socketTCP, mMutex_socketUDP;
	toadlet::egg::Socket::ptr mServerSocketTCP, mSocketTCP, mSocketUDP;
	Time mLastPacketTime, mLastCmdRcvTime;
	uint32 mAddrPC;
	int mPortPC;
	bool mRun, mDone;

	toadlet::egg::Collection<CommManagerListener*> mListeners;

	void pollUDP();
	void pollTCP();
	int receiveUDP(tbyte* data, int size);
	int receiveTCP(tbyte* data, int size);
	bool receivePacket(Packet &pck, int size);

	int mThreadPriority, mScheduler;
}; // class CommManager
} // namespace Quadrotor
} // namespace ICSL

#endif
