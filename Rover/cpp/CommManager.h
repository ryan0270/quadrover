#ifndef ICSL_COMM_MANAGER
#define ICSL_COMM_MANAGER
#include <sched.h>
#include <thread>

#include <vector>
#include <toadlet/egg.h>

#include "Common.h"
#include "Time.h"

namespace ICSL{
namespace Quadrotor{
class CommManagerListener
{
	public:
	explicit CommManagerListener(){};
	virtual ~CommManagerListener(){};

	virtual void onNewCommMotorOn(){};
	virtual void onNewCommMotorOff(){};
//	virtual void onNewCommGainPID(float const rollPID[3], float const pitchPID[3], float const yawPID[3]){};
	virtual void onNewCommMotorTrim(int const trim[4]){};
	virtual void onNewCommObserverReset(){};
	virtual void onNewCommAttObserverGain(double gainP, double gainI, double accelWeight, double magWeight){};
	virtual void onNewCommTimeSync(int time){};
	virtual void onNewCommLogTransfer(){};
//	virtual void onNewCommControlSystemGains(Collection<float> const &gains){};
	virtual void onNewCommSendControlSystem(Collection<tbyte> const &buff){};
	virtual void onNewCommLogMask(uint32 mask){};
	virtual void onNewCommLogClear(){};
	virtual void onNewCommStateVicon(toadlet::egg::Collection<float> const &data){};
	virtual void onNewCommDesState(toadlet::egg::Collection<float> const &data){};
	virtual void onCommConnectionLost(){};
	virtual void onNewCommForceGain(float k){};
	virtual void onNewCommTorqueGain(float k){};
	virtual void onNewCommMass(float m){};
//	virtual void onNewCommIbvsGains(toadlet::egg::Collection<float> const &gains){};
	virtual void onNewCommUseIbvs(bool useIbvs){};
	virtual void onNewCommAttBias(float rollBias, float pitchBias, float yawBias){};
	virtual void onNewCommAttBiasAdaptRate(toadlet::egg::Collection<float> const &rate){};
	virtual void onNewCommForceGainAdaptRate(float rate){};
	virtual void onNewCommAttitudeGains(toadlet::egg::Collection<float> const &gains){};
	virtual void onNewCommTransGains(toadlet::egg::Collection<float> const &gains){};
	virtual void onNewCommKalmanMeasVar(toadlet::egg::Collection<float> const &std){};
	virtual void onNewCommKalmanDynVar(toadlet::egg::Collection<float> const &std){};
	virtual void onNewCommNominalMag(toadlet::egg::Collection<float> const &nomMag){};
	virtual void onNewCommMotorArmLength(float l){};
	virtual void onNewCommImgBufferSize(int size){};
	virtual void onNewCommBarometerZeroHeight(float h){};
	virtual void onNewCommVisionFeatureFindQualityLevel(float const &qLevel){};
	virtual void onNewCommVisionFeatureFindSeparationDistance(int const &sepDist){};
	virtual void onNewCommVisionFeatureFindFASTThreshold(int const &thresh){};
	virtual void onNewCommVisionFeatureFindPointCntTarget(int const &target){};
	virtual void onNewCommVisionFeatureFindFASTAdaptRate(float const &r){};
	virtual void onNewCommVelEstMeasCov(float const &measCov){};
	virtual void onNewCommVelEstProbNoCorr(float const &probNoCorr){};
	virtual void onNewCommSetDesiredPos(){};
}; // class CommManagerListener

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
	void transmitImageBuffer(uint32 numRows, uint32 numCols, uint32 numChannels, uint32 type, vector<unsigned char> const &buff);
	bool sendLogFile(const char* filename);
	bool pcIsConnected(){return mConnected;}

	protected:
	bool mConnected;
	Mutex mMutex_socketTCP, mMutex_socketUDP;
	Socket::ptr mServerSocketTCP, mSocketTCP, mSocketUDP;
	Time mLastPacketTime, mLastCmdRcvTime;
	uint32 mAddrPC;
	int mPortPC;
	bool mRun, mDone;

	Collection<CommManagerListener*> mListeners;

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
