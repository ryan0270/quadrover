#ifndef ICSL_COMM_MANAGER
#define ICSL_COMM_MANAGER
#include <sched.h>
#include <thread>
#include <mutex>

#include <vector>
#include <toadlet/egg.h>

#include "Common.h"
#include "Time.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace toadlet;
using namespace toadlet;
class CommManagerListener
{
	public:
	explicit CommManagerListener(){};
	virtual ~CommManagerListener(){};

	virtual void onNewCommMotorOn(){};
	virtual void onNewCommMotorOff(){};
//	virtual void const onNewCommGainPID(float const rollPID[3], float const pitchPID[3], float yawPID[3]){};
	virtual void const onNewCommMotorTrim(int trim[4]){};
	virtual void onNewCommObserverReset(){};
	virtual void onNewCommAttObserverGain(double gainP, double gainI, double accelWeight, double magWeight){};
	virtual void onNewCommTimeSync(int time){};
	virtual void onNewCommLogTransfer(){};
//	virtual void onNewCommControlSystemGains(const Collection<float> &gains){};
	virtual void onNewCommSendControlSystem(const toadlet::egg::Collection<tbyte> &buff){};
	virtual void onNewCommLogMask(uint32 mask){};
	virtual void onNewCommLogClear(){};
	virtual void onNewCommStateVicon(const toadlet::egg::Collection<float> &data){};
	virtual void onNewCommDesState(const toadlet::egg::Collection<float> &data){};
	virtual void onCommConnectionLost(){};
	virtual void onNewCommForceGain(float k){};
	virtual void onNewCommTorqueGain(float k){};
	virtual void onNewCommMass(float m){};
//	virtual void onNewCommIbvsGains(const toadlet::egg::Collection<float> &gains){};
	virtual void onNewCommUseIbvs(bool useIbvs){};
	virtual void onNewCommAccelBias(float xBias, float yBias, float zBias){};
	virtual void onNewCommForceGainAdaptRate(float rate){};
	virtual void onNewCommAttitudeGains(const toadlet::egg::Collection<float> &gains){};
	virtual void onNewCommTransGains(const toadlet::egg::Collection<float> &gains){};
	virtual void onNewCommKalmanMeasVar(const toadlet::egg::Collection<float> &std){};
	virtual void onNewCommKalmanDynVar(const toadlet::egg::Collection<float> &std){};
	virtual void onNewCommNominalMag(const toadlet::egg::Collection<float> &nomMag){};
	virtual void onNewCommMotorArmLength(float l){};
	virtual void onNewCommImgBufferSize(int size){};
	virtual void onNewCommBarometerZeroHeight(float h){};
	virtual void onNewCommVisionFeatureFindQualityLevel(float qLevel){};
	virtual void onNewCommVisionFeatureFindSeparationDistance(int sepDist){};
	virtual void onNewCommVisionFeatureFindFASTThreshold(int thresh){};
	virtual void onNewCommVisionFeatureFindPointCntTarget(int target){};
	virtual void onNewCommVisionFeatureFindFASTAdaptRate(float r){};
	virtual void onNewCommVelEstMeasCov(float measCov){};
	virtual void onNewCommVelEstProbNoCorr(float probNoCorr){};
	virtual void onNewCommSetDesiredPos(){};
	virtual void onNewCommViconCameraOffset(float x, float y, float z){};
	virtual void onNewCommTargetNominalLength(float length){};
	virtual void onNewCommMAPHeightMeasCov(float cov){};
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
