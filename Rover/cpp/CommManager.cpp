#include "CommManager.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace toadlet::egg;
CommManager::CommManager()
{
	mLastCmdRcvTime.clear();
	mLastPacketTime.clear();

	mAddrPC = 0;
	mPortPC = 0;

	mDone = true;

	mConnected = false;

	mSocketUDP = NULL;
	mSocketTCP = NULL;
	mServerSocketTCP = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
}

CommManager::~CommManager()
{	
	mMutex_socketUDP.unlock();
	mMutex_socketTCP.unlock();
}

void CommManager::initialize()
{
	mServerSocketTCP = Socket::ptr(Socket::createTCPSocket());
	mServerSocketTCP->bind(13120);
	mServerSocketTCP->listen(1);
	mServerSocketTCP->setBlocking(false);
	mSocketTCP = Socket::ptr(mServerSocketTCP->accept());
	if(mSocketTCP != NULL)
	{
		Log::alert(String("Connected to PC"));
		mLastCmdRcvTime.setTime();
	}
	else
		Log::alert(String("Not connected to PC"));
	mSocketUDP = Socket::ptr(Socket::createUDPSocket());
	mSocketUDP->bind(13120);
//	mSocketUDP->listen(1);
	mSocketUDP->setBlocking(false);
}

void CommManager::shutdown()
{
	Log::alert("------------------ Comm Manager shutdown started --------------------");
	mRun = false;
	while(!mDone)
		System::msleep(10);

	if(mSocketTCP != NULL)
	{
		int code = COMM_HOST_EXIT;
		mSocketTCP->send((tbyte*)&code, sizeof(code));
	}

	if(mSocketUDP != NULL) mSocketUDP->close();
	if(mSocketTCP != NULL) mSocketTCP->close();
	if(mServerSocketTCP != NULL) mServerSocketTCP->close();

	mSocketUDP = NULL;
	mSocketTCP = NULL;
	mServerSocketTCP = NULL;
	
	Log::alert("------------------ Comm Manager done --------------------");
}

void CommManager::run()
{
	mDone = false;
	mRun = true;
	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);
	while(mRun)
	{
		mMutex_socketTCP.lock();
		mConnected = mSocketTCP != NULL;
		mMutex_socketTCP.unlock();

		if(mConnected && mLastCmdRcvTime.getElapsedTimeMS() > 500)
		{
			Log::alert("Lost connection");
			mConnected = false;
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onCommConnectionLost();
			mMutex_socketTCP.lock();
			mSocketTCP->close();
			mSocketTCP = NULL;
			mMutex_socketTCP.unlock();
			mConnected = false;
		}

		if(mConnected)
		{
			mConnected = true;
			pollUDP();
			pollTCP();
		}
		else
		{
			mConnected = false;
			mMutex_socketTCP.lock();
			// check again to see if we have a new connection
			// mServerSocketTCP->setBlocking(true);
//			Socket *sock = mServerSocketTCP->accept();
			mSocketTCP = Socket::ptr(mServerSocketTCP->accept());
			if(mSocketTCP != NULL)
			{
				Log::alert(String("Connected to PC"));
				mAddrPC = mSocketTCP->getHostIPAddress();
//				mPortPC = mSocketTCP->getHostPort();
mPortPC = 13120;

				if(mSocketUDP != NULL)
				{
					mSocketUDP->close();
					mSocketUDP = NULL;
				}
				mSocketUDP = Socket::ptr(Socket::createUDPSocket());
				mSocketUDP->bind(mPortPC);
//				mSocketUDP->listen(1);
				mSocketUDP->setBlocking(false);
			}
			mMutex_socketTCP.unlock();
			mLastCmdRcvTime.setTime();
		}

		System::msleep(1);
	}

	mDone = true;
	Log::alert("------------------ Comm manager runner is dead -------------------");
}

void CommManager::transmitUDP(Packet &pck)
{
	Collection<tbyte> buff;
	pck.serialize(buff);
	if(mAddrPC != 0)
	{
		mMutex_socketUDP.lock();
		if(mSocketUDP != NULL) mSocketUDP->sendTo(buff.begin(), buff.size(), mAddrPC, mPortPC);
		mMutex_socketUDP.unlock();
	}
}

void CommManager::transmitImageBuffer(uint32 numRows, uint32 numCols, uint32 numChannels, uint32 type, const vector<unsigned char> &buff)
{
	if(!mConnected)
		return;

	uint32 code = COMM_IMG_DATA;
	uint32 size = buff.size()*sizeof(unsigned char);
	mMutex_socketTCP.lock();
	if(mSocketTCP != NULL) mSocketTCP->send((tbyte*)&code, sizeof(code));
	if(mSocketTCP != NULL) mSocketTCP->send((tbyte*)&numRows, sizeof(numRows));
	if(mSocketTCP != NULL) mSocketTCP->send((tbyte*)&numCols, sizeof(numCols));
	if(mSocketTCP != NULL) mSocketTCP->send((tbyte*)&numChannels, sizeof(numChannels));
	if(mSocketTCP != NULL) mSocketTCP->send((tbyte*)&type, sizeof(type));
	if(mSocketTCP != NULL) mSocketTCP->send((tbyte*)&size, sizeof(size));
	if(mSocketTCP != NULL) mSocketTCP->send((tbyte*)&buff.front(), size);
	mMutex_socketTCP.unlock();
}

void CommManager::pollUDP()
{
	mMutex_socketUDP.lock();
	while(mSocketUDP != NULL && mSocketUDP->pollRead(0))
	{
		int size = 256;
		Packet pck;
		if(receivePacket(pck, size))
		{
//			Log::alert(String()+"Received UDP packet of type "+pck.type);
			if(pck.time >= mLastPacketTime.getMS())
				mLastPacketTime.setTimeMS(pck.time);
			else if(abs((long int)(pck.time - mLastPacketTime.getMS())) > 1000)
			{
				String s = String()+"Really old packet so assume this is a new starting poing. dt = "+(pck.time-mLastPacketTime.getMS());
				Log::alert(s);
				mLastPacketTime.setTimeMS(pck.time);
			}
			else
			{
				Log::alert(String("Ignoring old packet"));
				continue;
			}

			if(pck.time < mLastPacketTime.getMS())
				Log::alert("Why am I here?");

			switch(pck.type)
			{
				case COMM_PING:
					mLastCmdRcvTime.setTime();
					break;
				case COMM_STATE_VICON:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommStateVicon(pck.dataFloat);
					mLastCmdRcvTime.setTime();
					break;
				case COMM_SET_DESIRED_STATE:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommDesState(pck.dataFloat);
					mLastCmdRcvTime.setTime();
					break;
				default:
					Log::alert(String()+"Unknown UDP comm: " + pck.type);
			}
		}
	}
	mMutex_socketUDP.unlock();
}

void CommManager::pollTCP()
{
	if(!mConnected)
		return;
	mMutex_socketTCP.lock();
	bool newPacketReady = mSocketTCP != NULL && mSocketTCP->pollRead(0);
	mMutex_socketTCP.unlock();
	while(newPacketReady)
	{
		int code = -1;
		bool resetSocket = false;
		bool received = receiveTCP((tbyte*)&code, sizeof(code));
		if(received)
		{
			{
				String str = String() + "code: " + code;
				Log::alert(str);
			}
			switch(code)
			{
				case COMM_MOTOR_ON:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommMotorOn();
					break;
				case COMM_MOTOR_OFF:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommMotorOff();
					break;
				case COMM_CNTL_ATT_GAINS:
					{
						Collection<float> gains;
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							gains.resize(size);
							bool received = receiveTCP((tbyte*)&(gains[0]),size*sizeof(float));
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommAttitudeGains(gains);
							}
							else
								resetSocket = true;
						}
					}
					break;
				case COMM_CNTL_TRANS_GAINS:
					{
						Collection<float> gains;
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							gains.resize(size);
							bool received = receiveTCP((tbyte*)&(gains[0]),size*sizeof(float));
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommTransGains(gains);
							}
							else
								resetSocket = true;
						}
					}
					break;
				case COMM_CNTL_IBVS_GAINS:
					{
						Collection<float> posGains, velGains;
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							posGains.resize(size);
							velGains.resize(size);
							bool received = receiveTCP((tbyte*)&(posGains[0]),size*sizeof(float));
							if(received) received = receiveTCP((tbyte*)&(velGains[0]),size*sizeof(float));
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommIbvsGains(posGains, velGains);
							}
							else
								resetSocket = true;
						}
					}
					break;
				case COMM_KALMANFILTER_MEAS_VAR:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							Collection<float> var(size);
							received = receiveTCP((tbyte*)&(var[0]), size*sizeof(float));
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommKalmanMeasVar(var);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_KALMANFILTER_DYN_VAR:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							Collection<float> var(size);
							received = receiveTCP((tbyte*)&(var[0]), size*sizeof(float));
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommKalmanDynVar(var);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_MOTOR_TRIM:
					{
						int trim[4];
						bool received = receiveTCP((tbyte*)trim, 4*sizeof(int));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommMotorTrim(trim);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_OBSV_RESET:
					{
						for(int i=0; i<mListeners.size(); i++)
							mListeners[i]->onNewCommObserverReset();
					}
					break;
				case COMM_ATT_OBSV_GAIN:
					{
						float gainP, gainI, accelWeight, magWeight;
						bool received 			= receiveTCP((tbyte*)&gainP, sizeof(gainP));
						if(received) received 	= receiveTCP((tbyte*)&gainI, sizeof(gainI));
						if(received) received 	= receiveTCP((tbyte*)&accelWeight, sizeof(accelWeight));
						if(received) received 	= receiveTCP((tbyte*)&magWeight, sizeof(magWeight));
						if(received )
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommAttObserverGain(gainP, gainI, accelWeight, magWeight);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_TIME_SYNC:
					{
						int time;
						bool received = receiveTCP((tbyte*)&time, sizeof(time));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommTimeSync(time);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_LOG_FILE_REQUEST:
					{
						for(int i=0; i<mListeners.size(); i++)
							mListeners[i]->onNewCommLogTransfer();
					}
					break;
				case COMM_LOG_MASK:
					{
						uint32 mask;
						bool received = receiveTCP((tbyte*)&mask, sizeof(mask));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommLogMask(mask);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_CLEAR_LOG:
					{
						for(int i=0; i<mListeners.size(); i++)
							mListeners[i]->onNewCommLogClear();
					}
					break;
				case COMM_MOTOR_FORCE_GAIN:
					{
						float k;
						bool received = receiveTCP((tbyte*)&k,sizeof(k));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommForceGain(k);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_MOTOR_TORQUE_GAIN:
					{
						float k;
						bool received = receiveTCP((tbyte*)&k,sizeof(k));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommTorqueGain(k);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_MASS:
					{
						float m;
						bool received = receiveTCP((tbyte*)&m,sizeof(m));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommMass(m);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_USE_IBVS:
					{
						uint16 useIbvs;
						bool received = receiveTCP((tbyte*)&useIbvs,sizeof(useIbvs));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommUseIbvs(useIbvs==1);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_ACCELEROMETER_BIAS:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							Collection<float> bias(size);
							received = receiveTCP((tbyte*)&(bias[0]), size*sizeof(float));
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommAccelBias(bias[0], bias[1], bias[2]);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_KALMAN_FORCE_SCALING_ADAPT_RATE:
					{
						float rate;
						bool received = receiveTCP((tbyte*)&rate, sizeof(rate));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommForceGainAdaptRate(rate);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_NOMINAL_MAG:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							Collection<float> nomMag(size);;
							bool received = receiveTCP((tbyte*)&(nomMag[0]),size*sizeof(float));
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommNominalMag(nomMag);
							}
							else
								resetSocket = true;
						}
					}
					break;
				case COMM_MOTOR_ARM_LENGTH:
					{
						float l;
						bool received = receiveTCP((tbyte*)&l, sizeof(l));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommMotorArmLength(l);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_CLIENT_EXIT:
					{
						Log::alert("Client exiting");
						resetSocket = true;
					}
					break;
				case COMM_IMG_BUFFER_SIZE:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size, sizeof(size));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgBufferSize(size);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_BAROMETER_ZERO_HEIGHT:
					{
						float h;
						bool received = receiveTCP((tbyte*)&h, sizeof(h));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommBarometerZeroHeight(h);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_FEATURE_FIND_QUALITY_THRESHOLD:
					{
						float qLevel;
						bool received = receiveTCP((tbyte*)&qLevel, sizeof(qLevel));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommVisionFeatureFindQualityLevel(qLevel);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_FEATURE_FIND_SEPARATION_DISTANCE:
					{
						int sepDist;
						bool received = receiveTCP((tbyte*)&sepDist, sizeof(sepDist));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommVisionFeatureFindSeparationDistance(sepDist);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_FEATURE_FIND_FAST_THRESHOLD:
					{
						int thresh;
						bool received = receiveTCP((tbyte*)&thresh, sizeof(thresh));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommVisionFeatureFindFASTThreshold(thresh);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_FEATURE_FIND_POINT_COUNT_TARGET:
					{
						int target;
						bool received = receiveTCP((tbyte*)&target, sizeof(target));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommVisionFeatureFindPointCntTarget(target);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_FEATURE_FIND_FAST_ADAPT_RATE:
					{
						float r;
						bool received = receiveTCP((tbyte*)&r, sizeof(r));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommVisionFeatureFindFASTAdaptRate(r);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_VELOCITY_ESTIMATION_VISION_MEASUREMENT_COV:
					{
						float measCov;
						bool received = receiveTCP((tbyte*)&measCov, sizeof(measCov));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommVelEstMeasCov(measCov);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_VELOCITY_ESTIMATION_PROB_NO_CORR:
					{
						float probNoCorr;
						bool received = receiveTCP((tbyte*)&probNoCorr, sizeof(probNoCorr));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommVelEstProbNoCorr(probNoCorr);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_SET_DESIRED_POS:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommSetDesiredPos();
					break;
				case COMM_SEND_TRANS_CNTL_SYSTEM:
					{
						uint32 size;
						Collection<tbyte> buff;
						bool received = receiveTCP((tbyte*)&size, sizeof(size));
						if(received)
						{
							buff.resize(size);
							bool receieved = receiveTCP(buff.begin(), size);
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommSendControlSystem(buff);
							}
							else
								resetSocket = true;
						}
						else
							resetSocket = true;
					}
					break;
				default:
					Log::alert(String()+"Unknown code: " + code);
			}
			mLastCmdRcvTime.setTime();
			Log::alert("done");
		}
		else
			resetSocket = true;

		if(resetSocket)
		{
			mMutex_socketTCP.lock();
			mSocketTCP->close();
			mSocketTCP = NULL;
			mConnected = false;
			mMutex_socketTCP.unlock();

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onCommConnectionLost();
		}

		mMutex_socketTCP.lock();
		newPacketReady = mSocketTCP != NULL && mSocketTCP->pollRead(0);
		mMutex_socketTCP.unlock();
	}
}

int CommManager::receiveUDP(tbyte* data, int size)
{
	uint32 addr;
	int port;
	int received = mSocketUDP->receiveFrom(data, size, addr, port);

	if(received > 0)
	{
		mAddrPC = addr;
		mPortPC = port;
	}

	return received;
}

int CommManager::receiveTCP(tbyte* data, int size)
{
	if(!mConnected)
		return 0;
	mMutex_socketTCP.lock();
	int received= mSocketTCP->receive(data, size);
	while(received > 0 && received < size)
	{
		Log::alert(String("Saving teh world"));
		received += mSocketTCP->receive(data+received, size-received);
	}
	if(received != size)
	{
		String str = String()+"Read failure: " + received + " vs " + size;
		Log::alert(str);
	}
	mMutex_socketTCP.unlock();

	return received;
}

bool CommManager::receivePacket(Packet &pck, int size)
{
	Collection<tbyte> buff(size);

	int result = receiveUDP(buff, size);
	if(result > 0)
		pck.deserialize(buff);

	if(pck.size != result)
	{
		String s = String()+"Only received " + result + " vs " + pck.size;
		Log::alert(s);
	}

	return pck.size == result;
}

bool CommManager::sendLogFile(const char* filename)
{
	if(mSocketTCP==NULL)
		return false;

	Log::alert(String("Sending log file"));
	mMutex_socketTCP.lock();
	FileStream logFile(filename, FileStream::Open_BIT_READ);
	int buffSize = 1<<10;
	char buff[buffSize];
	int bytesRead = 1;
	int code = COMM_LOG_FILE_DATA;
	mSocketTCP->send((tbyte*)&code, sizeof(code));
	while(bytesRead > 0)
	{
		bytesRead = logFile.read((tbyte*)&buff,buffSize);

		if(bytesRead >0)
		{
//			String str = String() + "Sending " + bytesRead + " bytes";
//			Log::alert(str);
			mSocketTCP->send((tbyte*)&bytesRead, sizeof(bytesRead));
			mSocketTCP->send((tbyte*)buff,bytesRead);
		}
	}
	bytesRead= -1;
//	String str = String() + "Sending " + bytesRead + " bytes";
//	Log::alert(str);
	mSocketTCP->send((tbyte*)&bytesRead, sizeof(bytesRead));
	logFile.close();
	mMutex_socketTCP.unlock();

	return true;
}

} // namespace Quadrotor
} // namespace ICSL
