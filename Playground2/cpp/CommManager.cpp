#include "CommManager.h"
#include "errno.h"
#include "android/log.h"

using namespace toadlet;

namespace ICSL{
namespace Quadrotor{
CommManager::CommManager()
{
	mLastCmdRcvTime.clear();
	mLastPacketTime.clear();

	mAddrPC = 0;
	mPortPC = 0;

	mDone = true; // need this to be true in case the run thread never gets started

	mConnected = false;

	mSocketUDP = NULL;
	mSocketTCP = NULL;
	mServerSocketTCP = NULL;
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
	mSocketUDP->listen(1);
	mSocketUDP->setBlocking(false);
}

void CommManager::shutdown()
{
	Log::alert("------------------ Comm Manager shutdown started --------------------");
	mRun = false;
	toadlet::egg::System sys;
	while(!mDone) // since join doesn't seem to work correctly in NDK
	{
		Log::alert("CommManager Waiting");
		sys.msleep(10);
	}

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
	System sys;
	while(mRun)
	{
		mMutex_socketTCP.lock();
		mConnected = mSocketTCP != NULL && mSocketTCP->connected();
		mMutex_socketTCP.unlock();

		if(mConnected && mLastCmdRcvTime.getElapsedTimeMS() > 500)
		{
			mConnected = false;
			for(int i=0; i<mListeners.size(); i++)
			{
				if(mLastCmdRcvTime.getElapsedTimeMS() > 50)
					Log::alert("Where'd you go?");
				mListeners[i]->onCommConnectionLost();
			}
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
			if(mSocketTCP != NULL && mSocketTCP->connected())
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
				mSocketUDP->listen(1);
				mSocketUDP->setBlocking(false);
			}
			mMutex_socketTCP.unlock();
			mLastCmdRcvTime.setTime();
		}

		sys.msleep(1);
	}

	mDone = true;
	Log::alert("------------------ Comm manager runner is dead -------------------");
}

void CommManager::transmitUDP(Packet &pck)
{
	Collection<tbyte> buff;
	pck.serialize(buff);
	if(mSocketUDP != NULL && mAddrPC != 0)
	{
		mMutex_socketUDP.lock();
		mSocketUDP->sendTo(buff.begin(), buff.size(), mAddrPC, mPortPC);
		mMutex_socketUDP.unlock();
	}
}

void CommManager::transmitImageBuffer(uint32 numRows, uint32 numCols, uint32 numChannels, uint32 type, vector<unsigned char> const &buff)
{
	if(!mConnected)
		return;
//	mMutex_socketTCP.lock();
//	bool connected = mSocketTCP == NULL || mSocketTCP->connected() == false;
//	mMutex_socketTCP.unlock();
//	if(connected)
//		return;

	uint32 code = COMM_IMG_DATA;
	uint32 size = buff.size()*sizeof(unsigned char);
	mMutex_socketTCP.lock();
	mSocketTCP->send((tbyte*)&code, sizeof(code));
	mSocketTCP->send((tbyte*)&numRows, sizeof(numRows));
	mSocketTCP->send((tbyte*)&numCols, sizeof(numCols));
	mSocketTCP->send((tbyte*)&numChannels, sizeof(numChannels));
	mSocketTCP->send((tbyte*)&type, sizeof(type));
	mSocketTCP->send((tbyte*)&size, sizeof(size));
	mSocketTCP->send((tbyte*)&buff.front(), size);
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
	bool newPacketReady = mSocketTCP != NULL && mSocketTCP->connected() && mSocketTCP->pollRead(0);
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
				case COMM_KALMAN_ATT_BIAS:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							Collection<float> bias(size);
							received = receiveTCP((tbyte*)&(bias[0]), size*sizeof(float));
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommAttBias(bias[0], bias[1], bias[2]);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_KALMAN_ATT_BIAS_ADAPT_RATE:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							Collection<float> rate(size);
							received = receiveTCP((tbyte*)&(rate[0]), size*sizeof(float));
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommAttBiasAdaptRate(rate);
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
							{
								Log::alert(String()+"Listener "+i);
								mListeners[i]->onNewCommBarometerZeroHeight(h);
							}
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
		newPacketReady = mSocketTCP != NULL && mSocketTCP->connected() && mSocketTCP->pollRead(0);
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
	if(mSocketTCP==NULL || mSocketTCP->connected()==false)
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
			String str = String() + "Sending " + bytesRead + " bytes";
			Log::alert(str);
			mSocketTCP->send((tbyte*)&bytesRead, sizeof(bytesRead));
			mSocketTCP->send((tbyte*)buff,bytesRead);
		}
	}
	bytesRead= -1;
	String str = String() + "Sending " + bytesRead + " bytes";
	Log::alert(str);
	mSocketTCP->send((tbyte*)&bytesRead, sizeof(bytesRead));
	logFile.close();
	mMutex_socketTCP.unlock();

	return true;
}

} // namespace Quadrotor
} // namespace ICSL