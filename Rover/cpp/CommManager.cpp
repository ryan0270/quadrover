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
	mServerSocketTCP->bind(1312);
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
	mSocketUDP->bind(1312);
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

Log::alert("CommManager::shutdown() - 1");
	if(mSocketUDP != NULL) mSocketUDP->close();
Log::alert("CommManager::shutdown() - 2");
	if(mSocketTCP != NULL) mSocketTCP->close();
Log::alert("CommManager::shutdown() - 3");
	if(mServerSocketTCP != NULL) mServerSocketTCP->close();
Log::alert("CommManager::shutdown() - 4");

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
				Log::alert(String("Connected to PC"));
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

	uint32 code = 2000;
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
				case COMM_RATE_CMD:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommRateCmd(pck.dataFloat);
					mLastCmdRcvTime.setTime();
					break;
				case COMM_STATE_VICON:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommStateVicon(pck.dataFloat);
					mLastCmdRcvTime.setTime();
					break;
				case COMM_DES_STATE:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommDesState(pck.dataFloat);
					mLastCmdRcvTime.setTime();
				case COMM_DESIRED_IMAGE_MOMENT:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommDesiredImageMoment(pck.dataFloat);
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
				case COMM_CNTL_GAIN_PID:
					{
						float rollPID[3], pitchPID[3], yawPID[3];
						bool received 			= receiveTCP((tbyte*)rollPID, 3*sizeof(float));
						if(received) received 	= receiveTCP((tbyte*)pitchPID, 3*sizeof(float));
						if(received) received 	= receiveTCP((tbyte*)yawPID, 3*sizeof(float));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommGainPID(rollPID, pitchPID, yawPID);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_POS_CNTL_GAINS:
					{
						float gainP[12], gainI[12], gainILimit[12];
						float mass, forceScaling;
						bool received 			= receiveTCP((tbyte*)gainP, 12*sizeof(float));
						if(received) received 	= receiveTCP((tbyte*)gainI, 12*sizeof(float));
						if(received) received 	= receiveTCP((tbyte*)gainILimit, 12*sizeof(float));
						if(received) received 	= receiveTCP((tbyte*)&mass, sizeof(mass));
						if(received) received 	= receiveTCP((tbyte*)&forceScaling, sizeof(forceScaling));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommPosControllerGains(gainP, gainI, gainILimit, mass, forceScaling);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_ATT_GAINS:
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
				case COMM_KALMANFILTER_POS_MEAS_STD:
					{
						float std;
						bool received = receiveTCP((tbyte*)&std,sizeof(std));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommKalmanPosMeasStd(std);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_KALMANFILTER_VEL_MEAS_STD:
					{
						float std;
						bool received = receiveTCP((tbyte*)&std,sizeof(std));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommKalmanVelMeasStd(std);
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
				case COMM_OBSV_GAIN:
					{
						double gainP, gainI, gravWeight, compWeight, gravBandwidth;
						bool received 			= receiveTCP((tbyte*)&gainP, sizeof(gainP));
						if(received) received 	= receiveTCP((tbyte*)&gainI, sizeof(gainI));
						if(received) received 	= receiveTCP((tbyte*)&gravWeight, sizeof(gravWeight));
						if(received) received 	= receiveTCP((tbyte*)&compWeight, sizeof(compWeight));
						if(received )
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommObserverGain(gainP, gainI, gravWeight, compWeight, gravBandwidth);
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
				case COMM_LOG_TRANSFER:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommLogTransfer();
					break;
				case COMM_SEND_CNTL_SYSTEM:
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
				case COMM_CNTL_SYS_GAINS:
					{
						int size;
						bool received = receiveTCP((tbyte*)&size,sizeof(size));
						if(received)
						{
							Collection<float> gains(size);
							received = receiveTCP((tbyte*)gains.begin(), size*sizeof(float));
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommControlSystemGains(gains);
							}
							else
								resetSocket = true;
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_CNTL_TYPE:
					{
						uint16 cntlType;
						bool received = receiveTCP((tbyte*)&cntlType, sizeof(uint16));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommControlType(cntlType);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_BOX_COLOR_MIN:
					{
						int count;
						bool received = receiveTCP((tbyte*)&count, sizeof(count));
						if(received)
						{
							Collection<int> data(count);
							bool received = receiveTCP((tbyte*)&(data[0]),count*sizeof(int));
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommImgProcBoxColorCenter(data);
							}
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_BOX_COLOR_MAX:
					{
						int count;
						bool received = receiveTCP((tbyte*)&count, sizeof(count));
						if(received)
						{
							Collection<int> data(count);
							bool received = receiveTCP((tbyte*)&(data[0]),count*sizeof(int));
							if(received)
							{
								for(int i=0; i<mListeners.size(); i++)
									mListeners[i]->onNewCommImgProcBoxColorHalfRange(data);
							}
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_SAT_MIN:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcSatMin(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_SAT_MAX:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcSatMax(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_VAL_MIN:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcValMin(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_VAL_MAX:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcValMax(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_CIRC_MIN:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcCircMin(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_CIRC_MAX:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcCircMax(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_CONV_MIN:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcConvMin(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_CONV_MAX:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcConvMax(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_AREA_MIN:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcAreaMin(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGPROC_AREA_MAX:
					{
						int data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgProcAreaMax(data);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_IMGVIEW_TYPE:
					{
						uint16 data;
						bool received = receiveTCP((tbyte*)&data, sizeof(data));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommImgViewType(data);
						}
						else
							resetSocket = true;
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
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommLogClear();
					break;
				case COMM_MOTOR_FORCE_SCALING:
					{
						float k;
						bool received = receiveTCP((tbyte*)&k,sizeof(k));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommForceScaling(k);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_MOTOR_TORQUE_SCALING:
					{
						float k;
						bool received = receiveTCP((tbyte*)&k,sizeof(k));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommTorqueScaling(k);
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
				case COMM_IBVS_GAINS:
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
									mListeners[i]->onNewCommIbvsGains(gains);
							}
							else
								resetSocket = true;
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
				case COMM_SET_YAW_ZERO:
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewCommSetYawZero();
					break;
				case COMM_ATT_BIAS:
					{
						float roll, pitch, yaw;
						bool received 			= receiveTCP((tbyte*)&roll, sizeof(roll));
						if(received) received 	= receiveTCP((tbyte*)&pitch, sizeof(pitch));
						if(received) received 	= receiveTCP((tbyte*)&yaw, sizeof(yaw));
						if(received )
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommAttBias(roll, pitch, yaw);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_ATT_BIAS_GAIN:
					{
						float gain;
						bool received = receiveTCP((tbyte*)&gain, sizeof(gain));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommAttBiasGain(gain);
						}
						else
							resetSocket = true;
					}
					break;
				case COMM_FORCE_SCALING_GAIN:
					{
						float gain;
						bool received = receiveTCP((tbyte*)&gain, sizeof(gain));
						if(received)
						{
							for(int i=0; i<mListeners.size(); i++)
								mListeners[i]->onNewCommForceScalingGain(gain);
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

			String s = String() + "Something else: ";
			Log::alert(s);
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
	int code = 1000;
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
