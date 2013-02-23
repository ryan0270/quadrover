#include "Rover.h"

using namespace std;
using namespace TNT;
using namespace toadlet;
using namespace ICSL::Constants;
using toadlet::uint64;
using toadlet::egg::String;

namespace ICSL {
namespace Quadrotor {
Rover::Rover() : 
	mRotViconToQuad(3,3,0.0), 
	mRotQuadToPhone(3,3,0.0), 
	mRotCamToPhone(3,3,0.0),
	mRotPhoneToCam(3,3,0.0),
	mRotViconToPhone(3,3,0.0),
	mCurAtt(3,1,0.0),
	mCurAngularVel(3,1,0.0)
{
	mRunnerIsDone = true;
	mDataIsSending = false;
	mImageIsSending = false;

//	mRotViconToQuad = createRotMat(0, (double)toadlet::egg::math::Math::PI);
//	mRotQuadToPhone = matmult(createRotMat(2,-0.25*toadlet::egg::math::Math::PI),
//				  			  createRotMat(0,(double)toadlet::egg::math::Math::PI));
//	mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)toadlet::egg::math::Math::PI),
//							 createRotMat(0,(double)toadlet::egg::math::Math::PI));
//	mRotPhoneToCam = transpose(mRotCamToPhone);
	mRotViconToQuad = createRotMat(0, (double)PI);
	mRotQuadToPhone = matmult(createRotMat(2,-0.25*PI),
				  			  createRotMat(0,(double)PI));
	mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
							 createRotMat(0,(double)PI));
	mRotPhoneToCam = transpose(mRotCamToPhone);
	mRotViconToPhone = matmult(mRotQuadToPhone, mRotViconToQuad);

	mNumCpuCores = 1;
}

Rover::~Rover()
{
}

void Rover::initialize()
{
	mCommManager.initialize();
	mCommManager.addListener(this);
	mCommManager.start();

	mMutex_cntl.lock();
	mTranslationController.setRotViconToPhone(mRotViconToPhone);
	mTranslationController.setStartTime(mStartTime);
	mTranslationController.setQuadLogger(&mQuadLogger);
	mTranslationController.initialize();
	mTranslationController.start();
	mCommManager.addListener(&mTranslationController);

	mAttitudeThrustController.setStartTime(mStartTime);
	mAttitudeThrustController.setQuadLogger(&mQuadLogger);
	mAttitudeThrustController.initialize();
	mAttitudeThrustController.start();
	mCommManager.addListener(&mAttitudeThrustController);
	mTranslationController.addListener(&mAttitudeThrustController);
	mMutex_cntl.unlock();

	mObsvAngular.initialize();
	mObsvAngular.setStartTime(mStartTime);
	mObsvAngular.setQuadLogger(&mQuadLogger);
	mObsvAngular.start();
	mObsvAngular.addListener(this);
	mObsvAngular.addListener(&mAttitudeThrustController);
	mCommManager.addListener(&mObsvAngular);

	mObsvTranslational.setQuadLogger(&mQuadLogger);
	mObsvTranslational.setStartTime(mStartTime);
	mObsvTranslational.setRotViconToPhone(mRotViconToPhone);
	mObsvTranslational.initialize();
	mObsvTranslational.start();
	mObsvTranslational.addListener(&mTranslationController);
	mObsvAngular.addListener(&mObsvTranslational);
	mCommManager.addListener(&mObsvTranslational);
	mAttitudeThrustController.addListener(&mObsvTranslational);

	mVisionProcessor.setStartTime(mStartTime);
	mVisionProcessor.setQuadLogger(&mQuadLogger);
	mCommManager.addListener(&mVisionProcessor);
	mVisionProcessor.start();

	mSensorManager.initialize();
	mSensorManager.setStartTime(mStartTime);
	mSensorManager.setQuadLogger(&mQuadLogger);
	mSensorManager.start();
	mObsvAngular.addListener(&mSensorManager);
	mSensorManager.addListener(&mObsvAngular);
	mSensorManager.addListener(&mObsvTranslational);
	mSensorManager.addListener(&mVisionProcessor);

	mQuadLogger.setStartTime(mStartTime);

	this->start();

	Log::alert("Initialized");
}

void Rover::shutdown()
{
	mRunCommPC = false;
	mMutex_cntl.lock();
	mAttitudeThrustController.enableMotors(false);
	mMutex_cntl.unlock();
//	this->join(); 
	toadlet::egg::System sys;
	while(!mRunnerIsDone)
	{
		Log::alert("Main waiting");
		sys.msleep(10);
	}

	mSensorManager.shutdown();
	mMutex_cntl.lock();
	mAttitudeThrustController.shutdown();
	mTranslationController.shutdown();
	mMutex_cntl.unlock();

	mCommManager.shutdown(); // mCommManager is only ever accessed via the run thread or via functions returning bools so doesn't have a mutex

	mVisionProcessor.shutdown();
	mMutex_observer.lock(); 
	mObsvAngular.shutdown(); 
	mObsvTranslational.shutdown();
	mMutex_observer.unlock();


	stopLogging();

	Log::alert(String("----------------- really dead -------------"));
}

void Rover::run()
{
	mRunCommPC = true;
	System sys;
	mRunnerIsDone = false;
	class : public Thread{
			public: 
			void run(){ 
				parent->transmitDataUDP(); 
			}
			Rover *parent;
		} dataSendThread;
	class : public Thread{
			public:
			void run(){ parent->transmitImage();}
			Rover *parent;
			} imgSendThread;
	dataSendThread.parent = this;
	imgSendThread.parent = this;

	Array2D<int> cpuUsagePrev(1,7,0.0), cpuUsageCur(1,7,0.0);
	Time mLastCpuUsageTime;

	double alphaAttBias = 0.01/(1+0.01);
	while(mRunCommPC) 
	{
		if(!mDataIsSending && mLastDataSendTime.getElapsedTimeMS() > 100)
		{
			mDataIsSending = true;
			// assume the last send finished already
			dataSendThread.join();
			dataSendThread.start();
			mLastDataSendTime.setTime();
		}
		if(!mImageIsSending && mLastImageSendTime.getElapsedTimeMS() > 200)
		{
			mImageIsSending = true;
			imgSendThread.join(); 
			imgSendThread.start();
			mLastImageSendTime.setTime();
		}


		if(mLastCpuUsageTime.getElapsedTimeMS() > 100)
		{
			cpuUsageCur = getCpuUsage();
			mLastCpuUsageTime.setTime();
			if(cpuUsagePrev.dim1() != cpuUsageCur.dim1())
				cpuUsagePrev = Array2D<int>(cpuUsageCur.dim1(), cpuUsageCur.dim2(),0.0);
			double maxTotal= 0;
			Collection<double> usage;
			for(int i=1; i<cpuUsageCur.dim1(); i++)
			{
				if(cpuUsageCur[i][0] == 0 || cpuUsagePrev[i][0] == 0) // this cpu is turned off
					usage.push_back(0);
				else
				{
					double total = 0;
					for(int j=0; j<cpuUsageCur.dim2(); j++)
						total += cpuUsageCur[i][j] - cpuUsagePrev[i][j];
					double used = 0;
					for(int j=0; j<3; j++)
						used += cpuUsageCur[i][j] - cpuUsagePrev[i][j];

					maxTotal = max(maxTotal, total);
					usage.push_back(used/total);
				}
			}
			String str = String()+" "+mStartTime.getElapsedTimeMS()+"\t-2000\t";
			if(cpuUsageCur[0][0] != 0 && cpuUsagePrev[0][0] != 0)
			{
				// overall total has to be handled separately since it only counts cpus that were turned on
				// Assume that the total avaible was 4*maxTotal
				double used = 0;
				for(int j=0; j<3; j++)
					used += cpuUsageCur[0][j]-cpuUsagePrev[0][j];
				str = str+(used/maxTotal/(double)mNumCpuCores)+"\t";

				// finish making log string
				for(int i=0; i<usage.size(); i++)
					str = str+usage[i]+"\t";
				mQuadLogger.addLine(str,LOG_FLAG_PC_UPDATES);
			}
			cpuUsagePrev.inject(cpuUsageCur);
		}

		sys.msleep(1);
	}

	Log::alert(String("----------------- Rover runner dead -------------"));
	mRunnerIsDone = true;
}


void Rover::transmitDataUDP()
{
//	mLastDataSendTime.setTime();
	if(!mCommManager.pcIsConnected())
	{
		mDataIsSending = false;
		return;
	}

	Packet pArduinoStatus, pUseMotors, pState, pDesState, pGyro, pAccel, pComp, pBias, pCntl, pIntMemPos, pIntMemTorque;
	Packet pImgProcTime, pUseIbvs;
	Packet pTime;
	mMutex_cntl.lock();
	int arduinoStatus;
	mMutex_cntl.lock();
	MotorInterface* motorInterface = mAttitudeThrustController.getMotorInterface();
	if(motorInterface->isConnected())
		arduinoStatus = 1;
	else
		arduinoStatus = 0;

	pUseMotors.dataBool.push_back(motorInterface->isMotorsEnabled());
	pUseMotors.type = COMM_USE_MOTORS;

	Array2D<double> desAtt = mAttitudeThrustController.getDesAttitude();
	Array2D<double> desTransState = mTranslationController.getDesiredState();
	mMutex_cntl.unlock();

	pArduinoStatus.dataInt32.push_back(arduinoStatus); 
	pArduinoStatus.type = COMM_ARDUINO_STATUS;

	mMutex_observer.lock();
	Array2D<double> curAtt = mCurAtt.copy();
	Array2D<double> curVel = mCurAngularVel.copy();
	Array2D<double> curBias = mObsvAngular.getBias();
	Array2D<double> curTransState = mTranslationController.getCurState();
	mMutex_observer.unlock();
	Array2D<double> curState = stackVertical(stackVertical(curAtt, curVel), curTransState);
	Array2D<double> desState = stackVertical(stackVertical(desAtt, Array2D<double>(3,1,0.0)), desTransState);

	pState.dataFloat.resize(curState.dim1());
	for(int i=0; i<pState.dataFloat.size(); i++)
		pState.dataFloat[i] = curState[i][0];
	pState.type = COMM_STATE_PHONE;

	pDesState.dataFloat.resize(desState.dim1());
	for(int i=0; i<pDesState.dataFloat.size(); i++)
		pDesState.dataFloat[i] = desState[i][0];
	pDesState.type = COMM_DESIRED_STATE;

	pCntl.dataInt32.resize(4);
	mMutex_cntl.lock();
	Collection<uint8> cmds = mAttitudeThrustController.getLastMotorCmds();
	mMutex_cntl.unlock();
	for(int i=0; i<4; i++)
		pCntl.dataInt32[i] = cmds[i];
	pCntl.type = COMM_MOTOR_VAL;

	pIntMemPos.dataFloat.resize(3);
	pIntMemPos.dataFloat[0] = 0;
	pIntMemPos.dataFloat[1] = 0;
	pIntMemPos.dataFloat[2] = 0;
	pIntMemPos.type = COMM_INT_MEM_POS;

	pIntMemTorque.dataFloat.resize(3);
	pIntMemTorque.dataFloat[0] = 0;
	pIntMemTorque.dataFloat[1] = 0;
	pIntMemTorque.dataFloat[2] = 0;
	pIntMemTorque.type = COMM_INT_MEM_TORQUE;

	pImgProcTime.type = COMM_IMGPROC_TIME_US;
	pImgProcTime.dataInt32.push_back(mVisionProcessor.getImageProcTimeUS());

	pUseIbvs.type = COMM_USE_IBVS;
	pUseIbvs.dataBool.push_back(mUseIbvs);
	mMutex_cntl.unlock();

	Array2D<double> lastGyro, lastAccel, lastCompass;
	mMutex_observer.lock();
	if(mObsvAngular.doingBurnIn())
	{
		lastGyro = Array2D<double>(3,1,0.0);
		lastAccel = Array2D<double>(3,1,0.0);
		lastCompass = Array2D<double>(3,1,0.0);
	}
	else
	{
		lastGyro = mObsvAngular.getLastGyro();
		lastAccel = mObsvAngular.getLastAccel();
		lastCompass = mObsvAngular.getLastMagnometer();
	}
	mMutex_observer.unlock();
	pGyro.dataFloat.resize(3);
	pGyro.dataFloat[0] = lastGyro[0][0];
	pGyro.dataFloat[1] = lastGyro[1][0];
	pGyro.dataFloat[2] = lastGyro[2][0];
	pGyro.type = COMM_GYRO;

	pAccel.dataFloat.resize(3);
	pAccel.dataFloat[0] = lastAccel[0][0];
	pAccel.dataFloat[1] = lastAccel[1][0];
	pAccel.dataFloat[2] = lastAccel[2][0];
	pAccel.type = COMM_ACCEL;

	pBias.dataFloat.resize(3);
	pBias.dataFloat[0] = curBias[0][0];
	pBias.dataFloat[1] = curBias[1][0];
	pBias.dataFloat[2] = curBias[2][0];
	pBias.type = COMM_OBSV_BIAS;

	pComp.dataFloat.resize(3);
	pComp.dataFloat[0] = lastCompass[0][0];
	pComp.dataFloat[1] = lastCompass[1][0];
	pComp.dataFloat[2] = lastCompass[2][0];
	pComp.type = COMM_MAGNOMETER;

	pTime.dataInt32.push_back(mStartTime.getElapsedTimeMS());
	pTime.type = COMM_HOST_TIME_MS;

	uint64 time = mStartTime.getElapsedTimeMS();
	pArduinoStatus.time = time;
	pUseMotors.time = time;
	pState.time = time;
	pDesState.time = time;
	pGyro.time = time;
	pAccel.time = time;
	pComp.time = time;
	pBias.time = time;
	pCntl.time = time;
	pIntMemPos.time = time;
	pIntMemTorque.time = time;
	pImgProcTime.time = time;
	pUseIbvs.time = time;
	pTime.time = time;

	mCommManager.transmitUDP(pArduinoStatus);
	mCommManager.transmitUDP(pUseMotors);
	mCommManager.transmitUDP(pState);
	mCommManager.transmitUDP(pDesState);
	mCommManager.transmitUDP(pGyro);
	mCommManager.transmitUDP(pAccel);
	mCommManager.transmitUDP(pBias);
	mCommManager.transmitUDP(pComp);
	mCommManager.transmitUDP(pCntl);
	mCommManager.transmitUDP(pIntMemPos);
	mCommManager.transmitUDP(pIntMemTorque);
	mCommManager.transmitUDP(pImgProcTime);
	mCommManager.transmitUDP(pUseIbvs);
	mCommManager.transmitUDP(pTime);

	mDataIsSending = false;
}

void Rover::transmitImage()
{
	if(!mCommManager.pcIsConnected())
	{
		mImageIsSending = false;
		return;
	}

	cv::Mat img;
	mVisionProcessor.getLastImage(&img);

	int code, numRows, numCols, numChannels, type, size;
	vector<int> params;
	params.push_back(CV_IMWRITE_JPEG_QUALITY);
	params.push_back(30); // 0 to 100, higher is better
//	params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//	params.push_back(3);

	// now send it
//	code = 2000;
	numRows = img.rows;
	numCols = img.cols;
	numChannels = img.channels();
	type = img.type();
	vector<unsigned char> buff;
	cv::imencode(".jpg",img,buff,params);
	mCommManager.transmitImageBuffer(numRows, numCols, numChannels, type, buff);
	mImageIsSending = false;
}

void Rover::onObserver_AngularUpdated(Array2D<double> const &att, Array2D<double> const &angularVel)
{
	mMutex_observer.lock();
	mCurAtt.inject(att);
	mCurAngularVel.inject(angularVel);
	mMutex_observer.unlock();
}

void Rover::startLogging(){
	mQuadLogger.setFilename("log.txt");
	mQuadLogger.start();
}

void Rover::stopLogging()
{
	mQuadLogger.close();
}

void Rover::setLogFilename(String name)
{
	mQuadLogger.setFilename(name);
}

void Rover::setLogDir(String dir)
{
	mQuadLogger.setDir(dir);
	Log::alert("Log dir set to: " + mQuadLogger.getFullPath());
}

void Rover::onNewCommTimeSync(int time)
{
	int curTime = (int)mStartTime.getElapsedTimeMS();
	int delta = curTime-time;

	uint64 chad = mStartTime.getMS() + delta;
	mMutex_cntl.lock();
	mStartTime.setTimeMS(chad);
	mMutex_vision.lock(); mVisionProcessor.setStartTime(mStartTime); mMutex_vision.unlock();
	mMutex_observer.lock(); 
	mObsvAngular.setStartTime(mStartTime); 
	mObsvTranslational.setStartTime(mStartTime);
	mMutex_observer.unlock();
	mMutex_cntl.lock();
	mTranslationController.setStartTime(mStartTime);
	mAttitudeThrustController.setStartTime(mStartTime);
	mMutex_cntl.unlock();
	String str = String()+" " + mStartTime.getElapsedTimeMS() + "\t-500\t" + delta;
	mMutex_cntl.unlock();
	mQuadLogger.addLine(str,LOG_FLAG_PC_UPDATES);
}

void Rover::onNewCommLogTransfer()
{
	mQuadLogger.close();
	Log::alert("Before sending log file");
	mCommManager.sendLogFile(mQuadLogger.getFullPath().c_str());
	Log::alert("After sending log file");
	mQuadLogger.start();
}

void Rover::onNewCommLogMask(uint32 mask)
{
	mQuadLogger.setMask(mask);
	Log::alert(String()+"Log mask set to "+mQuadLogger.getMask());
}

void Rover::onNewCommLogClear()
{
	mQuadLogger.clearLog();
	Log::alert(String()+"Log cleared");
}

void Rover::copyImageData(cv::Mat *m)
{
	if(!mRunCommPC) // use this as an indicator that we are shutting down
		return;

	mVisionProcessor.getLastImage(m);
}

// assume p is sized for 16 elements
Collection<int> Rover::getVisionParams()
{
	mMutex_vision.lock();
	Collection<int> p = mVisionProcessor.getVisionParams();
	mMutex_vision.unlock();
	return p;
}

void Rover::setVisionParams(Collection<int> p)
{
	mMutex_vision.lock();
	mVisionProcessor.setVisionParams(p);
	mMutex_vision.unlock();
}

Array2D<double> Rover::getGyroValue()
{
	Array2D<double> temp;
	mMutex_observer.lock();
	if(mObsvAngular.doingBurnIn())
		temp = Array2D<double>(3,1,0.0);
	else
		temp = mObsvAngular.getLastGyro();
	mMutex_observer.unlock();
	return temp;
}

Array2D<double> Rover::getAccelValue()
{
	Array2D<double> temp;
	mMutex_observer.lock();
	if(mObsvAngular.doingBurnIn())
		temp = Array2D<double>(3,1,0.0);
	else
		temp = mObsvAngular.getLastAccel().copy();
	mMutex_observer.unlock();
	return temp;
}

Array2D<double> Rover::getMagValue()
{
	Array2D<double> temp;
	mMutex_observer.lock();
	if(mObsvAngular.doingBurnIn())
		temp = Array2D<double>(3,1,0.0);
	else
		temp = mObsvAngular.getLastMagnometer().copy();
	mMutex_observer.unlock();
	return temp;
}

Array2D<double> Rover::getAttitude()
{
	Array2D<double> att;
	mMutex_observer.lock();
// 	att = mObsvAngular.getCurAttitude();
	att = mCurAtt.copy();
	mMutex_observer.unlock();

//	{
//		String str = "Att: \t";
//		for(int i=0; i<att.dim1(); i++)
//			str = str+att[i][0]+"\t";
//		Log::alert(str);
//	}
	return att;
}

// return array stores results obtained from /proc/stat
// cpu	user	nice	system	idle	iowait	irq		softirq
// cpu0	user0	nice0	system0	idle0	iowait0	irq0	softirq0
// cpu1	user1	nice1	system1	idle1	iowait1	irq1	softirq1
// ...
Array2D<int> Rover::getCpuUsage()
{
	Collection<String> lines;
	string line;
	ifstream file("/proc/stat");
	if(file.is_open())
	{
		while(file.good())
		{
			string tok;
			getline(file, line);
			stringstream stream(line);
			stream >> tok;
			if(strncmp("cpu",tok.c_str(),3) == 0) // this is a cpu line
				lines.push_back(String(line.c_str()));
		}
		file.close();
	}
	else
	{
		Log::alert("Failed to open /proc/stat");
	}

	// Note that since mobile phones tend to turn cores off for power, the number of 
	// cpu lines found may not actually match the number of cores on the device
	Array2D<int> data(mNumCpuCores+1, 7,0.0);
	for(int i=0; i<lines.size(); i++)
	{
		String tok;
		int index;
		int tokEnd= lines[i].find(' ');
		tok = lines[i].substr(0,tokEnd); // tok won't include the space
		if(tok.length() < 3)
		{
			Log::alert(String()+"getCpuUsage: token too short -- length = " + tok.length());
			return data;
		}
		else if(tok.length() == 3)
			index = 0;
		else
			index = tok.substr(3,tok.length()-3).toInt32()+1;
		String remainder = lines[i].substr(tokEnd+1,lines[i].length()-tokEnd);
		while(remainder.c_str()[0] == ' ')
			remainder = remainder.substr(1,remainder.length()-1);

		int j=0;
		tokEnd = remainder.find(' '); 
		while(tokEnd != String::npos && j < data.dim2())
		{
			tok = remainder.substr(0,tokEnd); // tok won't include the space
			data[index][j++] = tok.toInt32();
			remainder = remainder.substr(tokEnd+1,remainder.length()-tokEnd);
			while(remainder.c_str()[0] == ' ')
				remainder = remainder.substr(1,remainder.length()-1);
			tokEnd = remainder.find(' ');
		}

	}

	return data;
}

} // namespace Quadrotor
} // namespace ICSL
