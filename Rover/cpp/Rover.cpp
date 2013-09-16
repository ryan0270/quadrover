#include "Rover.h"


namespace ICSL {
namespace Quadrotor {
//using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

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

	mRotViconToQuad = createRotMat(0, (double)PI);
	mRotQuadToPhone = matmult(createRotMat(2,-0.25*PI),
				  			  createRotMat(0,(double)PI));
	mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
							 createRotMat(0,(double)PI));
	mRotPhoneToCam = transpose(mRotCamToPhone);
	mRotViconToPhone = matmult(mRotQuadToPhone, mRotViconToQuad);

	mNumCpuCores = 1;

	mPressure = 0;
	mPhoneTemp = 0;

//	mImageMatchData = NULL;
	mFeatureData = NULL;
	mTargetData = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
}

Rover::~Rover()
{
}

void Rover::initialize()
{
	int sched = SCHED_NORMAL;
	int maxPriority = sched_get_priority_max(sched);
	int minPriority = sched_get_priority_min(sched);

	mMotorInterface.setThreadPriority(sched,maxPriority);
	mSensorManager.setThreadPriority(sched,maxPriority);
	mObsvAngular.setThreadPriority(sched,maxPriority);
	mHeightSensor.setThreadPriority(sched.maxPriority-1);
	mObsvTranslational.setThreadPriority(sched,maxPriority-1);
	mVelocityEstimator.setThreadPriority(sched,maxPriority-1);
	mFeatureFinder.setThreadPriority(sched,maxPriority-2);
	mTargetFinder.setThreadPriority(sched,maxPriority-2);
	mTranslationController.setThreadPriority(sched,maxPriority-2);
	mAttitudeThrustController.setThreadPriority(sched,maxPriority-2);
	mCommManager.setThreadPriority(sched,minPriority);
	mQuadLogger.setThreadPriority(sched,minPriority);
	mVideoMaker.setThreadPriority(sched,minPriority);
	this->setThreadPriority(sched,minPriority);

	mCommManager.initialize();
	mCommManager.addListener(this);
	mCommManager.start();

	mMotorInterface.initialize();
	mMotorInterface.enableMotors(false);
	mMotorInterface.start();
	
	mMutex_cntl.lock();
	mTranslationController.setRotViconToPhone(mRotViconToPhone);
	mTranslationController.setStartTime(mStartTime);
	mTranslationController.setQuadLogger(&mQuadLogger);
	mTranslationController.initialize();
	mTranslationController.start();
	mCommManager.addListener(&mTranslationController);

	mAttitudeThrustController.setStartTime(mStartTime);
	mAttitudeThrustController.setQuadLogger(&mQuadLogger);
	mAttitudeThrustController.setMotorInterface(&mMotorInterface);
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
//	mAttitudeThrustController.addListener(&mObsvTranslational);

	mFeatureFinder.initialize();
	mFeatureFinder.setStartTime(mStartTime);
	mFeatureFinder.setQuadLogger(&mQuadLogger);
	mFeatureFinder.start();
	mSensorManager.addListener(&mFeatureFinder);
	mCommManager.addListener(&mFeatureFinder);
	mFeatureFinder.addListener(this);

	mTargetFinder.initialize();
	mTargetFinder.setStartTime(mStartTime);
	mTargetFinder.setQuadLogger(&mQuadLogger);
	mTargetFinder.start();
	mSensorManager.addListener(&mTargetFinder);
	mCommManager.addListener(&mTargetFinder);
	mTargetFinder.addListener(this);
	mTargetFinder.addListener(&mObsvTranslational);

	mVelocityEstimator.initialize();
	mVelocityEstimator.setStartTime(mStartTime);
	mVelocityEstimator.setQuadLogger(&mQuadLogger);
	mVelocityEstimator.setObserverTranslational(&mObsvTranslational);
	mVelocityEstimator.setRotPhoneToCam(mRotPhoneToCam);
	mVelocityEstimator.start();
	mVelocityEstimator.addListener(&mObsvTranslational);
	mFeatureFinder.addListener(&mVelocityEstimator);
	mCommManager.addListener(&mVelocityEstimator);

	mVideoMaker.initialize();
	mVideoMaker.start();
	mCommManager.addListener(&mVideoMaker);
	mSensorManager.addListener(&mVideoMaker);

	mSensorManager.setStartTime(mStartTime);
	mSensorManager.setQuadLogger(&mQuadLogger);
	mSensorManager.initialize();
	mSensorManager.setObserverAngular(&mObsvAngular);
	mSensorManager.start();
	mObsvAngular.addListener(&mSensorManager);
	mSensorManager.addListener(&mObsvAngular);
	mSensorManager.addListener(&mObsvTranslational);
	mSensorManager.addListener(this);

	mQuadLogger.setStartTime(mStartTime);

//	mMotorInterface.addListener(&mObsvTranslational);
	mMotorInterface.addListener(&mTranslationController);

	mNumCpuCores = android_getCpuCount();

	this->start();

	Log::alert("Initialized");
}

void Rover::shutdown()
{
	Log::alert("Main shutdown started");
	mQuadLogger.shutdown();
	mRunning = false;
	mMutex_cntl.lock();
	mMotorInterface.enableMotors(false);
	mMutex_cntl.unlock();
	while(!mRunnerIsDone)
		System::msleep(10);

	mMutex_cntl.lock();
	mTranslationController.shutdown();
	mAttitudeThrustController.shutdown();
	mMutex_cntl.unlock();

	mCommManager.shutdown();

	mVelocityEstimator.shutdown();
	mFeatureFinder.shutdown();
	mTargetFinder.shutdown();
	mVideoMaker.shutdown();
	mObsvAngular.shutdown(); 
	mObsvTranslational.shutdown(); 

//	mImageMatchData = NULL;
	mFeatureData = NULL;
	mTargetData = NULL;
	mSensorManager.shutdown();

	mMotorInterface.shutdown();

	Log::alert(String("----------------- really dead -------------"));
}

void Rover::run()
{
	mRunning = true;
	mRunnerIsDone = false;

	Array2D<int> cpuUsagePrev(1,7,0.0), cpuUsageCur(1,7,0.0);
	Time mLastCpuUsageTime;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	thread dataSendTh, imageSendTh;
	while(mRunning) 
	{
		if(!mDataIsSending && mLastDataSendTime.getElapsedTimeMS() > 100)
		{
			mDataIsSending = true;
			dataSendTh = thread(&Rover::transmitDataUDP, this);
			dataSendTh.detach();
			mLastDataSendTime.setTime();
		}
		if(!mImageIsSending && mLastImageSendTime.getElapsedTimeMS() > 200)
		{
			mImageIsSending = true;
			imageSendTh = thread(&Rover::transmitImage, this);
			imageSendTh.detach();
			mLastImageSendTime.setTime();
		}

		if(mLastCpuUsageTime.getElapsedTimeMS() > 1000)
		{
			cpuUsageCur = getCpuUsage(mNumCpuCores);
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

			if(cpuUsageCur[0][0] != 0 && cpuUsagePrev[0][0] != 0)
			{
				String str;
				// overall total has to be handled separately since it only counts cpus that were turned on
				// Assume that the total avaible was 4*maxTotal
				double used = 0;
				for(int j=0; j<3; j++)
					used += cpuUsageCur[0][j]-cpuUsagePrev[0][j];
				str = str+(used/maxTotal/(double)mNumCpuCores)+"\t";

				// finish making log string
				for(int i=0; i<usage.size(); i++)
					str = str+usage[i]+"\t";
				mQuadLogger.addEntry(Time(), LOG_ID_CPU_USAGE, str,LOG_FLAG_PC_UPDATES);
			}
			cpuUsagePrev.inject(cpuUsageCur);
		}

		System::msleep(10);
	}

	if(dataSendTh.joinable())
		dataSendTh.join();
	if(imageSendTh.joinable())
		imageSendTh.join();

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
	Packet pImgProcTime, pUseIbvs, pBarometerHeight, pPressure, pPhoneTemp;
	Packet pTime;
	mMutex_cntl.lock();
	int arduinoStatus;
	if(mMotorInterface.isConnected())
		arduinoStatus = 1;
	else
		arduinoStatus = 0;

	pUseMotors.dataBool.push_back(mMotorInterface.isMotorsEnabled());
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
	mMutex_cntl.lock(); Collection<uint16> cmds = mAttitudeThrustController.getLastMotorCmds(); mMutex_cntl.unlock();
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
	mMutex_vision.lock();
	pImgProcTime.dataInt32.push_back(mVelocityEstimator.getLastVisionDelayTimeUS());
	mMutex_vision.unlock();

	pUseIbvs.type = COMM_USE_IBVS;
	pUseIbvs.dataBool.push_back(mUseIbvs);

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

//	pBarometerHeight.dataFloat.push_back(mObsvTranslational.getBarometerHeight());
	pBarometerHeight.dataFloat.push_back(0);//mObsvTranslational.getBarometerHeight());
	pBarometerHeight.type = COMM_BAROMETER_HEIGHT;
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

	mMutex_data.lock();
	pTime.dataInt32.push_back(mStartTime.getElapsedTimeMS());
	pTime.type = COMM_HOST_TIME_MS;

	pPressure.dataFloat.push_back(mPressure);
	pPressure.type = COMM_PRESSURE;

	pPhoneTemp.dataFloat.push_back(mPhoneTemp);
	pPhoneTemp.type = COMM_PHONE_TEMP;

	Packet pNumFeatures;
	pNumFeatures.type = COMM_VISION_NUM_FEATURES;
	mMutex_vision.lock();
//	if(mImageMatchData != NULL)
//	{
//		mImageMatchData->lock();
//		if(mImageMatchData->featurePoints.size() > 0)
//			pNumFeatures.dataInt32.push_back(mImageMatchData->featurePoints[0].size());
//		else
//			pNumFeatures.dataInt32.push_back(0);
//		mImageMatchData->unlock();
//	}
	if(mFeatureData != NULL)
	{
//		mFeatureData->lock();
		if(mFeatureData->featurePoints.size() > 0)
			pNumFeatures.dataInt32.push_back(mFeatureData->featurePoints.size());
		else
			pNumFeatures.dataInt32.push_back(0);
//		mFeatureData->unlock();
	}
	else
		pNumFeatures.dataInt32.push_back(0);
	mMutex_vision.unlock();

	uint64 time = mStartTime.getElapsedTimeMS();
	mMutex_data.unlock();

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
	pBarometerHeight.time = time;
	pPressure.time = time;
	pPhoneTemp.time = time;
	pNumFeatures.time = time;

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
	mCommManager.transmitUDP(pBarometerHeight);
	mCommManager.transmitUDP(pPressure);
	mCommManager.transmitUDP(pPhoneTemp);
	mCommManager.transmitUDP(pNumFeatures);

	mDataIsSending = false;
}

void Rover::transmitImage()
{
//mImageIsSending = false;
//return;
	if(!mCommManager.pcIsConnected())
	{
		mImageIsSending = false;
		return;
	}

	cv::Mat img;
	mMutex_vision.lock();
//	mFeatureFinder.getLastImageAnnotated(&img);
	if(mObsvTranslational.isTargetFound() && mTargetData != NULL)
	{
//		mTargetData->lock();
		try{ mTargetData->imageAnnotatedData->imageAnnotated->copyTo(img); }
		catch(...) {Log::alert("copyTo error in Rover 1"); /*mTargetData->unlock();*/}
//		mTargetData->unlock();
	}
	else if(mFeatureData != NULL)
	{
//		mFeatureData->lock();
		try { mFeatureData->imageAnnotated->imageAnnotated->copyTo(img); }
		catch(...) {Log::alert("copyTo error in Rover 2"); /*mTargetData->unlock();*/}
//		mFeatureData->unlock();
	}
	mMutex_vision.unlock();

	if(img.rows == 0 || img.cols == 0)
	{
		mImageIsSending = false;
		return;
	}

	// save on transmitted data
	cv::resize(img, img, cv::Size(320,240));

	int code, numRows, numCols, numChannels, type, size;
	vector<int> params;
	params.push_back(CV_IMWRITE_JPEG_QUALITY);
	params.push_back(30); // 0 to 100, higher is better
//	params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//	params.push_back(3);

	// now send it
	numRows = img.rows;
	numCols = img.cols;
	numChannels = img.channels();
	type = img.type();
	vector<unsigned char> buff;
	cv::imencode(".jpg",img,buff,params);
	mCommManager.transmitImageBuffer(numRows, numCols, numChannels, type, buff);
	mImageIsSending = false;
}

void Rover::onObserver_AngularUpdated(const shared_ptr<SO3Data<double>> &attData, const shared_ptr<DataVector<double> > &angularVelData)
{
	mMutex_observer.lock();
	mCurAtt.inject(attData->rotation.getAnglesZYX());
	mCurAngularVel.inject(angularVelData->data);
	mMutex_observer.unlock();
}

void Rover::startLogging(){
	mQuadLogger.setFilename("log.txt");
	mQuadLogger.start();
}

void Rover::stopLogging()
{
	mQuadLogger.shutdown();
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
	mMutex_cntl.unlock();
	mMutex_observer.lock(); 
	mObsvAngular.setStartTime(mStartTime); 
	mObsvTranslational.setStartTime(mStartTime);
	mMutex_observer.unlock();
	mMutex_cntl.lock();
	mTranslationController.setStartTime(mStartTime);
	mAttitudeThrustController.setStartTime(mStartTime);
	mMutex_cntl.unlock();
	
	mSensorManager.setStartTime(mStartTime);
	mQuadLogger.setStartTime(mStartTime);

	mFeatureFinder.setStartTime(mStartTime);
	mTargetFinder.setStartTime(mStartTime);
	mVelocityEstimator.setStartTime(mStartTime);

	String str = String()+delta;
	mQuadLogger.addEntry(Time(), LOG_ID_TIME_SYNC, str,LOG_FLAG_PC_UPDATES);
}

void Rover::onNewCommLogTransfer()
{
//	mQuadLogger.close();
	mQuadLogger.pause();
	mCommManager.sendLogFile(mQuadLogger.getFullPath().c_str());
	mQuadLogger.resume();
//	mQuadLogger.start();
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

void Rover::onNewSensorUpdate(const shared_ptr<IData> &data)
{
	switch(data->type)
	{
		case DATA_TYPE_PRESSURE:
			mPressure = static_pointer_cast<Data<double> >(data)->data;
			break;
		case DATA_TYPE_PHONE_TEMP:
			mPhoneTemp = static_pointer_cast<DataPhoneTemp<double> >(data)->secTemp;
			break;
	}
}

void Rover::onFeaturesFound(const shared_ptr<ImageFeatureData> &data)
{
	mFeatureData = data;
}

void Rover::onTargetFound(const shared_ptr<ImageTargetFindData> &data)
{
	mTargetData = data;
}

void Rover::copyImageData(cv::Mat *m)
{
	if(!mRunning) // use this as an indicator that we are shutting down
		return;

	if(/*mObsvTranslational.isTargetFound() &&*/ mTargetData != NULL)
	{
//		mTargetData->lock();
		try{ mTargetData->imageAnnotatedData->imageAnnotated->copyTo(*m); }
		catch(...) {Log::alert("copyTo error in Rover 3"); /*mTargetData->unlock();*/}
//		mTargetData->unlock();
	}
	else if(mFeatureData != NULL)
	{
//		mFeatureData->lock();
		try{ mFeatureData->imageAnnotated->imageAnnotated->copyTo(*m); }
		catch(...) {Log::alert("copyTo error in Rover 4"); /*mFeatureData->unlock();*/}
//		mFeatureData->unlock();
	}
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
Array2D<int> Rover::getCpuUsage(int numCpuCores)
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
	Array2D<int> data(numCpuCores+1, 7,0.0);
	String tok, remainder;
	int index, tokEnd;
	for(int i=0; i<lines.size(); i++)
	{
		tokEnd= lines[i].find(' ');
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
		remainder = lines[i].substr(tokEnd+1,lines[i].length()-tokEnd);
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
