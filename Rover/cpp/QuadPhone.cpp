#include "QuadPhone.h"
#include <algorithm>
#include "TNT/jama_cholesky.h"

#include "android/log.h"

using namespace std;
using namespace TNT;
using namespace toadlet;
using namespace ICSL::Constants;
using toadlet::uint64;
using toadlet::egg::String;

namespace ICSL {
namespace Quadrotor {
ChadPhone::ChadPhone() : 
	mRotViconToQuad(3,3,0.0), 
	mRotQuadToPhone(3,3,0.0), 
	mCurFeat(3,1,0.0),
	mDesFeat(3,1,0.0), 
	mFeatErrInt(3,1,0.0),
	mDesCentroid(3,1,0.0), 
	mGainImg(3,1,0.1),
	mGainFlow(3,1,0.5),
	mGainFF(3,1,0.0),
	mGainFlowInt(3,1,0.0),
	mGainAngularRateIBVS(3,1,1.0),
	mTorque2Cmd(3,1,0.0),
	mRotErr(3,1,0.0),
	mAttCmdOffset(3,1,0.0),
	mAttBias(3,1,0.0),
	mFlow(3,1,0.0),
	mFlowErrInt(3,1,0.0),
	mFlowErr_phone(3,1,0.0),
	mDesFlow(3,1,0.0),
	mCentroidErrInt(3,1,0.0),
	mLastImageAtt(3,1,0.0),
	mLastImageRotVel(3,1,0.0),
	mCentroid(3,1,0.0),
	mRotVelSum(3,1,0.0),
	mRotCamToPhone(3,3,0.0),
	mRotPhoneToCam(3,3,0.0),
	mRotViconToPhone(3,3,0.0),
	mCurAtt(3,1,0.0),
	mCurAngularVel(3,1,0.0)
{
	mRunnerIsDone = true;
	mDoInnerControl = false;
	mDoOuterControl = false;
	mDataIsSending = false;
	mImageIsSending = false;

	mMutex_cntl.unlock();
	mMutex_vision.unlock();
	mMutex_observer.unlock();
//	mConnectedToArduino = false;

	mThrottlePC = 0;
	mThrottleIbvs = 0;
	mDesiredAtt = Array2D<double>(3,1,0.0);
	mDesAngularVelPC = Array2D<double>(3,1,0.0);
	mDesAngularVelIbvs = Array2D<double>(3,1,0.0);

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

	mLastMotorVal[0] = mLastMotorVal[1] = mLastMotorVal[2] = mLastMotorVal[3] = 0;
	mMotorTrim[0] = mMotorTrim[1] = mMotorTrim[2] = mMotorTrim[3] = 0;

//	mCurCntlType = CNTL_PID;
	mCntlCalcTimeUS = 0;

	mUseIbvs = false;
//	mVisionProcessor.enableIbvs(false);
	mFirstImageProcessed = false;
	mIbvsControlReady = false;
	mBoxCenters.resize(4);
	mBoxFound.resize(4);
	for(int i=0; i<4; i++)
	{
		mBoxCenters[i] = cv::Point2f(0.0,0.0);
		mBoxFound[i] = false;
	}

	mFocalLength = 3.7*320.0/5.76; // (focal length mm)*(img width px)/(ccd width mm)
	mDesFeat[0][0] = 0;
	mDesFeat[1][0] = 0;
	mDesFeat[2][0] = 1;
	mDesCentroid[0][0] = 0;
	mDesCentroid[1][0] = 0;
	mDesCentroid[2][0] = 1;

	mStateVicon = Array2D<double>(12,1,0.0);
	
	mGainAngleIBVS = 1;
	mTorque2Cmd[0][0] = mTorque2Cmd[1][0] = 30;
	mTorque2Cmd[2][0] = 50;

	mBoxProjections.resize(4);
	for(int i=0; i<mBoxProjections.size(); i++)
		mBoxProjections[i] = Array2D<double>(3,1,0.0);

	mHeight = 1;
	mRho = 1;

	mGainDynamicIBVS = 0.1;

	mRotVelSum_count = 0;

	mThrottleBase = 500;

	mNumCpuCores = 1;

	mAssetManager = NULL;
}

ChadPhone::~ChadPhone()
{
}

void ChadPhone::initialize()
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

	initControllers();

	mObsvAngular.initialize();
	mObsvAngular.setStartTime(mStartTime);
	mObsvAngular.setQuadLogger(&mQuadLogger);
	mObsvAngular.start();
	mObsvAngular.doBurnIn(10e3);
	mObsvAngular.enableViconAttitude(false);
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
	mVisionProcessor.addListener(this);
	mVisionProcessor.setAttitudeObserver(&mObsvAngular);
	mVisionProcessor.start();

	this->start(); // this should spawn a separate thread running the run() function

	Log::alert("Initialized");
}

void ChadPhone::shutdown()
{
	mRunCommPC = false;
	mMutex_cntl.lock();
	mAttitudeThrustController.enableMotors(false);
	mMutex_cntl.unlock();
//	this->join(); // join doesn't work correctly in NDK
	toadlet::egg::System sys;
	while(!mRunnerIsDone)
	{
		Log::alert("Main waiting");
		sys.msleep(10);
	}

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

void ChadPhone::initControllers()
{
	mRollPID.setGain(100,0,0);
	mRollPID.setIntegratorLimit(100);

	mPitchPID.setGain(100,0,0);
	mPitchPID.setIntegratorLimit(100);

	mYawPID.setGain(100,0,0);
	mYawPID.setIntegratorLimit(100);

	mController.addSisoController(&mRollPID);
	mController.addSisoController(&mPitchPID);
	mController.addSisoController(&mYawPID);

	mRollPIDIbvs.setGain(100,0,0);
	mRollPIDIbvs.setIntegratorLimit(100);

	mPitchPIDIbvs.setGain(100,0,0);
	mPitchPIDIbvs.setIntegratorLimit(100);

	mYawPIDIbvs.setGain(100,0,0);
	mYawPIDIbvs.setIntegratorLimit(100);

	mControllerIbvs.addSisoController(&mRollPIDIbvs);
	mControllerIbvs.addSisoController(&mPitchPIDIbvs);
	mControllerIbvs.addSisoController(&mYawPIDIbvs);
}

void ChadPhone::run()
{
	mRunCommPC = true;
	System sys;
	mRunnerIsDone = false;
	class : public Thread{
			public: 
			void run(){ 
				parent->transmitDataUDP(); 
			}
			ChadPhone *parent;
		} dataSendThread;
	class : public Thread{
			public:
			void run(){ parent->transmitImage();}
			ChadPhone *parent;
			} imgSendThread;
	dataSendThread.parent = this;
	imgSendThread.parent = this;

	Array2D<int> cpuUsagePrev(1,7,0.0), cpuUsageCur(1,7,0.0);
	Time mLastCpuUsageTime;

	double alphaAttBias = 0.01/(1+0.01);
	while(mRunCommPC) 
	{
		if(mDoOuterControl)
			runOuterControlLoop();
//		if(mDoInnerControl)
//			runInnerControlLoop();
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


		if(!mUseIbvs)
		{
			double alphaThrott = 0.05;
			mThrottleBase = alphaThrott*mThrottlePC+(1-alphaThrott)*mThrottleBase;
//			mMutex_observer.lock();
//			mAttBias = alphaAttBias*mObsvAngular.getCurAttitude()+(1.0-alphaAttBias)*mAttBias;
//			mMutex_observer.unlock();
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
				mQuadLogger.addLine(str,PC_UPDATES);
			}
			cpuUsagePrev.inject(cpuUsageCur);
		}

		sys.msleep(1);
	}

	Log::alert(String("----------------- ChadPhone runner dead -------------"));
	mRunnerIsDone = true;
}


void ChadPhone::transmitDataUDP()
{
//	mLastDataSendTime.setTime();
	if(!mCommManager.pcIsConnected())
	{
		mDataIsSending = false;
		return;
	}

	Packet pArduinoStatus, pUseMotors, pState, pDesState, pGyro, pAccel, pComp, pBias, pCntl, pIntMem, pCntlType, pCntlCalcTime;
	Packet pImgProcTime, pImgMoment, pDesImgMoment, pUseIbvs;
	mMutex_cntl.lock();
	int arduinoStatus;
	mMutex_cntl.lock();
	if(mAttitudeThrustController.isMotorInterfaceConnected())
		arduinoStatus = 0;
	else
		arduinoStatus = 1;

	Array2D<double> desAtt = mAttitudeThrustController.getDesAttitude();
	mMutex_cntl.unlock();
//	if(mSocketArduino == NULL)
//		arduinoStatus = 0;
//	else if(mSocketArduino->connected())
//		arduinoStatus = 1;
//	else
//		arduinoStatus = -1;
	pArduinoStatus.dataInt32.push_back(arduinoStatus); 
	pArduinoStatus.type = COMM_ARDUINO_STATUS;

//	pUseMotors.dataBool.push_back(mUseMotors);
//	pUseMotors.type = COMM_USE_MOTORS;
	
	mMutex_observer.lock();
//	Array2D<double> curAtt = mObsvAngular.getCurAttitude().copy();
//	Array2D<double> curVel = mObsvAngular.getCurVel().copy();
	Array2D<double> curAtt = mCurAtt.copy();
	Array2D<double> curVel = mCurAngularVel.copy();
	Array2D<double> curBias = mObsvAngular.getBias().copy();
	mMutex_observer.unlock();
	Array2D<double> curState = stackVertical(curAtt, curVel);

	pState.dataFloat.resize(6);
	for(int i=0; i<6; i++)
		pState.dataFloat[i] = curState[i][0];
	pState.type = COMM_STATE_PHONE;

	Array2D<double> curDesState(6,1,0.0); 
	for(int i=0; i<3; i++)
		curDesState[i][0] = desAtt[i][0];

	pDesState.dataFloat.resize(6);
	for(int i=0; i<6; i++)
		pDesState.dataFloat[i] = curDesState[i][0];
	pDesState.type = COMM_DESIRED_STATE;

	pImgMoment.dataFloat.resize(6);
	pDesImgMoment.dataFloat.resize(6);
	for(int i=0; i<3; i++)
	{
		pImgMoment.dataFloat[i] = 0;
		pDesImgMoment.dataFloat[i] = mDesFeat[i][0];
	}
	for(int i=3; i<6; i++)
	{
//		pImgMoment.dataFloat[i] = mStateVicon[9+(i-3)][0];
pImgMoment.dataFloat[i] = 0;
		pDesImgMoment.dataFloat[i] = 0;
	}
	pImgMoment.type = COMM_IMAGE_STATE;
	pDesImgMoment.type = COMM_DESIRED_IMAGE_STATE;

	pCntl.dataInt32.resize(4);
	mMutex_cntl.lock();
	Collection<uint16> cmds = mAttitudeThrustController.getLastMotorCmds();
	mMutex_cntl.unlock();
	for(int i=0; i<4; i++)
		pCntl.dataInt32[i] = cmds[i];
	pCntl.type = COMM_MOTOR_VAL;

	pIntMem.dataFloat.resize(3);
	pIntMem.dataFloat[0] = mRollPID.getIntegratorMemory();
	pIntMem.dataFloat[1] = mPitchPID.getIntegratorMemory();
	pIntMem.dataFloat[2] = mYawPID.getIntegratorMemory();
	pIntMem.type = COMM_INT_MEM;

	pCntlType.type = COMM_CNTL_TYPE;
	mMutex_cntl.lock();
	int cntlType = mTranslationController.getControlType();
	mMutex_cntl.unlock();
	pCntlType.dataInt32.push_back(cntlType);

	pCntlCalcTime.type = COMM_CNTL_CALC_TIME;
	pCntlCalcTime.dataInt32.push_back(mCntlCalcTimeUS);

	pImgProcTime.type = COMM_IMGPROC_TIME;
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

	uint64 time = mStartTime.getElapsedTimeMS();
	pArduinoStatus.time = time;
//	pUseMotors.time = time;
	pState.time = time;
	pDesState.time = time;
	pImgMoment.time = time;
	pDesImgMoment.time = time;
	pGyro.time = time;
	pAccel.time = time;
	pComp.time = time;
	pBias.time = time;
	pCntl.time = time;
	pIntMem.time = time;
	pCntlType.time = time;
	pCntlCalcTime.time = time;
	pImgProcTime.time = time;
	pUseIbvs.time = time;

	mCommManager.transmitUDP(pArduinoStatus);
//	mCommManager.transmitUDP(pUseMotors);
	mCommManager.transmitUDP(pState);
	mCommManager.transmitUDP(pDesState);
	mCommManager.transmitUDP(pImgMoment);
	mCommManager.transmitUDP(pDesImgMoment);
	mCommManager.transmitUDP(pGyro);
	mCommManager.transmitUDP(pAccel);
	mCommManager.transmitUDP(pBias);
	mCommManager.transmitUDP(pComp);
	mCommManager.transmitUDP(pCntl);
	mCommManager.transmitUDP(pIntMem);
	mCommManager.transmitUDP(pCntlType);
	mCommManager.transmitUDP(pCntlCalcTime);
	mCommManager.transmitUDP(pImgProcTime);
	mCommManager.transmitUDP(pUseIbvs);

	mDataIsSending = false;
}

void ChadPhone::transmitImage()
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

	if(img.channels() == 3 && mUseIbvs)
	{
		// draw desired moment
		cvtColor(img,img, CV_HSV2BGR);

		cv::Mat alpha(img.size(), CV_8UC4, cv::Scalar(0,0,0,255));
		cvtColor(alpha,alpha, CV_BGRA2BGR);

		img += alpha;

		alpha.release();
	}

	// now send it
	code = 2000;
	numRows = img.rows;
	numCols = img.cols;
	numChannels = img.channels();
	type = img.type();
	vector<unsigned char> buff;
	cv::imencode(".jpg",img,buff,params);
	mCommManager.transmitImageBuffer(numRows, numCols, numChannels, type, buff);
	mImageIsSending = false;
}

// Controller from Mahony, Corke, and Hamel, "Dynamics Image-Based Visual Servo Control Using Centroid and Optic Flow Features"
void ChadPhone::runOuterControlLoop()
{
	mDoOuterControl = false;
	double dt = mLastOuterLoopCntlTime.getElapsedTimeUS()/1.0e6;
	mLastOuterLoopCntlTime.setTime();
	mMutex_observer.lock();
//	Array2D<double> curAtt= mObsvAngular.getCurAttitude();
	Array2D<double> curAtt = mCurAtt.copy();
	mMutex_observer.unlock();

//	Array2D<double> curRotMat = matmult(createRotMat(2,curAtt[2][0]),matmult(createRotMat(1,curAtt[1][0]), createRotMat(0,curAtt[0][0])));
	Array2D<double> curRotMat = createRotMat_ZYX(curAtt[2][0], curAtt[1][0], curAtt[0][0]);

	Collection<Array2D<double> > boxProjections(4);
	Collection<Array2D<double> > opticalFlow;
	Array2D<double> centroid(3,1,0.0);
	float avgLength = 0;
	int numEdges= 0;
	Array2D<double> Q(3,3,0.0);
	Array2D<double> nomDir(3,1,0.0);
	nomDir[2][0] = 1;
	mMutex_vision.lock();
		Array2D<double> imgAtt_cam = matmult(mRotPhoneToCam, mLastImageAtt);
		Array2D<double> imgRotVel_cam = matmult(mRotPhoneToCam, mLastImageRotVel);
//		Array2D<double> imgAttRotMat_cam = matmult(createRotMat(2,imgAtt_cam[2][0]),matmult(createRotMat(1,imgAtt_cam[1][0]), createRotMat(0,imgAtt_cam[0][0])));
		Array2D<double> imgAttRotMat_cam = createRotMat_ZYX(imgAtt_cam[2][0],imgAtt_cam[1][0],imgAtt_cam[0][0]);
		nomDir = matmult(transpose(imgAttRotMat_cam), nomDir);
		for(int i=0; i<mBoxCenters.size(); i++)
		{
			Array2D<double> proj(3,1);
			proj[0][0] = mBoxCenters[i].x;
			proj[1][0] = mBoxCenters[i].y;
			proj[2][0] = mFocalLength;
			proj = 1.0/norm2(proj)*proj;
			centroid += proj;
			boxProjections[i] = proj.copy();
			double cAlpha = matmultS(transpose(proj), nomDir); // recall that proj and nomDir are unit vectors already
			Array2D<double> eye = createIdentity(3);
			Array2D<double> pi = eye-matmult(proj, transpose(proj));
			Q += cAlpha*pi;

			if(i > 0)
			{
				if(mBoxFound[i] && mBoxFound[i-1])
				{
					numEdges++;
					avgLength += sqrt(pow(boxProjections[i][0][0]-boxProjections[i-1][0][0],2) +
							pow(boxProjections[i][1][0]-boxProjections[i-1][1][0],2));

				}
			}

		}
		if(mBoxFound[0] && mBoxFound[mBoxFound.size()-1])
		{
			numEdges++;
			avgLength += sqrt(pow(boxProjections[boxProjections.size()-1][0][0]-boxProjections[0][0][0],2) +
					pow(boxProjections[boxProjections.size()-1][1][0]-boxProjections[0][1][0],2));
		}
		if(numEdges == 0)
		{
			mMutex_vision.unlock();
			mThrottleIbvs = mThrottlePC;
//			onImageLost();
			return;
		}
		avgLength /= numEdges;

		for(int i=0; i<boxProjections.size(); i++)
			mBoxProjections[i].inject(boxProjections[i]);

		Array2D<double> centroidVel;
		if(!mIbvsControlReady) // first time through
			centroidVel = Array2D<double>(3,1,0.0);
		else
			centroidVel = 1.0/dt*(centroid-mCentroid);
		mCentroid.inject(centroid);

		Array2D<double> centroidErr = centroid-matmult(transpose(imgAttRotMat_cam),mDesCentroid);
		mCentroidErrInt += dt*(centroidErr);
		Array2D<double> desFlow = mGainImg*centroidErr;
		Array2D<double> desFlowRate = 1.0/dt*(desFlow-mDesFlow);
//		for(int i=0; i<desFlowRate.dim1(); i++)
//		{
//			if(desFlowRate[i][0] > 0.2)
//			{
//				desFlow[i][0] = mDesFlow[i][0]+0.2*dt;
//				desFlowRate[i][0] = 0.2;
//			}
//			else if(desFlowRate[i][0] < -0.2)
//			{
//				desFlow[i][0] = mDesFlow[i][0]-0.2*dt;
//				desFlowRate[i][0] = -0.2;
//			}
//		}
		mDesFlow.inject(desFlow);

		Array2D<double> B = centroidVel+cross(imgRotVel_cam, centroid); 
		JAMA::Cholesky<double> cholQ(Q);
		Array2D<double> flow = -1.0*cholQ.solve(B);
		double alpha = dt/(0.1+dt);
		alpha = 0.2;
		flow = alpha*flow+(1.0-alpha)*mFlow;
mMutex_vicon.lock();
flow = matmult(mRotPhoneToCam, matmult(mRotViconToPhone, submat(mStateVicon,9,11,0,0)));
mMutex_vicon.unlock();
		mFlow.inject(flow);
		Array2D<double> flowErr = flow-desFlow;

		Array2D<double> Q_phone = matmult(mRotCamToPhone, Q);
		Array2D<double> flow_phone= matmult(mRotCamToPhone, flow);
		Array2D<double> desFlow_phone = matmult(mRotCamToPhone, desFlow);
		Array2D<double> desFlowRate_phone = matmult(mRotCamToPhone, desFlowRate);
		Array2D<double> centroidErr_phone = matmult(mRotCamToPhone, centroidErr);
		Array2D<double> nomDir_phone = matmult(mRotCamToPhone, nomDir);
		Array2D<double> flowErr_phone = matmult(mRotCamToPhone, flowErr);
	
		Array2D<double> flowErrVel_phone = 1.0/dt*(flowErr_phone-mFlowErr_phone);
		mFlowErr_phone.inject(flowErr_phone);

	mMutex_vision.unlock();

//if(mDoInnerControl)
//	runInnerControlLoop();

	// actual control calc
	mMutex_cntl.lock();
		mFlowErrInt += dt*flowErr_phone;
		Array2D<double> nuTilde = -1.0*(flowErr_phone+
										mGainFlowInt*mFlowErrInt+
										mGainFF*desFlowRate_phone);
		nuTilde[2][0] = -GRAVITY;
 		double angleLimit = 0.2;
		double desRoll = asin(-mGainFlow[1][0]*nuTilde[1][0]/norm2(nuTilde));
		desRoll = min(angleLimit, max(-angleLimit, desRoll));
		desRoll += mAttCmdOffset[0][0];
		desRoll += mAttBias[0][0];

		double desPitch = asin(mGainFlow[0][0]*nuTilde[0][0]/norm2(nuTilde)/cos(desRoll));
		desPitch = min(angleLimit, max(-angleLimit, desPitch));
		desPitch += mAttCmdOffset[1][0];
		desPitch += mAttBias[1][0];

		mDesiredAtt[0][0] = desRoll;
		mDesiredAtt[1][0] = desPitch;
		mDesiredAtt[2][0] = 0;
		
		// high norm means high altitude
		double desNorm = norm2(mDesCentroid);
		double curNorm = norm2(centroid);
		mThrottleIbvs = mThrottlePC;
//		mThrottleIbvs = mThrottleBase-mGainImg[2][0]*(curNorm-desNorm);
		mDesAngularVelIbvs.inject(mDesAngularVelPC);

		Array2D<double> rotErr = mRotErr.copy();
	mMutex_cntl.unlock();

	{
		String str1 = String()+" "+mStartTime.getElapsedTimeMS()+"\t-610\t";
		for(int i=0; i<centroid.dim1(); i++)
			str1 = str1+centroid[i][0]+"\t";
		mQuadLogger.addLine(str1,CAM_RESULTS);

		String str3 = String()+" "+mStartTime.getElapsedTimeMS()+"\t-612\t";
		for(int i=0; i<curAtt.dim1(); i++)
			str3 = str3+curAtt[i][0]+"\t";
		for(int i=0; i<flow_phone.dim1(); i++)
			str3 = str3+flow_phone[i][0]+"\t";
		mQuadLogger.addLine(str3,CAM_RESULTS);

		String str5 = String()+" "+mStartTime.getElapsedTimeMS()+"\t-614\t";
		for(int i=0; i<desFlow_phone.dim1(); i++)
			str5 = str5+desFlow_phone[i][0]+"\t";
		mQuadLogger.addLine(str5,CAM_RESULTS);

		String str6 = String()+" "+mStartTime.getElapsedTimeMS()+"\t-615\t";
		for(int i=0; i<nuTilde.dim1(); i++)
			str6 = str6+nuTilde[i][0]+"\t";
		mQuadLogger.addLine(str6,CAM_RESULTS);
	}

	mIbvsControlReady = true;
}

// tracks target angular velocities
// void ChadPhone::runInnerControlLoop()
// {
// 	mDoInnerControl = false;
// 	// burn in time to let the observers settle
// 	if(mObsvAngular.doingBurnIn())
// 	{
// 		mLastInnerLoopCntlTime.setTime();
// 		return;
// 	}
// 
// 	Time curTime;
// 	double dt =  Time::calcDiffUS(mLastInnerLoopCntlTime,curTime)/1.0e6;
// 	if(dt > 1e-3)
// 	{
// 		Time calcStartTime;
// 
// 		mMutex_cntl.lock();
// 		int throttle;
// 		Array2D<double> desVel;
// 		mMutex_observer.lock();
// //		Array2D<double> curAtt = mObsvAngular.getCurAttitude();
// //		Array2D<double> curAngularVel= mObsvAngular.getCurVel();
// 		Array2D<double> curAtt = mCurAtt.copy();
// 		Array2D<double> curAngularVel = mCurAngularVel;
// 		mMutex_observer.unlock();
// 
// 		// outer loop control wants the average rotional velocity
// 		// for now be lazy and assume roughly constant dt
// 		mRotVelSum += curAngularVel;
// 		mRotVelSum_count++;
// 
// 		double cntl[] = {0,0,0,0};
// 		Array2D<double> cntlOffset(3,1,0.0);
// 		if(mUseIbvs && mIbvsControlReady)
// 		{
// 			throttle = (int)(mThrottleIbvs+0.5);
// //		 	desVel = mDesAngularVelPC.copy();
// //			cntlOffset = mController.calcControlSignal(curAngularVel-desVel);
// 
//  			Array2D<double> curRotMat = matmult(createRotMat(2,curAtt[2][0]), matmult(createRotMat(1,curAtt[1][0]), createRotMat(0,curAtt[0][0])));
//  //			Array2D<double> curRotMat = createRotMat_ZYX(curAtt[2][0],curAtt[1][0],curAtt[0][0]);
//  			Array2D<double> desRot = matmult(createRotMat(1,mDesiredAtt[1][0]), createRotMat(0,mDesiredAtt[0][0]));
//  //			Array2D<double> desRot = createRotMat_ZYX(0.0,mDesiredAtt[1][0],mDesiredAtt[0][0]);
//  
//  			// see my Feb 12, 2012 notes for controller description
//  			Array2D<double> rotMatErr = matmult(transpose(desRot), curRotMat); // will be eye if no error
//  			Array2D<double> rotMatErr_AS = rotMatErr-transpose(rotMatErr); // anti-symmetric part of error matrix
//  			mRotErr[0][0] = rotMatErr_AS[2][1];
//  			mRotErr[1][0] = rotMatErr_AS[0][2];
//  			mRotErr[2][0] = rotMatErr_AS[1][0];
//  
//  			Array2D<double> tau = -mGainAngleIBVS*mRotErr-mGainAngularRateIBVS*curAngularVel; // * operator for matrix multiplication is element-wise
//  			cntlOffset = mTorque2Cmd*tau;
// 		}
// 		else
// 		{
// //			throttle = (int)(mThrottlePC+0.5);
// //		 	desVel = mDesAngularVelPC.copy();
// 			Array2D<double> motorVals(4,1,0.0);
// 			if(mMotorInterface.isMotorsEnabled())
// 			{
// 				for(int i=0; i<4; i++)
// 					motorVals[i][0] = mLastMotorVal[i]-1000-mMotorTrim[i];
// 			}
// //			Array2D<double> u = mTranslationController.calcControl();
// 			Array2D<double> u(4,1,0.0);
// 			throttle = u[0][0];
// 			desVel = submat(u,1,3,0,0);
// 			mDesAngularVelPC.inject(desVel);
// //printArray(transpose(u),"u: \t");
// 			cntlOffset = mController.calcControlSignal(curAngularVel-desVel);
// 		}
// 
// 		cntl[0] = -cntlOffset[0][0] - cntlOffset[1][0] + cntlOffset[2][0];
// 		cntl[1] = -cntlOffset[0][0] + cntlOffset[1][0] - cntlOffset[2][0];
// 		cntl[2] =  cntlOffset[0][0] + cntlOffset[1][0] + cntlOffset[2][0];
// 		cntl[3] =  cntlOffset[0][0] - cntlOffset[1][0] - cntlOffset[2][0];
// 	
// 		mMutex_observer.lock();
// 		mObsvTranslational.setMotorCmds(cntl);
// 		mMutex_observer.unlock();
// 		uint16 motorCmds[4];
// 
// 		motorCmds[0] = (cntl[0] + 1000 + throttle + 0.5) + mMotorTrim[0];
// 		motorCmds[1] = (cntl[1] + 1000 + throttle + 0.5) + mMotorTrim[1];
// 		motorCmds[2] = (cntl[2] + 1000 + throttle + 0.5) + mMotorTrim[2];
// 		motorCmds[3] = (cntl[3] + 1000 + throttle + 0.5) + mMotorTrim[3];
// 
// 		for(int i=0; i<4; i++)
// 			mLastMotorVal[i] = motorCmds[i];
// 		mLastInnerLoopCntlTime.setTime();
// 
// 		Time calcFinishTime;
// 		mCntlCalcTimeUS = Time::calcDiffUS(calcStartTime,calcFinishTime);
// 
// 		Collection<uint16> cmds(4);
// 		if(mCommManager.pcIsConnected())
// 		{
// 			for(int i=0; i<4; i++)
// 				cmds[i] = motorCmds[i];
// 		}
// 		else
// 		{
// 			for(int i=0; i<4; i++)
// 				cmds[i] = 1000;
// 		}
// 		mMotorInterface.sendCommand(cmds);
// 
// 		Array2D<double> desAttState(6,1,0.0);
// 		Array2D<double> desTranState = mTranslationController.getDesiredState();
// 		Array2D<double> curTranState = mTranslationController.getCurState();
// 		mMutex_cntl.unlock();
// 		Array2D<double> desState = stackVertical(desAttState, desTranState);
// 		Array2D<double> curAttState = stackVertical(curAtt,curAngularVel);
// 		Array2D<double> curState = stackVertical(curAttState,curTranState);
// 
// 		String s1=String() + " " + Time::calcDiffMS(mStartTime,curTime) + "\t" + "-1000" +"\t" + throttle + "\t";
// 		for(int i=0; i<4; i++)
// 			s1 = s1+cntl[i] + "\t";
// 		s1 = s1+mCntlCalcTimeUS;
// 
// 		String s2=String() + " " + Time::calcDiffMS(mStartTime,curTime) + "\t" + "-1001" +"\t";
// 		for(int i=0; i<desState.dim1(); i++)
// 			s2 = s2 + desState[i][0] + "\t";
// 
// 		String s3=String() + " " + Time::calcDiffMS(mStartTime,curTime) + "\t" + "-1002" +"\t";
// 		for(int i=0; i<curState.dim1(); i++)
// 			s3 = s3+curState[i][0] + "\t";
// 
// 		String s4=String() + Time::calcDiffMS(mStartTime,curTime) + "\t" + "-1003" +"\t";
// 		mMutex_observer.lock(); Array2D<double> bias = mObsvAngular.getBias(); mMutex_observer.unlock();
// 		for(int i=0; i<bias.dim1(); i++)
// 			s4 = s4+bias[i][0] + "\t";
// 
// 		mQuadLogger.addLine(s1,MOTORS);
// 		mQuadLogger.addLine(s2,STATE_DES);
// 		mQuadLogger.addLine(s3,STATE);
// 		mQuadLogger.addLine(s4,OBSV_BIAS);
// 	}
// }

void ChadPhone::onObserver_AngularUpdated(Array2D<double> const &att, Array2D<double> const &angularVel)
{
	mMutex_observer.lock();
	mCurAtt.inject(att);
	mCurAngularVel.inject(angularVel);
	mMutex_observer.unlock();

	mDoInnerControl = true;
}

void ChadPhone::startLogging(){
	mQuadLogger.setFilename("log.txt");
	mQuadLogger.start();
}

void ChadPhone::stopLogging()
{
	mQuadLogger.close();
}

void ChadPhone::setLogFilename(String name)
{
	mQuadLogger.setFilename(name);
}

void ChadPhone::setLogDir(String dir)
{
	mQuadLogger.setDir(dir);
	Log::alert("Log dir set to: " + mQuadLogger.getFullPath());
}

void ChadPhone::onNewCommRateCmd(toadlet::egg::Collection<float> const &data)
{
	mMutex_cntl.lock();
	mThrottlePC = (uint16)data[0];
//	mDesAngularVelPC[0][0] = data[1];
//	mDesAngularVelPC[1][0] = data[2];
//	mDesAngularVelPC[2][0] = data[3];
//	mDesiredAtt[0][0] = 0;
//	mDesiredAtt[1][0] = 0;
//	mDesiredAtt[2][0] = 0;
//
//	mDesiredAtt = matmult(mRotQuadToPhone,mDesiredAtt);
//	mDesAngularVelPC = matmult(mRotQuadToPhone,mDesAngularVelPC);
	mMutex_cntl.unlock();
}

// void ChadPhone::onNewCommMotorOn()
// {
// 	mTranslationController.reset();
// 	mMotorInterface.enableMotors(true);
// }
// 
// void ChadPhone::onNewCommMotorOff()
// {
// 	Log::alert(String("Turning motors off"));
// 	mMotorInterface.enableMotors(false);
// }

void ChadPhone::onNewCommGainPID(float const rollPID[3], float const pitchPID[3], float const yawPID[3])
{
	mMutex_cntl.lock();
	mRollPID.setGain( rollPID[0], rollPID[1], rollPID[2]);
	mPitchPID.setGain(pitchPID[0], pitchPID[1], pitchPID[2]);
	mYawPID.setGain(  yawPID[0], yawPID[1], yawPID[2]);

	mRollPID.resetMemory();
	mPitchPID.resetMemory();
	mYawPID.resetMemory();

	mRollPIDIbvs.setGain( rollPID[0], rollPID[1], rollPID[2]);
	mPitchPIDIbvs.setGain(pitchPID[0], pitchPID[1], pitchPID[2]);
	mYawPIDIbvs.setGain(  yawPID[0], yawPID[1], yawPID[2]);
//	mRollPIDIbvs.setGain( rollPID[0], 0, rollPID[2]);
//	mPitchPIDIbvs.setGain(pitchPID[0], 0, pitchPID[2]);
//	mYawPIDIbvs.setGain(  yawPID[0], 0, yawPID[2]);

	mRollPIDIbvs.resetMemory();
	mPitchPIDIbvs.resetMemory();
	mYawPIDIbvs.resetMemory();
	mMutex_cntl.unlock();

	Log::alert("Gain values updated");
	String str = String() + " " + mStartTime.getElapsedTimeMS() + "\t-100\t";
	mQuadLogger.addLine(str,PC_UPDATES);
}

void ChadPhone::onNewCommMotorTrim(int const trim[4])
{
	mMutex_cntl.lock();
	mMotorTrim[0] = trim[0];
	mMotorTrim[1] = trim[1];
	mMotorTrim[2] = trim[2];
	mMotorTrim[3] = trim[3];
	mMutex_cntl.unlock();

	{
		String str= String()+ "Trim values updated: ";
		for(int i=0; i<4; i++)
			str = str + trim[i] + "\t";
		Log::alert(str);
	}
	String str = String() + " " + mStartTime.getElapsedTimeMS() + "\t-110\t";
	for(int i=0; i<4; i++)
		str = str + trim[i] + "\t";
	mQuadLogger.addLine(str,PC_UPDATES);
}

void ChadPhone::onNewCommTimeSync(int time)
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
	mQuadLogger.addLine(str,PC_UPDATES);
}

void ChadPhone::onNewCommLogTransfer()
{
	mQuadLogger.close();
//	sendLogFile(mSocketTCP, mQuadLogger.getFullPath().c_str());
	Log::alert("Before sending log file");
	mCommManager.sendLogFile(mQuadLogger.getFullPath().c_str());
	Log::alert("After sending log file");
	mQuadLogger.start();
}

// void ChadPhone::onNewCommSendMuControl(Collection<tbyte> const &buff)
// {
// 	mMutex_cntl.lock();
// 	mCntlSys.deserialize(buff);
// 
// 	int numIn = mCntlSys.getNumInputs();
// 	int numState = mCntlSys.getNumStates();
// 	int numOut = mCntlSys.getNumOutputs();
// 
// 	mCntlSys.setCurState(Array2D<double>(numState,1,0.0));
// 	mMutex_cntl.unlock();
// 
// 	String str = String()+"Received mu controller system: ";
// 	str = str +numIn+" inputs x "+numOut+" outputs x "+numState+" states";
// 	Log::alert(str);
// }

//void ChadPhone::onNewCommControlType(uint16 cntlType)
//{
//	mMutex_cntl.lock();
//	mCurCntlType = cntlType;
//	int numStates = mCntlSys.getNumStates();
//	mCntlSys.setCurState(Array2D<double>(numStates,1,0.0));
//	mMutex_cntl.unlock(); 
//
//	Log::alert(String()+"Control type set to "+cntlType);
//}

void ChadPhone::onNewCommLogMask(uint32 mask)
{
	mQuadLogger.setMask(mask);
	Log::alert(String()+"Log mask set to "+mQuadLogger.getMask());
}

void ChadPhone::onNewCommLogClear()
{
	mQuadLogger.clearLog();
	Log::alert(String()+"Log cleared");
}

void ChadPhone::onNewCommStateVicon(Collection<float> const &data)
{
	mMutex_vicon.lock();
	for(int i=0; i<mStateVicon.dim1(); i++)
		mStateVicon[i][0] = data[i];
	mMutex_vicon.unlock();
}

void ChadPhone::onCommConnectionLost()
{
	Log::alert("gah");
	mMutex_cntl.lock();
	mUseIbvs = false;
	mFirstImageProcessed = false;
	mIbvsControlReady = false;
	mVisionProcessor.enableIbvs(false);
	for(int i=0; i<3; i++)
	{
		mDesiredAtt[i][0] = 0;
		mDesAngularVelPC[i][0] = 0;
		mDesAngularVelIbvs[i][0] = 0;
	}
	mThrottlePC = 0;
	mThrottleIbvs = 0;

	mMutex_cntl.unlock();
}

// gains in the in the order ....
// image gain [0], [1], [2]
// flow gain [0], [1], [2]
// flow int gain [0], [1], [2]
// att cmd offset [0], [1], [2]
// angle rate gain [0], [1], [2]
// angle gain [0]
void ChadPhone::onNewCommIbvsGains(Collection<float> const &gains)
{
	if(gains.size() < 20)
	{
		Log::alert("WARNING: Not enough gains sent for IBVS.");
		return;
	}
	mMutex_vision.lock();
	for(int i=0; i<3; i++)
	{
		mGainImg[i][0] = gains[i];
		mGainFlow[i][0] = gains[i+3];
		mGainFlowInt[i][0] = gains[i+6];
		mGainFF[i][0] = gains[i+9];
		mAttCmdOffset[i][0] = gains[i+12];
		mGainAngularRateIBVS[i][0] = gains[i+15];
	}
	mGainAngleIBVS = gains[18];
	mGainDynamicIBVS = gains[19];
	
	mMutex_vision.unlock();

	{
		String str = "Received gains:\n";
		for(int i=0; i<gains.size(); i++)
		{
			if(i%3==0)
				str = str+"\n";
			str = str+gains[i]+"\t";
		}
		Log::alert(str);
	}
}

void ChadPhone::onNewCommUseIbvs(bool useIbvs)
{
	mUseIbvs = useIbvs;
	mVisionProcessor.enableIbvs(useIbvs);
	if(useIbvs)
		mLastOuterLoopCntlTime.setTime();
	mFirstImageProcessed = false;
	mIbvsControlReady = false;

	mRollPID.resetMemory();
	mPitchPID.resetMemory();
	mYawPID.resetMemory();
	mRollPIDIbvs.resetMemory();
	mPitchPIDIbvs.resetMemory();
	mYawPIDIbvs.resetMemory();
}

void ChadPhone::onNewCommDesiredImageMoment(Collection<float> const &data)
{
}

void ChadPhone::copyImageData(cv::Mat *m)
{
	if(!mRunCommPC) // use this as an indicator that we are shutting down
		return;

	mVisionProcessor.getLastImage(m);

	if(m->channels() == 3)
	{
		// draw desired moment
		cvtColor(*m,*m, CV_HSV2BGR);

		cv::Mat alpha(m->size(), CV_8UC4, cv::Scalar(0,0,0,255));
		cvtColor(alpha,alpha, CV_BGRA2BGR);

		*m += alpha;

		alpha.release();
	}
	else if(m->channels() == 1)
		cvtColor(*m,*m,CV_GRAY2BGR);

}

// assume p is sized for 16 elements
Collection<int> ChadPhone::getVisionParams()
{
	mMutex_vision.lock();
	Collection<int> p = mVisionProcessor.getVisionParams();
	mMutex_vision.unlock();
	return p;
}

void ChadPhone::setVisionParams(Collection<int> p)
{
	mMutex_vision.lock();
	mVisionProcessor.setVisionParams(p);
	mMutex_vision.unlock();
}

void ChadPhone::onImageProcessed(Collection<cv::Point2f> const &boxCenters, Collection<bool> const &boxFound, Array2D<double> const &imgAtt, Array2D<double> const &rotVel)
{
	mMutex_vision.lock();
	mLastImageAtt.inject(imgAtt);
	mLastImageRotVel.inject(rotVel);
	if(mBoxCenters.size() != boxCenters.size())
		mBoxCenters.resize(boxCenters.size());

	for(int i=0; i<mBoxCenters.size(); i++)
	{
		mBoxCenters[i].x = boxCenters[i].x-160;
		mBoxCenters[i].y = boxCenters[i].y-120;
	}

	if(mBoxFound.size() != boxFound.size())
		mBoxFound.resize(boxFound.size());
	for(int i=0; i<mBoxFound.size(); i++)
		mBoxFound[i] = boxFound[i];

	if(!mFirstImageProcessed)
	{
		Collection<Array2D<double> > boxProjections;
		Array2D<double> centroid(3,1,0.0);
		float avgLength = 0;
		for(int i=0; i<mBoxCenters.size(); i++)
		{
			Array2D<double> proj(3,1);
			double fov = 75.0/180.0*PI;
			double scale = tan(fov/2.0)/160.0;
			proj[0][0] = scale*mBoxCenters[i].x;
			proj[1][0] = scale*mBoxCenters[i].y;
			proj[2][0] = 1;
			proj = 1.0/norm2(proj)*proj;
			centroid += proj;
		}

		// assume we always want the desired centroid to point down
		mDesCentroid[0][0] = 0;
		mDesCentroid[1][0] = 0;
		mDesCentroid[2][0] = norm2(centroid); // set the target "height" to the current "height"
		/////////////////////////// HACK //////////////////////
//		mDesCentroid[2][0] = 3.8;
		/////////////////////////// HACK //////////////////////

		{
			String str = "mDesCentroid: \t";
			for(int i=0; i<mDesCentroid.dim1(); i++)
				str = str+mDesCentroid[i][0]+"\t";
			Log::alert(str);
		}
		mFirstImageProcessed = true;
	}
	mMutex_vision.unlock();

	if(mUseIbvs)
		mDoOuterControl = true;
//		runOuterControlLoop();
}

Array2D<double> ChadPhone::getGyroValue()
{
	Array2D<double> temp;
	mMutex_observer.lock();
	if(mObsvAngular.doingBurnIn())
		temp = Array2D<double>(3,1,0.0);
	else
		temp = mObsvAngular.getLastGyro().copy();
	mMutex_observer.unlock();
	return temp;
}

Array2D<double> ChadPhone::getAccelValue()
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

Array2D<double> ChadPhone::getMagValue()
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

Array2D<double> ChadPhone::getAttitude()
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
Array2D<int> ChadPhone::getCpuUsage()
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
