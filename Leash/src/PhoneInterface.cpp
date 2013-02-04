#include "PhoneInterface.h"


namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

PhoneInterface::PhoneInterface(QWidget *parent) : 
	QWidget(parent),
	mState(6,1,0.0), mDesiredState(6,1,0.0),
	mGyro(3,1,0.0), mAccel(3,1,0.0), mBias(3,1,0.0), mComp(3,1,0.0),
	mAttBias(3,1,0.0),
	mIntMemory(3,1,0.0),
	mGainCntlSys(6,0)
{
	setupUi(this);

	mIP = string("0.0.0.0");
	mPort = 1312;

	mGainP[0] = 1;
	mGainP[1] = mGainP[2] = 0;
	mGainI[0] = mGainI[1] = mGainI[2] = 0;
	mGainD[0] = mGainD[1] = mGainD[2] = 0;
	mObserverGainP = 50;
	mObserverGainI = 1;

	mArduinoStatus = 0;
//	mState = Array2D<double>(6,1,0.0);
//	mDesiredState = Array2D<double>(6,1,0.0);
	mStateImage.resize(6);
	mDesiredStateImage.resize(6);
	for(int i=0; i<mStateImage.size(); i++)
	{
		mStateImage[i] = 0;
		mDesiredStateImage[i] = 0;
	}
	mDesiredStateImage[0] = 160; mDesiredStateImage[1] = 120; mDesiredStateImage[2] = 0;
//	mGyro = Array2D<double>(3,1,0.0);
//	mAccel = Array2D<double>(3,1,0.0);
//	mBias= Array2D<double>(3,1,0.0);
//	mComp = Array2D<double>(3,1,0.0);
	mMotorValues[0] = mMotorValues[1] = mMotorValues[2] = mMotorValues[3] = 0;
	mMotorValuesIbvs[0] = mMotorValuesIbvs[1] = mMotorValuesIbvs[2] = mMotorValuesIbvs[3] = 0;
	mTimeMS = 0;

	mObserverWeights.push_back(.9);
	mObserverWeights.push_back(.1);
//	mGravBandwidth = 20;

//	mIntMemory = Array2D<double>(3,1,0.0);

	mMotorTrim[0] = mMotorTrim[1] = mMotorTrim[2] = mMotorTrim[3] = 0;

	mCntlSysFile = "";

	mCntlCalcTimeUS = 0;
	mImgProcTimeUS = 0;

	mFiltBoxColorMin.resize(4);
	mFiltBoxColorMax.resize(4);
	mFiltBoxColorMin[0] = 0; mFiltBoxColorMax[0] = 10;
	mFiltBoxColorMin[1] = 80; mFiltBoxColorMax[1] = 100;
	mFiltBoxColorMin[2] = 170; mFiltBoxColorMax[2] = 190;
	mFiltBoxColorMin[3] = 260; mFiltBoxColorMax[3] = 280;
	mFiltSatMin = 0; mFiltSatMax = 50;
	mFiltValMin = 0; mFiltValMax = 50;
	mFiltCircMin = 75; mFiltCircMax = 100;
	mFiltConvMin = 75; mFiltConvMax = 100;
	mFiltAreaMin = 10; mFiltAreaMax = 9999999;

	mLogMask = PC_UPDATES;
	
	mDeltaT = 0;

	mUseMotors = false;
	mUseIbvs = false;

	mAttBiasGain = 0;
	mForceScalingGain = 0;

	mKfPosMeasStdDev = 1e-3;
	mKfVelMeasStdDev = 1e-3;
}

PhoneInterface::~PhoneInterface()
{
	if(mSocketTCP != NULL)
		mSocketTCP->close();
	if(mSocketUDP != NULL)
		mSocketUDP->close();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Public functions
////////////////////////////////////////////////////////////////////////////////////////////////////
void PhoneInterface::initialize()
{
	connect(btnPhoneApply,SIGNAL(clicked()),this,SLOT(onBtnApply_clicked()));
	connect(btnPhoneResetConfig,SIGNAL(clicked()),this,SLOT(onBtnResetConfig_clicked()));
	connect(btnPhoneLoadFromFile,SIGNAL(clicked()),this,SLOT(onBtnLoadFromFile_clicked()));
	connect(btnPhoneSaveToFile,SIGNAL(clicked()),this,SLOT(onBtnSaveToFile_clicked()));
	connect(btnConnect,SIGNAL(clicked()),this,SLOT(onBtnConnect_clicked()));
	connect(btnSendParameters,SIGNAL(clicked()),this,SLOT(onBtnSendParams_clicked()));
	connect(btnResetObserver,SIGNAL(clicked()),this,SLOT(onBtnResetObserver_clicked()));
	connect(btnSyncTime,SIGNAL(clicked()),this,SLOT(onBtnSyncTime_clicked()));
	connect(btnRequestLogFile,SIGNAL(clicked()),this,SLOT(onBtnRequestLogFile_clicked()));
	connect(btnSendMuCntl,SIGNAL(clicked()),this,SLOT(onBtnSendMuCntl_clicked()));
	connect(btnClearLog,SIGNAL(clicked()),this,SLOT(onBtnClearLog_clicked()));
	connect(chkUseMuCntl,SIGNAL(clicked()),this,SLOT(onChkUseMuCntl_clicked()));
	connect(chkViewBinarizedImage,SIGNAL(clicked()),this,SLOT(onChkViewBinarizedImage_clicked()));
	connect(chkUseIbvsController,SIGNAL(clicked()),this,SLOT(onChkUseIbvsController_clicked()));
	connect(btnResetDesImgMoment,SIGNAL(clicked()),this,SLOT(onBtnResetDesImgMoment_clicked()));
	connect(btnConfirmDesImgMoment,SIGNAL(clicked()),this,SLOT(onBtnConfirmDesImgMoment_clicked()));
	connect(btnSetYawZero,SIGNAL(clicked()),this,SLOT(onBtnSetYawZero_clicked()));

	mSocketTCP = Socket::ptr(Socket::createTCPSocket());
	mSocketTCP = NULL;
//	mSocketUDP = Socket::ptr(Socket::createUDPSocket());
	mSocketUDP = NULL;

	mLastImage.create(240,320,CV_8UC1);
	mLastImage = cv::Scalar(0);

	mImgViewType = 0;

	mIbvsGainImg.resize(3); mIbvsGainImg[0] = mIbvsGainImg[1] = mIbvsGainImg[2] = 0;
	mIbvsGainFlow.resize(3); mIbvsGainFlow[0] = mIbvsGainFlow[1] = mIbvsGainFlow[2] = 0;
	mIbvsGainFlowInt.resize(3); mIbvsGainFlowInt[0] = mIbvsGainFlowInt[1] = mIbvsGainFlowInt[2] = 0;
	mIbvsGainFF.resize(3); mIbvsGainFF[0] = mIbvsGainFF[1] = mIbvsGainFF[2] = 0;
	mAttCmdOffset.resize(3); mAttCmdOffset[0] = mAttCmdOffset[1] = mAttCmdOffset[2] = 0;	
	mIbvsGainAngularRate.resize(3); mIbvsGainAngularRate[0] = mIbvsGainAngularRate[1] = mIbvsGainAngularRate[2] = 1;
	mIbvsGainAngle = 1;
	mIbvsGainDynamic = 0.1;

	onBtnResetDesImgMoment_clicked();

	populateConfigTree();
}

void PhoneInterface::pollUDP()
{
	if(mSocketUDP == NULL)
		return;

	// need blocking on to avoid conflicts with sending data ... I think
	mMutex_socketUDP.lock();
	mSocketUDP->setBlocking(true);
	while(mSocketUDP != NULL && mSocketUDP->pollRead(0))
	{
		int size=256;
		Packet pck;
		if(receivePacket(mSocketUDP, pck, size))
		{
			if(pck.time >= mTimeMS)
				mTimeMS = pck.time;
			else if(abs((int)(pck.time - mTimeMS)) > 1000)
			{
				cout << "Old packet so assume this is a new starting point. dt = " << pck.time-mTimeMS << endl;
				mTimeMS = pck.time;
			}
			else
			{
				cout << "Ignoring old packet" <<endl;
				continue;
			}

			if(pck.time < mTimeMS)
				cout << "Why am I here?" << endl;

			mMutex_data.lock();
			try
			{
				switch(pck.type)
				{
					case COMM_ARDUINO_STATUS:
						mArduinoStatus = pck.dataInt32[0];
						break;
					case COMM_USE_MOTORS:
						mUseMotors = pck.dataBool[0];
						break;
					case COMM_STATE_PHONE:
						for(int i=0; i<6; i++)
							mState[i][0] = pck.dataFloat[i];
						break;
					case COMM_DESIRED_STATE:
						for(int i=0; i<6; i++)
							mDesiredState[i][0] = pck.dataFloat[i];
						break;
					case COMM_IMAGE_STATE:
						{
							double an = pck.dataFloat[2];
							mStateImage[0] = pck.dataFloat[0]/an;
							mStateImage[1] = pck.dataFloat[1]/an;
							mStateImage[2] = mDesiredStateImage[2]/pow(an/mDesiredHeight,2);
							for(int i=3; i<6; i++)
								mStateImage[i] = pck.dataFloat[i];
						}
						break;
					case COMM_DESIRED_IMAGE_STATE:
						for(int i=3; i<6; i++)
							mDesiredStateImage[i] = pck.dataFloat[i];
						break;
					case COMM_GYRO:
						for(int i=0; i<3; i++)
							mGyro[i][0] = pck.dataFloat[i];
						break;
					case COMM_ACCEL:
						for(int i=0; i<3; i++)
							mAccel[i][0] = pck.dataFloat[i];
						break;
					case COMM_OBSV_BIAS:
						for(int i=0; i<3; i++)
							mBias[i][0] = pck.dataFloat[i];
						break;
					case COMM_MAGNOMETER:
						for(int i=0; i<3; i++)
							mComp[i][0] = pck.dataFloat[i];
						break;
					case COMM_MOTOR_VAL:
						for(int i=0; i<4; i++)
							mMotorValues[i] = pck.dataInt32[i];
						break;
					case COMM_MOTOR_VAL_IBVS:
						for(int i=0; i<4; i++)
							mMotorValuesIbvs[i] = pck.dataInt32[i];
						break;
					case COMM_INT_MEM:
						for(int i=0; i<3; i++)
							mIntMemory[i][0] = pck.dataFloat[i];
						break;
					case COMM_CNTL_TYPE:
						mCurCntlType = pck.dataInt32[0];
						break;
					case COMM_CNTL_CALC_TIME:
						mCntlCalcTimeUS = pck.dataInt32[0];
						break;
					case COMM_IMGPROC_TIME:
						mImgProcTimeUS = pck.dataInt32[0];
						break;
					case COMM_USE_IBVS:
						mUseIbvs = pck.dataBool[0];
						chkUseIbvsController->setChecked(mUseIbvs);
						break;
					default:
						cout << "Unknown phone code: " << pck.type << endl;
				}
			}
			catch (...)
			{ cout << "grr " << pck.type << endl;}

			mMutex_data.unlock();
		}
		mMutex_socketUDP.unlock();
	}
}

void PhoneInterface::pollTCP()
{
	if(mSocketTCP == NULL || !mSocketTCP->connected())
		return;

	// need blocking on to avoid conflicts with sending data ... I think
	mMutex_socketTCP.lock();
	mSocketTCP->setBlocking(true);
	while(mSocketTCP != NULL && mSocketTCP->pollRead(0) && mSocketTCP->connected())
	{
		int code;
		if( receiveTCP(mSocketTCP,(tbyte*)&code, sizeof(code)) == sizeof(code))
		{
			try
			{
				switch(code)
				{
					case 1000:
						receiveLogFile(mSocketTCP,"../phoneLog.txt");
						break;
					case 2000:
						{
							uint32 numRows, numCols, numChannels, type, size;
							receiveTCP(mSocketTCP,(tbyte*)&numRows, sizeof(numRows));
							receiveTCP(mSocketTCP,(tbyte*)&numCols, sizeof(numCols));
							receiveTCP(mSocketTCP,(tbyte*)&numChannels, sizeof(numChannels));
							receiveTCP(mSocketTCP,(tbyte*)&type, sizeof(type));
							receiveTCP(mSocketTCP,(tbyte*)&size, sizeof(size));
//							cout << "Image info: " << numRows << " x "  << numCols << " x " << numChannels << " x " << type << " x " << size << endl;
							vector<uchar> imgData;
							imgData.resize(size);
							receiveTCP(mSocketTCP,(tbyte*)&(imgData.front()),size);
							mMutex_image.lock();
								mLastImage = cv::imdecode(imgData,CV_LOAD_IMAGE_COLOR);
								if(chkViewBinarizedImage->isChecked() == false && chkUseIbvsController->isChecked() == false)
									cv::cvtColor(mLastImage,mLastImage,CV_HSV2BGR);
							mMutex_image.unlock();
						}
						break;
					default:
						cout << "Unknown phone code: " << code << endl;
				}
			}
			catch (...)
			{ cout << "grr " << code << endl;}
		}
		else
		{
			cout << "Closing sockets (1)" << endl;
			mSocketTCP->close();
			mSocketTCP = NULL;
			mMutex_socketUDP.lock();
			mSocketUDP->close();
			mSocketUDP = NULL;
			mMutex_socketUDP.unlock();
		}
		mMutex_socketTCP.unlock();
	}
}

void PhoneInterface::updateDisplay()
{
//	cout << "PhoneInterface::updateDisplay start" << endl;
	if(mSocketTCP != NULL)
		lblPhoneStatus->setText("Connected");
	else
		lblPhoneStatus->setText("Disconnected");

	if(mSocketTCP != NULL)
	{
		pollUDP();
		pollTCP();
		mMutex_data.lock();
		if(mArduinoStatus == 0) lblPhoneArduinoStatus->setText("NULL");
		else if(mArduinoStatus == 1) lblPhoneArduinoStatus->setText("Connected");
		else lblPhoneArduinoStatus->setText("Unknown");

		if(mUseMotors)
			lblPhoneUseMotors->setText("On");
		else
			lblPhoneUseMotors->setText("Off");

		if(mUseIbvs)
			lblPhoneUseIbvs->setText("True");
		else
			lblPhoneUseIbvs->setText("False");

		lblPhoneTime->setText(QString::number(mTimeMS/1000.0+0.5,'f',0));

		lblPhoneCommand0->setText(QString::number(mMotorValues[0]));
		lblPhoneCommand1->setText(QString::number(mMotorValues[1]));
		lblPhoneCommand2->setText(QString::number(mMotorValues[2]));
		lblPhoneCommand3->setText(QString::number(mMotorValues[3]));

		lblPhoneIbvsCommand0->setText(QString::number(mMotorValuesIbvs[0]));
		lblPhoneIbvsCommand1->setText(QString::number(mMotorValuesIbvs[1]));
		lblPhoneIbvsCommand2->setText(QString::number(mMotorValuesIbvs[2]));
		lblPhoneIbvsCommand3->setText(QString::number(mMotorValuesIbvs[3]));

		switch(mCurCntlType)
		{
			case CNTL_TRANSLATION_PID:
				lblController->setText("PID");
				chkUseMuCntl->setChecked(false);
				break;
			case CNTL_TRANSLATION_SYS:
				lblController->setText("sys");
				chkUseMuCntl->setChecked(true);
				break;
			default:
				lblController->setText("Unknown");
		}
		lblCntlCalcTime->setText(QString::number(mCntlCalcTimeUS/1.0e3,'f',2)+"ms");

		lblImgProcTime->setText(QString::number(mImgProcTimeUS/1.0e3,'f',0)+"ms");
		
		for(int i=0; i<3; i++)
		{
			tblPosition->item(i,1)->setText(QString::number(mState[i][0]*RAD2DEG,'f',1)+QChar(0x00B0));
			tblPosition->item(i,0)->setText(QString::number(mDesiredState[i][0]*RAD2DEG,'f',1)+QChar(0x00B0));
		}
		for(int i=3; i<6; i++)
		{
			tblPosition->item(i,1)->setText(QString::number(mState[i][0]*RAD2DEG,'f',1)+QChar(0x00B0)+"/s");
			tblPosition->item(i,0)->setText(QString::number(mDesiredState[i][0]*RAD2DEG,'f',1)+QChar(0x00B0)+"/s");
			tblPosition->item(i,2)->setText(QString::number(mIntMemory[i-3][0]*RAD2DEG,'f',1)+QChar(0x00B0));
		}

		for(int i=0; i<6; i++)
			tblImgState->item(0,i)->setText(QString::number(mStateImage[i],'f',3));
		for(int i=3; i<6; i++)
			tblImgState->item(1,i)->setText(QString::number(mDesiredStateImage[i],'f',3));
		tblImgState->item(1,2)->setText(QString::number(mDesiredStateImage[2],'f',0));

		for(int i=0; i<3; i++)
			tblMeasures->item(i,0)->setText(QString::number(mGyro[i][0]*RAD2DEG,'f',1)+QChar(0x00B0)+"/s");
		for(int i=0; i<3; i++)
			tblMeasures->item(i,1)->setText(QString::number(mBias[i][0]*RAD2DEG,'f',2)+QChar(0x00B0)+"/s");
		for(int i=0; i<3; i++)
			tblMeasures->item(i,2)->setText(QString::number(mAccel[i][0],'f',1)+"m/s/s");
		for(int i=0; i<3; i++)
			tblMeasures->item(i,3)->setText(QString::number(mComp[i][0],'f',1)+"uT");
	}
	else
	{
		lblPhoneArduinoStatus->setText("N/A");
		lblPhoneUseMotors->setText("N/A");
		lblPhoneUseIbvs->setText("N/A");
		lblPhoneTime->setText("N/A");
		lblPhoneCommand0->setText("N/A");
		lblPhoneCommand1->setText("N/A");
		lblPhoneCommand2->setText("N/A");
		lblPhoneCommand3->setText("N/A");
		lblPhoneIbvsCommand0->setText("N/A");
		lblPhoneIbvsCommand1->setText("N/A");
		lblPhoneIbvsCommand2->setText("N/A");
		lblPhoneIbvsCommand3->setText("N/A");
		lblController->setText("N/A");

		tblImgState->item(1,2)->setText(QString::number(mDesiredStateImage[2],'f',0));
	}
	mMutex_data.unlock();

	mMutex_image.lock();
		lblImageDisplay->setPixmap(QPixmap::fromImage(cvMat2QImage(mLastImage)));
	mMutex_image.unlock();
}

QImage PhoneInterface::cvMat2QImage(const cv::Mat &mat)
{
	int height = mat.rows;
	int width = mat.cols;

	if(mat.depth() == CV_8U && mat.channels() == 3)
	{
		const uchar *qImageBuffer = (const uchar*)mat.data;
		QImage img(qImageBuffer, width, height, QImage::Format_RGB888);
		return img.rgbSwapped();
	}
	else if(mat.depth() == CV_8U && mat.channels() == 1)
	{
		const uchar *qImageBuffer = (const uchar*)mat.data;
		QImage img(qImageBuffer, width, height, QImage::Format_Indexed8);

		QVector<QRgb> colorTable;
		for(int i=0; i<256; i++)
			colorTable.push_back(qRgb(i,i,i));
		img.setColorTable(colorTable);
		return img;
	}
	else
	{
		qWarning() << "Image cannot be converted" << endl;
		return QImage();
	}
}

bool PhoneInterface::sendUDP(tbyte* data, int size)
{
	if(mSocketUDP == NULL)
		return false;

//	mMutex_socketUDP.lock();
	try
	{
		mSocketUDP->sendTo(data,size,toadlet::egg::net::Socket::stringToIP(String(mIP.c_str())),mPort);
	}
	catch (...)
	{
		cout << "Send failure" << endl;
		mSocketUDP->close();
		mSocketUDP = NULL;
		mIsConnected = false;
		return false;
	}
//	mMutex_socketUDP.unlock();

	return true;
}

bool PhoneInterface::sendTCP(tbyte* data, int size)
{
	if(mSocketTCP == NULL || mSocketTCP->connected() == false)
		return false;

//	mMutex_socketTCP.lock();
	try
	{
		mSocketTCP->send(data,size);
	}
	catch (...)
	{
		cout << "Send failure" << endl;
		mSocketTCP->close();
		mSocketTCP = Socket::ptr(Socket::createTCPSocket());
		mIsConnected = false;
		return false;
	}
//	mMutex_socketTCP.unlock();

	return true;
}

bool PhoneInterface::sendMotorStop()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send motor start because I'm currently not connected to the phone.");
		box.exec();
		return false;
	}

	cout << "Sending motor stop ... ";
	int code = COMM_MOTOR_OFF;
	int result = sendTCP((tbyte*)&code,sizeof(code));
	cout << "done." << endl;
	
	return result;
}

bool PhoneInterface::sendMotorStart()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send motor start because I'm currently not connected to the phone.");
		box.exec();
		return false;
	}

	sendParams();

	int code = COMM_MOTOR_ON;
	return sendTCP((tbyte*)&code,sizeof(code));
}

bool PhoneInterface::sendParams()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send phone params because I'm currently not connected to the phone.");
		box.exec();
		return false;
	}

	mMutex_data.lock();
		int code = COMM_CNTL_GAIN_PID;
		float rollPID[3], pitchPID[3], yawPID[3];
		rollPID[0] = mGainP[0];
		rollPID[1] = mGainI[0];
		rollPID[2] = mGainD[0];
		pitchPID[0] = mGainP[1];
		pitchPID[1] = mGainI[1];
		pitchPID[2] = mGainD[1];
		yawPID[0] = mGainP[2];
		yawPID[1] = mGainI[2];
		yawPID[2] = mGainD[2];

		bool result = true;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)rollPID,3*sizeof(float));
		if(result) result = result && sendTCP((tbyte*)pitchPID,3*sizeof(float));
		if(result) result = result && sendTCP((tbyte*)yawPID,3*sizeof(float));

		code = COMM_CNTL_SYS_GAINS;
		int size = mGainCntlSys.size();
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&size,sizeof(size));
		if(result) result = result && sendTCP((tbyte*)&(mGainCntlSys[0]),size*sizeof(float));

		code = COMM_MOTOR_TRIM;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)mMotorTrim, 4*sizeof(int));

		code = COMM_OBSV_GAIN;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result =result && sendTCP((tbyte*)&mObserverGainP,sizeof(mObserverGainP));
		if(result) result =result && sendTCP((tbyte*)&mObserverGainI,sizeof(mObserverGainI));
		if(result) result =result && sendTCP((tbyte*)&(mObserverWeights[0]), sizeof(mObserverWeights[0]));
		if(result) result =result && sendTCP((tbyte*)&(mObserverWeights[1]), sizeof(mObserverWeights[1]));

		int cnt =  mFiltBoxColorMin.size();
		Collection<int> mins(cnt), maxs(cnt);
		for(int i=0; i<cnt; i++)
		{
			mins[i] = mFiltBoxColorMin[i];
			maxs[i] = mFiltBoxColorMax[i];
		}
		code = COMM_IMGPROC_BOX_COLOR_MIN; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&cnt,sizeof(cnt));
		if(result) result = result && sendTCP((tbyte*)&(mins[0]),cnt*sizeof(int));
		code = COMM_IMGPROC_BOX_COLOR_MAX; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&cnt,sizeof(cnt));
		if(result) result = result && sendTCP((tbyte*)&(maxs[0]),cnt*sizeof(int));
		code = COMM_IMGPROC_VAL_MIN; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltValMin,sizeof(mFiltValMin));
		code = COMM_IMGPROC_VAL_MAX; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltValMax,sizeof(mFiltValMax));
		code = COMM_IMGPROC_SAT_MIN; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltSatMin,sizeof(mFiltSatMin));
		code = COMM_IMGPROC_SAT_MAX; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltSatMax,sizeof(mFiltSatMax));
		code = COMM_IMGPROC_CIRC_MIN; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltCircMin,sizeof(mFiltCircMin));
		code = COMM_IMGPROC_CIRC_MAX; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltCircMax,sizeof(mFiltCircMax));
		code = COMM_IMGPROC_CONV_MIN; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltConvMin,sizeof(mFiltConvMin));
		code = COMM_IMGPROC_CONV_MAX; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltConvMax,sizeof(mFiltConvMax));
		code = COMM_IMGPROC_AREA_MIN; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltAreaMin,sizeof(mFiltAreaMin));
		code = COMM_IMGPROC_AREA_MAX; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mFiltAreaMax,sizeof(mFiltAreaMax));

		code = COMM_LOG_MASK; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mLogMask, sizeof(mLogMask));

//		float dt = mDeltaT;
//		code = COMM_DELTA_T; 
//		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
//		if(result) result = result && sendTCP((tbyte*)&dt, sizeof(dt));

		float scale = mForceScaling;
		code = COMM_MOTOR_FORCE_SCALING; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&scale,sizeof(scale));

		float torqueScale = mTorqueScaling;
		code = COMM_MOTOR_TORQUE_SCALING;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&torqueScale,sizeof(torqueScale));

		code = COMM_KALMANFILTER_POS_MEAS_STD;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mKfPosMeasStdDev,sizeof(mKfPosMeasStdDev));

		code = COMM_KALMANFILTER_VEL_MEAS_STD;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&mKfVelMeasStdDev,sizeof(mKfVelMeasStdDev));

		float m = mMass;
		code = COMM_MASS; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&m,sizeof(m));

		int gainSize = mIbvsGainImg.size()+mIbvsGainFlowInt.size()+mIbvsGainFlow.size()+mIbvsGainFF.size()+mAttCmdOffset.size()+mIbvsGainAngularRate.size()+1+1;
		Collection<float> gains(gainSize);
		for(int i=0; i<3; i++)
		{
			gains[i] = mIbvsGainImg[i];
			gains[i+3] = mIbvsGainFlow[i];
			gains[i+6] = mIbvsGainFlowInt[i];
			gains[i+9] = mIbvsGainFF[i];
			gains[i+12] = mAttCmdOffset[i];
			gains[i+15] = mIbvsGainAngularRate[i];
			
		}
		gains[18] = mIbvsGainAngle;
		gains[19] = mIbvsGainDynamic;
		code = COMM_IBVS_GAINS; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&gainSize,sizeof(gainSize));
		if(result) result = result && sendTCP((tbyte*)&(gains[0]),gainSize*sizeof(float));

		Collection<float> attGains(4);
		int attGainSize = attGains.size();
		for(int i=0; i<3; i++)
			attGains[i] = mIbvsGainAngularRate[i];
		attGains[3] = mIbvsGainAngle;
		code = COMM_ATT_GAINS;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&attGainSize,sizeof(attGainSize));
		if(result) result = result && sendTCP((tbyte*)&(attGains[0]),attGainSize*sizeof(float));

		float rollBias = mAttBias[0][0];
		float pitchBias = mAttBias[1][0];
		float yawBias = mAttBias[2][0];
		code = COMM_ATT_BIAS; 
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&rollBias,sizeof(rollBias));
		if(result) result = result && sendTCP((tbyte*)&pitchBias,sizeof(pitchBias));
		if(result) result = result && sendTCP((tbyte*)&yawBias,sizeof(yawBias));

		float attBiasGain = mAttBiasGain;
		code = COMM_ATT_BIAS_GAIN;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&attBiasGain,sizeof(attBiasGain));

		float forceScalingGain = mForceScalingGain;
		code = COMM_FORCE_SCALING_GAIN;
		if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
		if(result) result = result && sendTCP((tbyte*)&forceScalingGain, sizeof(forceScalingGain));

		if(result)
			cout << "Phone params sent." << endl;
		else
			cout << "Error sending phone params." << endl;
	mMutex_data.unlock();
	return result;
}

bool PhoneInterface::loadConfigFromFile(string filename)
{
	cout << "Loading config from " << filename << endl;
	FILE *fp = fopen((filename).c_str(),"r");
	if(fp == NULL)
	{
		populateConfigTree(); // with whatever is in there right now
		return false;
	}

	mxml_node_t *xmlRoot;
	xmlRoot = mxmlLoadFile(NULL,fp,MXML_TEXT_CALLBACK);
	fclose(fp);

	if(xmlRoot == NULL)
	{
		populateConfigTree(); // with whatever is in there right now
		return false;
	}

	mMutex_data.lock();
		mxml_node_t *commRoot = mxmlFindElement(xmlRoot,xmlRoot,"Communication",NULL,NULL,MXML_DESCEND);
		if(commRoot != NULL)
		{
			mxml_node_t *ipNode = mxmlFindElement(commRoot,commRoot,"IP",NULL,NULL,MXML_DESCEND);
			mxml_node_t *portNode = mxmlFindElement(commRoot,commRoot,"Port",NULL,NULL,MXML_DESCEND);
			if(ipNode != NULL)	mIP = ipNode->child->value.text.string;
			if(portNode != NULL)	mPort = QString(portNode->child->value.text.string).toInt();
		}
		mxml_node_t *cntlRoot = mxmlFindElement(xmlRoot,xmlRoot,"Controller",NULL,NULL,MXML_DESCEND);
		if(cntlRoot != NULL)
		{
			mxml_node_t *rollNode = mxmlFindElement(cntlRoot,cntlRoot,"Roll",NULL,NULL,MXML_DESCEND);
			mxml_node_t *pNode = mxmlFindElement(rollNode,rollNode,"P",NULL,NULL,MXML_DESCEND);
			mxml_node_t *iNode = mxmlFindElement(rollNode,rollNode,"I",NULL,NULL,MXML_DESCEND);
			mxml_node_t *dNode = mxmlFindElement(rollNode,rollNode,"D",NULL,NULL,MXML_DESCEND);
			if(pNode != NULL) mGainP[0] = QString(pNode->child->value.text.string).toDouble();
			if(iNode != NULL) mGainI[0] = QString(iNode->child->value.text.string).toDouble();
			if(dNode != NULL) mGainD[0] = QString(dNode->child->value.text.string).toDouble();
			mxml_node_t *pitchNode = mxmlFindElement(cntlRoot,cntlRoot,"Pitch",NULL,NULL,MXML_DESCEND);
			pNode = mxmlFindElement(pitchNode,pitchNode,"P",NULL,NULL,MXML_DESCEND);
			iNode = mxmlFindElement(pitchNode,pitchNode,"I",NULL,NULL,MXML_DESCEND);
			dNode = mxmlFindElement(pitchNode,pitchNode,"D",NULL,NULL,MXML_DESCEND);
			if(pNode != NULL) mGainP[1] = QString(pNode->child->value.text.string).toDouble();
			if(iNode != NULL) mGainI[1] = QString(iNode->child->value.text.string).toDouble();
			if(dNode != NULL) mGainD[1] = QString(dNode->child->value.text.string).toDouble();
			mxml_node_t *yawNode = mxmlFindElement(cntlRoot,cntlRoot,"Yaw",NULL,NULL,MXML_DESCEND);
			pNode = mxmlFindElement(yawNode,yawNode,"P",NULL,NULL,MXML_DESCEND);
			iNode = mxmlFindElement(yawNode,yawNode,"I",NULL,NULL,MXML_DESCEND);
			dNode = mxmlFindElement(yawNode,yawNode,"D",NULL,NULL,MXML_DESCEND);
			if(pNode != NULL) mGainP[2] = QString(pNode->child->value.text.string).toDouble();
			if(iNode != NULL) mGainI[2] = QString(iNode->child->value.text.string).toDouble();
			if(dNode != NULL) mGainD[2] = QString(dNode->child->value.text.string).toDouble();
		}
		mxml_node_t *muCntlNode= mxmlFindElement(xmlRoot,xmlRoot,"cntlSys",NULL,NULL,MXML_DESCEND);
		if(muCntlNode!= NULL)
		{
			mxml_node_t *sysFileNode = mxmlFindElement(muCntlNode,muCntlNode,"sysFile",NULL,NULL,MXML_DESCEND);
			mxml_node_t *gainXNode = mxmlFindElement(muCntlNode,muCntlNode,"gainX",NULL,NULL,MXML_DESCEND);
			mxml_node_t *gainYNode = mxmlFindElement(muCntlNode,muCntlNode,"gainY",NULL,NULL,MXML_DESCEND);
			mxml_node_t *gainZNode = mxmlFindElement(muCntlNode,muCntlNode,"gainZ",NULL,NULL,MXML_DESCEND);
			mxml_node_t *gainXDotNode = mxmlFindElement(muCntlNode,muCntlNode,"gainXDot",NULL,NULL,MXML_DESCEND);
			mxml_node_t *gainYDotNode = mxmlFindElement(muCntlNode,muCntlNode,"gainYDot",NULL,NULL,MXML_DESCEND);
			mxml_node_t *gainZDotNode = mxmlFindElement(muCntlNode,muCntlNode,"gainZDot",NULL,NULL,MXML_DESCEND);

			if(sysFileNode != NULL) mCntlSysFile = sysFileNode->child->value.text.string;
			if(gainXNode != NULL) mGainCntlSys[0] = QString(gainXNode->child->value.text.string).toDouble();
			if(gainYNode != NULL) mGainCntlSys[1] = QString(gainYNode->child->value.text.string).toDouble();
			if(gainZNode != NULL) mGainCntlSys[2] = QString(gainZNode->child->value.text.string).toDouble();
			if(gainXDotNode != NULL) mGainCntlSys[3] = QString(gainXDotNode->child->value.text.string).toDouble();
			if(gainYDotNode != NULL) mGainCntlSys[4] = QString(gainYDotNode->child->value.text.string).toDouble();
			if(gainZDotNode != NULL) mGainCntlSys[5] = QString(gainZDotNode->child->value.text.string).toDouble();
		}
		mxml_node_t *motorRoot = mxmlFindElement(xmlRoot,xmlRoot,"Motors",NULL,NULL,MXML_DESCEND);
		if(motorRoot != NULL)
		{
			mxml_node_t *motorTrimRoot = mxmlFindElement(motorRoot,motorRoot,"Trim",NULL,NULL,MXML_DESCEND);
			mxml_node_t *nNode = mxmlFindElement(motorTrimRoot,motorTrimRoot,"N",NULL,NULL,MXML_DESCEND);
			mxml_node_t *eNode = mxmlFindElement(motorTrimRoot,motorTrimRoot,"E",NULL,NULL,MXML_DESCEND);
			mxml_node_t *sNode = mxmlFindElement(motorTrimRoot,motorTrimRoot,"S",NULL,NULL,MXML_DESCEND);
			mxml_node_t *wNode = mxmlFindElement(motorTrimRoot,motorTrimRoot,"W",NULL,NULL,MXML_DESCEND);
			mMotorTrim[0] = QString(nNode->child->value.text.string).toInt();
			mMotorTrim[1] = QString(eNode->child->value.text.string).toInt();
			mMotorTrim[2] = QString(sNode->child->value.text.string).toInt();
			mMotorTrim[3] = QString(wNode->child->value.text.string).toInt();
		}
		mxml_node_t *observerRoot = mxmlFindElement(xmlRoot,xmlRoot,"Observer",NULL,NULL,MXML_DESCEND);
		if(observerRoot != NULL)
		{
			mxml_node_t *kpNode = mxmlFindElement(observerRoot,observerRoot,"Kp",NULL,NULL,MXML_DESCEND);
			mxml_node_t *kiNode = mxmlFindElement(observerRoot,observerRoot,"Ki",NULL,NULL,MXML_DESCEND);
			mxml_node_t *accelWeightNode = mxmlFindElement(observerRoot,observerRoot,"accelWeight",NULL,NULL,MXML_DESCEND);
			mxml_node_t *magWeightNode = mxmlFindElement(observerRoot,observerRoot,"magWeight",NULL,NULL,MXML_DESCEND);
	//		mxml_node_t *gravBWNode = mxmlFindElement(observerRoot,observerRoot,"gravBandwidth",NULL,NULL,MXML_DESCEND);

			if(kpNode != NULL)	mObserverGainP = QString(kpNode->child->value.text.string).toDouble();
			if(kiNode != NULL)	mObserverGainI = QString(kiNode->child->value.text.string).toDouble();
			if(accelWeightNode != NULL)	mObserverWeights[0] = QString(accelWeightNode->child->value.text.string).toDouble();
			if(magWeightNode != NULL)	mObserverWeights[1] = QString(magWeightNode->child->value.text.string).toDouble();
	//		if(gravBWNode != NULL)	mGravBandwidth = QString(gravBWNode->child->value.text.string).toDouble();
		}

		mxml_node_t *imageRoot = mxmlFindElement(xmlRoot,xmlRoot,"ImgProc",NULL,NULL,MXML_DESCEND);
		if(imageRoot != NULL)
		{
			mxml_node_t *box0MinNode = mxmlFindElement(imageRoot,imageRoot,"Box0Min",NULL,NULL,MXML_DESCEND);
			mxml_node_t *box0MaxNode = mxmlFindElement(imageRoot,imageRoot,"Box0Max",NULL,NULL,MXML_DESCEND);
			mxml_node_t *box1MinNode = mxmlFindElement(imageRoot,imageRoot,"Box1Min",NULL,NULL,MXML_DESCEND);
			mxml_node_t *box1MaxNode = mxmlFindElement(imageRoot,imageRoot,"Box1Max",NULL,NULL,MXML_DESCEND);
			mxml_node_t *box2MinNode = mxmlFindElement(imageRoot,imageRoot,"Box2Min",NULL,NULL,MXML_DESCEND);
			mxml_node_t *box2MaxNode = mxmlFindElement(imageRoot,imageRoot,"Box2Max",NULL,NULL,MXML_DESCEND);
			mxml_node_t *box3MinNode = mxmlFindElement(imageRoot,imageRoot,"Box3Min",NULL,NULL,MXML_DESCEND);
			mxml_node_t *box3MaxNode = mxmlFindElement(imageRoot,imageRoot,"Box3Max",NULL,NULL,MXML_DESCEND);
			mxml_node_t *satMinNode = mxmlFindElement(imageRoot,imageRoot,"SatMin",NULL,NULL,MXML_DESCEND);
			mxml_node_t *satMaxNode = mxmlFindElement(imageRoot,imageRoot,"SatMax",NULL,NULL,MXML_DESCEND);
			mxml_node_t *valMinNode = mxmlFindElement(imageRoot,imageRoot,"ValMin",NULL,NULL,MXML_DESCEND);
			mxml_node_t *valMaxNode = mxmlFindElement(imageRoot,imageRoot,"ValMax",NULL,NULL,MXML_DESCEND);
			mxml_node_t *circMinNode = mxmlFindElement(imageRoot,imageRoot,"CircMin",NULL,NULL,MXML_DESCEND);
			mxml_node_t *circMaxNode = mxmlFindElement(imageRoot,imageRoot,"CircMax",NULL,NULL,MXML_DESCEND);
			mxml_node_t *convMinNode = mxmlFindElement(imageRoot,imageRoot,"ConvMin",NULL,NULL,MXML_DESCEND);
			mxml_node_t *convMaxNode = mxmlFindElement(imageRoot,imageRoot,"ConvMax",NULL,NULL,MXML_DESCEND);
			mxml_node_t *areaMinNode = mxmlFindElement(imageRoot,imageRoot,"AreaMin",NULL,NULL,MXML_DESCEND);
			mxml_node_t *areaMaxNode = mxmlFindElement(imageRoot,imageRoot,"AreaMax",NULL,NULL,MXML_DESCEND);

			if(box0MinNode != NULL) mFiltBoxColorMin[0] = QString(box0MinNode->child->value.text.string).toInt();
			if(box0MaxNode != NULL) mFiltBoxColorMax[0] = QString(box0MaxNode->child->value.text.string).toInt();
			if(box1MinNode != NULL) mFiltBoxColorMin[1] = QString(box1MinNode->child->value.text.string).toInt();
			if(box1MaxNode != NULL) mFiltBoxColorMax[1] = QString(box1MaxNode->child->value.text.string).toInt();
			if(box2MinNode != NULL) mFiltBoxColorMin[2] = QString(box2MinNode->child->value.text.string).toInt();
			if(box2MaxNode != NULL) mFiltBoxColorMax[2] = QString(box2MaxNode->child->value.text.string).toInt();
			if(box3MinNode != NULL) mFiltBoxColorMin[3] = QString(box3MinNode->child->value.text.string).toInt();
			if(box3MaxNode != NULL) mFiltBoxColorMax[3] = QString(box3MaxNode->child->value.text.string).toInt();
			if(satMinNode != NULL) mFiltSatMin = QString(satMinNode->child->value.text.string).toInt();
			if(satMaxNode != NULL) mFiltSatMax = QString(satMaxNode->child->value.text.string).toInt();
			if(valMinNode != NULL) mFiltValMin = QString(valMinNode->child->value.text.string).toInt();
			if(valMaxNode != NULL) mFiltValMax = QString(valMaxNode->child->value.text.string).toInt();
			if(circMinNode != NULL) mFiltCircMin = QString(circMinNode->child->value.text.string).toInt();
			if(circMaxNode != NULL) mFiltCircMax = QString(circMaxNode->child->value.text.string).toInt();
			if(convMinNode != NULL) mFiltConvMin = QString(convMinNode->child->value.text.string).toInt();
			if(convMaxNode != NULL) mFiltConvMax = QString(convMaxNode->child->value.text.string).toInt();
			if(areaMinNode != NULL) mFiltAreaMin = QString(areaMinNode->child->value.text.string).toInt();
			if(areaMaxNode != NULL) mFiltAreaMax = QString(areaMaxNode->child->value.text.string).toInt();
		}

		mxml_node_t *ibvsRoot = mxmlFindElement(xmlRoot,xmlRoot,"IBVS",NULL,NULL,MXML_DESCEND);
		if(ibvsRoot != NULL)
		{
			mxml_node_t *xNode = 	mxmlFindElement(ibvsRoot,ibvsRoot,"imgX",NULL,NULL,MXML_DESCEND);
			mxml_node_t *yNode = 	mxmlFindElement(ibvsRoot,ibvsRoot,"imgY",NULL,NULL,MXML_DESCEND);
			mxml_node_t *areaNode = mxmlFindElement(ibvsRoot,ibvsRoot,"imgArea",NULL,NULL,MXML_DESCEND);
			mxml_node_t *xFlowNode = mxmlFindElement(ibvsRoot,ibvsRoot,"xFlow",NULL,NULL,MXML_DESCEND);
			mxml_node_t *yFlowNode = mxmlFindElement(ibvsRoot,ibvsRoot,"yFlow",NULL,NULL,MXML_DESCEND);
			mxml_node_t *zFlowNode = mxmlFindElement(ibvsRoot,ibvsRoot,"zFlow",NULL,NULL,MXML_DESCEND);
			mxml_node_t *xFlowIntNode = mxmlFindElement(ibvsRoot,ibvsRoot,"xFlowInt",NULL,NULL,MXML_DESCEND);
			mxml_node_t *yFlowIntNode = mxmlFindElement(ibvsRoot,ibvsRoot,"yFlowInt",NULL,NULL,MXML_DESCEND);
			mxml_node_t *zFlowIntNode = mxmlFindElement(ibvsRoot,ibvsRoot,"zFlowInt",NULL,NULL,MXML_DESCEND);
			mxml_node_t *xFFNode = mxmlFindElement(ibvsRoot,ibvsRoot,"xFF",NULL,NULL,MXML_DESCEND);
			mxml_node_t *yFFNode = mxmlFindElement(ibvsRoot,ibvsRoot,"yFF",NULL,NULL,MXML_DESCEND);
			mxml_node_t *zFFNode = mxmlFindElement(ibvsRoot,ibvsRoot,"zFF",NULL,NULL,MXML_DESCEND);
			mxml_node_t *rollAttOffsetNode = mxmlFindElement(ibvsRoot,ibvsRoot,"rollAttCmdOffset",NULL,NULL,MXML_DESCEND);
			mxml_node_t *pitchAttOffsetNode = mxmlFindElement(ibvsRoot,ibvsRoot,"pitchAttCmdOffset",NULL,NULL,MXML_DESCEND);
			mxml_node_t *yawAttOffsetNode = mxmlFindElement(ibvsRoot,ibvsRoot,"yawAttCmdOffset",NULL,NULL,MXML_DESCEND);
			mxml_node_t *xOmegaNode = mxmlFindElement(ibvsRoot,ibvsRoot,"xOmega",NULL,NULL,MXML_DESCEND);
			mxml_node_t *yOmegaNode = mxmlFindElement(ibvsRoot,ibvsRoot,"yOmega",NULL,NULL,MXML_DESCEND);
			mxml_node_t *zOmegaNode = mxmlFindElement(ibvsRoot,ibvsRoot,"zOmega",NULL,NULL,MXML_DESCEND);
			mxml_node_t *angleNode = mxmlFindElement(ibvsRoot,ibvsRoot,"angle",NULL,NULL,MXML_DESCEND);
			mxml_node_t *dynamicNode = mxmlFindElement(ibvsRoot,ibvsRoot,"dynamic",NULL,NULL,MXML_DESCEND);

			if(xNode != NULL) 		mIbvsGainImg[0] = QString(xNode->child->value.text.string).toDouble();
			if(yNode != NULL) 		mIbvsGainImg[1] = QString(yNode->child->value.text.string).toDouble();
			if(areaNode != NULL) 	mIbvsGainImg[2] = QString(areaNode->child->value.text.string).toDouble();
			if(xFlowNode != NULL) 	mIbvsGainFlow[0] = QString(xFlowNode->child->value.text.string).toDouble();
			if(yFlowNode != NULL) 	mIbvsGainFlow[1] = QString(yFlowNode->child->value.text.string).toDouble();
			if(zFlowNode != NULL) 	mIbvsGainFlow[2] = QString(zFlowNode->child->value.text.string).toDouble();
			if(xFlowIntNode != NULL) 	mIbvsGainFlowInt[0] = QString(xFlowIntNode->child->value.text.string).toDouble();
			if(yFlowIntNode != NULL) 	mIbvsGainFlowInt[1] = QString(yFlowIntNode->child->value.text.string).toDouble();
			if(zFlowIntNode!= NULL) 	mIbvsGainFlowInt[2] = QString(zFlowIntNode->child->value.text.string).toDouble();
			if(xFFNode != NULL) 	mIbvsGainFF[0] = QString(xFFNode->child->value.text.string).toDouble();
			if(yFFNode != NULL) 	mIbvsGainFF[1] = QString(yFFNode->child->value.text.string).toDouble();
			if(zFFNode != NULL) 	mIbvsGainFF[2] = QString(zFFNode->child->value.text.string).toDouble();
			if(rollAttOffsetNode != NULL) 	mAttCmdOffset[0] = QString(rollAttOffsetNode->child->value.text.string).toDouble();
			if(pitchAttOffsetNode != NULL) 	mAttCmdOffset[1] = QString(pitchAttOffsetNode->child->value.text.string).toDouble();
			if(yawAttOffsetNode!= NULL) 	mAttCmdOffset[2] = QString(yawAttOffsetNode->child->value.text.string).toDouble();
			if(xOmegaNode != NULL) 	mIbvsGainAngularRate[0] = QString(xOmegaNode->child->value.text.string).toDouble();
			if(yOmegaNode != NULL) 	mIbvsGainAngularRate[1] = QString(yOmegaNode->child->value.text.string).toDouble();
			if(zOmegaNode != NULL) 	mIbvsGainAngularRate[2] = QString(zOmegaNode->child->value.text.string).toDouble();
			if(angleNode != NULL) 	mIbvsGainAngle = QString(angleNode->child->value.text.string).toDouble();
			if(dynamicNode != NULL) 	mIbvsGainDynamic = QString(dynamicNode->child->value.text.string).toDouble();
		}

		mxml_node_t *kfRoot = mxmlFindElement(xmlRoot,xmlRoot,"KalmanFilter",NULL,NULL,MXML_DESCEND);
			if(kfRoot != NULL)
			{
				mxml_node_t *attBiasRoot = mxmlFindElement(kfRoot, kfRoot, "attBias", NULL, NULL, MXML_DESCEND);
				if(attBiasRoot != NULL)
				{
					mxml_node_t *rollBiasRoot = mxmlFindElement(attBiasRoot, attBiasRoot, "roll", NULL, NULL, MXML_DESCEND);
					mxml_node_t *pitchBiasRoot = mxmlFindElement(attBiasRoot, attBiasRoot, "pitch", NULL, NULL, MXML_DESCEND);
					mxml_node_t *yawBiasRoot = mxmlFindElement(attBiasRoot, attBiasRoot, "yaw", NULL, NULL, MXML_DESCEND);

					if(rollBiasRoot!=NULL) mAttBias[0][0] = QString(rollBiasRoot->child->value.text.string).toDouble(); else cout << "KF roll bias node not found" << endl;
					if(pitchBiasRoot!=NULL) mAttBias[1][0] = QString(pitchBiasRoot->child->value.text.string).toDouble(); else cout << "KF pitch bias node not found" << endl;
					if(yawBiasRoot!=NULL) mAttBias[2][0] = QString(yawBiasRoot->child->value.text.string).toDouble(); else cout << "KF yaw bias node not found" << endl;
				}
				else
					cout << "KF att bias node not found" << endl;
			
				mxml_node_t *attBiasGainRoot= mxmlFindElement(kfRoot, kfRoot, "attBiasGain", NULL, NULL, MXML_DESCEND);
					if(attBiasGainRoot!=NULL) mAttBiasGain = QString(attBiasGainRoot->child->value.text.string).toDouble(); else cout << "KF att bias gain node not found" << endl;

				mxml_node_t *forceScalingGainRoot= mxmlFindElement(kfRoot, kfRoot, "forceScalingGain", NULL, NULL, MXML_DESCEND);
					if(forceScalingGainRoot!=NULL) mForceScalingGain = QString(forceScalingGainRoot->child->value.text.string).toDouble(); else cout << "KF force scaling gain node not found" << endl;

				mxml_node_t *posMeasStdDevRoot= mxmlFindElement(kfRoot, kfRoot, "posMeasStdDev", NULL, NULL, MXML_DESCEND);
					if(posMeasStdDevRoot!=NULL) mKfPosMeasStdDev = QString(posMeasStdDevRoot->child->value.text.string).toDouble(); else cout << "Pos meas StdDev node not found" << endl;

				mxml_node_t *velMeasStdDevRoot= mxmlFindElement(kfRoot, kfRoot, "velMeasStdDev", NULL, NULL, MXML_DESCEND);
					if(velMeasStdDevRoot!=NULL) mKfVelMeasStdDev = QString(velMeasStdDevRoot->child->value.text.string).toDouble(); else cout << "Vel meas StdDev node not found" << endl;
			}
			else
				cout << "Kalman filter node not found" << endl;

		mxml_node_t *logRoot = mxmlFindElement(xmlRoot,xmlRoot,"Log",NULL,NULL,MXML_DESCEND);
		if(logRoot != NULL)
		{
			mxml_node_t *maskNode = mxmlFindElement(logRoot,logRoot,"Mask",NULL,NULL,mLogMask);
			if(maskNode != NULL) mLogMask = QString(maskNode->child->value.text.string).toInt();
		}
	mMutex_data.unlock();

	populateConfigTree();
	populateImgProcParams();
	return true;
}

void PhoneInterface::saveConfigToFile(string filename)
{
	mMutex_data.lock();
		mxml_node_t *xmlRoot = mxmlNewXML("1.0");

		mxml_node_t *commNode = mxmlNewElement(xmlRoot,"Communication");
			mxmlNewText(mxmlNewElement(commNode,"IP"),0,mIP.c_str());
			mxmlNewInteger(mxmlNewElement(commNode,"Port"),mPort);
		mxml_node_t *phoneCntlNode = mxmlNewElement(xmlRoot,"Controller");
			mxml_node_t *rollNode = mxmlNewElement(phoneCntlNode,"Roll");
				mxmlNewReal(mxmlNewElement(rollNode,"P"),mGainP[0]);
				mxmlNewReal(mxmlNewElement(rollNode,"I"),mGainI[0]);
				mxmlNewReal(mxmlNewElement(rollNode,"D"),mGainD[0]);
			mxml_node_t *pitchNode = mxmlNewElement(phoneCntlNode,"Pitch");
				mxmlNewReal(mxmlNewElement(pitchNode,"P"),mGainP[1]);
				mxmlNewReal(mxmlNewElement(pitchNode,"I"),mGainI[1]);
				mxmlNewReal(mxmlNewElement(pitchNode,"D"),mGainD[1]);
			mxml_node_t *yawNode = mxmlNewElement(phoneCntlNode,"Yaw");
				mxmlNewReal(mxmlNewElement(yawNode,"P"),mGainP[2]);
				mxmlNewReal(mxmlNewElement(yawNode,"I"),mGainI[2]);
				mxmlNewReal(mxmlNewElement(yawNode,"D"),mGainD[2]);
		mxml_node_t *muCntlNode = mxmlNewElement(xmlRoot,"cntlSys");
			if(mCntlSysFile.size() == 0)
			{
				char c = ' ';
				mxml_node_t *sysFileNode = mxmlNewText(mxmlNewElement(muCntlNode,"sysFile"),0,&c);
			}
			else
				mxml_node_t *sysFileNode = mxmlNewText(mxmlNewElement(muCntlNode,"sysFile"),0,mCntlSysFile.c_str());
			mxmlNewReal(mxmlNewElement(muCntlNode,"gainX"),mGainCntlSys[0]);
			mxmlNewReal(mxmlNewElement(muCntlNode,"gainY"),mGainCntlSys[1]);
			mxmlNewReal(mxmlNewElement(muCntlNode,"gainZ"),mGainCntlSys[2]);
			mxmlNewReal(mxmlNewElement(muCntlNode,"gainXDot"),mGainCntlSys[3]);
			mxmlNewReal(mxmlNewElement(muCntlNode,"gainYDot"),mGainCntlSys[4]);
			mxmlNewReal(mxmlNewElement(muCntlNode,"gainZDot"),mGainCntlSys[5]);
		mxml_node_t *motorNode = mxmlNewElement(xmlRoot,"Motors");
			mxml_node_t *motorTrimNode = mxmlNewElement(motorNode,"Trim");
				mxmlNewInteger(mxmlNewElement(motorTrimNode,"N"),mMotorTrim[0]);
				mxmlNewInteger(mxmlNewElement(motorTrimNode,"E"),mMotorTrim[1]);
				mxmlNewInteger(mxmlNewElement(motorTrimNode,"S"),mMotorTrim[2]);
				mxmlNewInteger(mxmlNewElement(motorTrimNode,"W"),mMotorTrim[3]);
		mxml_node_t *observerNode = mxmlNewElement(xmlRoot,"Observer");
			mxmlNewReal(mxmlNewElement(observerNode,"Kp"),mObserverGainP);
			mxmlNewReal(mxmlNewElement(observerNode,"Ki"),mObserverGainI);
			mxmlNewReal(mxmlNewElement(observerNode,"accelWeight"),mObserverWeights[0]);
			mxmlNewReal(mxmlNewElement(observerNode,"magWeight"),mObserverWeights[1]);
	//		mxmlNewReal(mxmlNewElement(observerNode,"gravBandwidth"),mGravBandwidth);

		mxml_node_t *imageNode = mxmlNewElement(xmlRoot,"ImgProc");
			mxmlNewInteger(mxmlNewElement(imageNode,"Box0Min"),mFiltBoxColorMin[0]);
			mxmlNewInteger(mxmlNewElement(imageNode,"Box0Max"),mFiltBoxColorMax[0]);
			mxmlNewInteger(mxmlNewElement(imageNode,"Box1Min"),mFiltBoxColorMin[1]);
			mxmlNewInteger(mxmlNewElement(imageNode,"Box1Max"),mFiltBoxColorMax[1]);
			mxmlNewInteger(mxmlNewElement(imageNode,"Box2Min"),mFiltBoxColorMin[2]);
			mxmlNewInteger(mxmlNewElement(imageNode,"Box2Max"),mFiltBoxColorMax[2]);
			mxmlNewInteger(mxmlNewElement(imageNode,"Box3Min"),mFiltBoxColorMin[3]);
			mxmlNewInteger(mxmlNewElement(imageNode,"Box3Max"),mFiltBoxColorMax[3]);
			mxmlNewInteger(mxmlNewElement(imageNode,"SatMin"),mFiltSatMin);
			mxmlNewInteger(mxmlNewElement(imageNode,"SatMax"),mFiltSatMax);
			mxmlNewInteger(mxmlNewElement(imageNode,"ValMin"),mFiltValMin);
			mxmlNewInteger(mxmlNewElement(imageNode,"ValMax"),mFiltValMax);
			mxmlNewInteger(mxmlNewElement(imageNode,"ConvMin"),mFiltConvMin);
			mxmlNewInteger(mxmlNewElement(imageNode,"ConvMax"),mFiltConvMax);
			mxmlNewInteger(mxmlNewElement(imageNode,"CircMin"),mFiltCircMin);
			mxmlNewInteger(mxmlNewElement(imageNode,"CircMax"),mFiltCircMax);
			mxmlNewInteger(mxmlNewElement(imageNode,"AreaMin"),mFiltAreaMin);
			mxmlNewInteger(mxmlNewElement(imageNode,"AreaMax"),mFiltAreaMax);

		mxml_node_t *ibvsNode = mxmlNewElement(xmlRoot,"IBVS");
			mxmlNewReal(mxmlNewElement(ibvsNode,"imgX"),mIbvsGainImg[0]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"imgY"),mIbvsGainImg[1]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"imgArea"),mIbvsGainImg[2]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"xFlow"),mIbvsGainFlow[0]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"yFlow"),mIbvsGainFlow[1]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"zFlow"),mIbvsGainFlow[2]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"xFlowInt"),mIbvsGainFlowInt[0]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"yFlowInt"),mIbvsGainFlowInt[1]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"zFlowInt"),mIbvsGainFlowInt[2]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"xFF"),mIbvsGainFF[0]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"yFF"),mIbvsGainFF[1]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"zFF"),mIbvsGainFF[2]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"rollAttCmdOffset" ),mAttCmdOffset[0]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"pitchAttCmdOffset"),mAttCmdOffset[1]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"yawAttCmdOffset"  ),mAttCmdOffset[2]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"xOmega"),mIbvsGainAngularRate[0]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"yOmega"),mIbvsGainAngularRate[1]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"zOmega"),mIbvsGainAngularRate[2]);
			mxmlNewReal(mxmlNewElement(ibvsNode,"angle"),mIbvsGainAngle);
			mxmlNewReal(mxmlNewElement(ibvsNode,"dynamic"),mIbvsGainDynamic);

		// Kalman Filter
		mxml_node_t *kfNode = mxmlNewElement(xmlRoot,"KalmanFilter");
			mxml_node_t *attBiasNode = mxmlNewElement(kfNode,"attBias");
				mxmlNewReal(mxmlNewElement(attBiasNode,"roll"),mAttBias[0][0]);
				mxmlNewReal(mxmlNewElement(attBiasNode,"pitch"),mAttBias[1][0]);
				mxmlNewReal(mxmlNewElement(attBiasNode,"yaw"),mAttBias[2][0]);
			mxmlNewReal(mxmlNewElement(kfNode,"attBiasGain"),mAttBiasGain);
			mxmlNewReal(mxmlNewElement(kfNode,"forceScalingGain"),mForceScalingGain);
			mxmlNewReal(mxmlNewElement(kfNode,"posMeasStdDev"),mKfPosMeasStdDev);
			mxmlNewReal(mxmlNewElement(kfNode,"velMeasStdDev"),mKfVelMeasStdDev);

		mxml_node_t *logNode = mxmlNewElement(xmlRoot,"Log");
			mxmlNewInteger(mxmlNewElement(logNode,"Mask"),mLogMask);
	mMutex_data.unlock();

	FILE *fp = fopen((filename).c_str(),"w");
	mxmlSaveFile(xmlRoot, fp, ICSL::XmlUtils::whitespaceCallback);
	fclose(fp);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Event Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////
void PhoneInterface::onBtnApply_clicked()
{
	mMutex_data.lock();
	try
	{
		while(treePhoneConfig->topLevelItemCount() > 0)
		{
			QTreeWidgetItem *item = treePhoneConfig->takeTopLevelItem(0);
			if(item->text(0) == "Communication")
				applyCommConfig(item);
			else if(item->text(0) == "Controller")
				applyControlConfig(item);
			else if(item->text(0) == "Controller System")
				applyControllerSysConfig(item);
			else if(item->text(0) == "Motors")
				applyMotorConfig(item);
			else if(item->text(0) == "Observer")
				applyObserverConfig(item);
			else if(item->text(0) == "IBVS Gains")
				applyIbvsConfig(item);
			else if(item->text(0) == "Kalman Filter")
				applyKalmanFilterConfig(item);
			else if(item->text(0) == "Logging")
				applyLogConfig(item);
			else
			{
				QMessageBox box(QMessageBox::Warning, "Config Error", "Unknown phone config tree item " + item->text(0));
				box.exec();
				throw("Unknown config tree item: " + item->text(0));
			}
		}

		mFiltBoxColorMin[0] = spnBox0Min->value();
		mFiltBoxColorMax[0] = spnBox0Max->value();
		mFiltBoxColorMin[1] = spnBox1Min->value();
		mFiltBoxColorMax[1] = spnBox1Max->value();
		mFiltBoxColorMin[2] = spnBox2Min->value();
		mFiltBoxColorMax[2] = spnBox2Max->value();
		mFiltBoxColorMin[3] = spnBox3Min->value();
		mFiltBoxColorMax[3] = spnBox3Max->value();
		mFiltSatMin = spnSatMin->value();
		mFiltSatMax = spnSatMax->value();
		mFiltValMin = spnValMin->value();
		mFiltValMax = spnValMax->value();
		mFiltCircMin = spnCircMin->value();
		mFiltCircMax = spnCircMax->value();
		mFiltConvMin = spnConvMin->value();
		mFiltConvMax = spnConvMax->value();
		mFiltAreaMin = spnAreaMin->value();
		mFiltAreaMax = spnAreaMax->value();

	}
	catch(const char* exStr)
	{ 
		QMessageBox box(QMessageBox::Warning, "Config Error", exStr);
		box.exec();
	}
	mMutex_data.unlock();

	populateConfigTree();
}

void PhoneInterface::onBtnResetConfig_clicked()
{
	populateConfigTree();
	populateImgProcParams();
}

void PhoneInterface::populateImgProcParams()
{
	spnBox0Min->setValue(mFiltBoxColorMin[0]);
	spnBox0Max->setValue(mFiltBoxColorMax[0]);
	spnBox1Min->setValue(mFiltBoxColorMin[1]);
	spnBox1Max->setValue(mFiltBoxColorMax[1]);
	spnBox2Min->setValue(mFiltBoxColorMin[2]);
	spnBox2Max->setValue(mFiltBoxColorMax[2]);
	spnBox3Min->setValue(mFiltBoxColorMin[3]);
	spnBox3Max->setValue(mFiltBoxColorMax[3]);
	spnSatMin->setValue(mFiltSatMin);
	spnSatMax->setValue(mFiltSatMax);
	spnValMin->setValue(mFiltValMin);
	spnValMax->setValue(mFiltValMax);
	spnCircMin->setValue(mFiltCircMin);
	spnCircMax->setValue(mFiltCircMax);
	spnConvMin->setValue(mFiltConvMin);
	spnConvMax->setValue(mFiltConvMax);
	spnAreaMin->setValue(mFiltAreaMin);
	spnAreaMax->setValue(mFiltAreaMax);
}

void PhoneInterface::onBtnLoadFromFile_clicked()
{
	QFileDialog fileDialog(this,"Open config","../","Phone config files (*.phoneConfig)");
	fileDialog.setDefaultSuffix("phoneConfig");
	fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
	string filename;
	if(fileDialog.exec())
		filename = fileDialog.selectedFiles().front().toStdString();
	else
		return;

	loadConfigFromFile(filename);
}

void PhoneInterface::onBtnSaveToFile_clicked()
{
	QFileDialog fileDialog(this,"Save config","../","Phone config files (*.phoneConfig)");
	fileDialog.setDefaultSuffix("phoneConfig");
	fileDialog.setAcceptMode(QFileDialog::AcceptSave);
	string filename;
	if(fileDialog.exec())
		filename = fileDialog.selectedFiles().front().toStdString();
	else
		return;

	saveConfigToFile(filename);
}

void PhoneInterface::onBtnConnect_clicked()
{
	if(btnConnect->text() == "Connect")
	{
		cout << "Connecting to phone at " << mIP << ":" << mPort << " ... ";
		lblPhoneStatus->setText("Connecting ...");
		try
		{
			if(mSocketTCP == NULL)
				mSocketTCP = Socket::ptr(Socket::createTCPSocket());
			mSocketTCP->connect(mIP.c_str(),mPort);
			if(mSocketUDP != NULL)
				mSocketUDP->close();
			mSocketUDP = Socket::ptr(Socket::createUDPSocket());
			mSocketUDP->bind(mPort);
			cout << "done." << endl;
			btnConnect->setText("Disconnect");

			sendParams();
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onPhoneConnected();
		}
		catch (...)
		{ cout << "Failed to connect" << endl; }

	}
	else
	{
		if(mSocketTCP != NULL)
		{
			mSocketTCP->close();
			mSocketTCP = NULL;
		}
		if(mSocketUDP != NULL)
		{
			mSocketUDP->close();
			mSocketUDP = NULL;
		}
		btnConnect->setText("Connect");
	}		
}

void PhoneInterface::onBtnSendParams_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send motor start because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	sendParams();
}

void PhoneInterface::onBtnResetObserver_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send command because I'm currently not connected to the phone.");
		box.exec();
		return;
	}
	int code = COMM_OBSV_RESET;
	sendTCP((tbyte*)&code,sizeof(code));
}

void PhoneInterface::onBtnSyncTime_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send command because I'm currently not connected to the phone.");
		box.exec();
		return;
	}
	int code = COMM_TIME_SYNC;
	int time = (int)(mSys.mtime() - mStartTimeUniverseMS);

	sendTCP((tbyte*)&code, sizeof(code));
	sendTCP((tbyte*)&time, sizeof(time));
}

void PhoneInterface::onBtnRequestLogFile_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send command because I'm currently not connected to the phone.");
		box.exec();
		return;
	}
	int code = COMM_LOG_TRANSFER;

	sendTCP((tbyte*)&code, sizeof(code));
}

void PhoneInterface::onBtnSendMuCntl_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send controller because I'm currently not connected to the phone.");
		box.exec();
		return;
	}
	int code = COMM_SEND_CNTL_SYSTEM;

	cout << "Sending mu controller" << endl;
	try
	{
		SystemModelLinear sys;
		sys.loadFromFile(mCntlSysFile.c_str());
		cout << "System loaded from " << mCntlSysFile << endl;
		vector<tbyte> buff;
		sys.serialize(buff);
		cout << buff.size() << " bytes serialized" << endl;
		uint32 size = (uint32)buff.size();
		sendTCP((tbyte*)&code, sizeof(code));
		sendTCP((tbyte*)&size, sizeof(size));
		sendTCP(&(buff[0]), size);
	}
	catch(exception e)
	{
		QMessageBox box(QMessageBox::Warning, "Send Error", QString("Failed to send mu controller. ")+e.what());
		box.exec();
	}
}

void PhoneInterface::onBtnClearLog_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't clear log because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	cout << "Clearing phone log" << endl;
	int code = COMM_CLEAR_LOG;
	sendTCP((tbyte*)&code, sizeof(code));
}

void PhoneInterface::onChkUseMuCntl_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't enable mu controller because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	int code = COMM_CNTL_TYPE;
	uint16 cntlType = chkUseMuCntl->isChecked() ? CNTL_TRANSLATION_SYS : CNTL_TRANSLATION_PID;

	cout << "Setting control type" << endl;
	sendTCP((tbyte*)&code, sizeof(code));
	sendTCP((tbyte*)&cntlType, sizeof(cntlType));
}

void PhoneInterface::onChkViewBinarizedImage_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't change view because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	int code = COMM_IMGVIEW_TYPE;
	uint16 viewType = chkViewBinarizedImage->isChecked() ? 1 : 0;

	cout << "Setting image view type" << endl;
	sendTCP((tbyte*)&code, sizeof(code));
	sendTCP((tbyte*)&viewType, sizeof(viewType));
}

void PhoneInterface::onChkUseIbvsController_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't tell phone to use IBVS controller because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	int code = COMM_USE_IBVS;
	uint16 useIbvs = chkUseIbvsController->isChecked() ? 1 : 0;

	cout << "Setting IBVS usage to " << useIbvs << endl;
	sendTCP((tbyte*)&code, sizeof(code));
	sendTCP((tbyte*)&useIbvs, sizeof(useIbvs));
}

void PhoneInterface::onBtnResetDesImgMoment_clicked()
{
	for(int i=0; i<2; i++)
		tblImgState->item(1,i)->setText(QString::number(mDesiredStateImage[i],'f',3));
}

void PhoneInterface::onBtnConfirmDesImgMoment_clicked()
{
	mDesiredStateImage[0] = tblImgState->item(1,0)->text().toDouble();
	mDesiredStateImage[1] = tblImgState->item(1,1)->text().toDouble();
//	mDesiredStateImage[2] = tblImgState->item(1,2)->text().toDouble();
}

void PhoneInterface::onBtnSetYawZero_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Not connected to the phone.");
		box.exec();
		return;
	}

	int code = COMM_SET_YAW_ZERO;

	cout << "Setting yaw zero " << endl;
	sendTCP((tbyte*)&code, sizeof(code));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Helper functions
////////////////////////////////////////////////////////////////////////////////////////////////////
void PhoneInterface::applyCommConfig(QTreeWidgetItem *root)
{
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
		if(item->text(0) == "IP")
			mIP = item->text(1).toStdString();
		else if(item->text(0) == "Port")
			mPort = item->text(1).toInt();
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown comm config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void PhoneInterface::applyControlConfig(QTreeWidgetItem *root)
{
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
		if(item->text(0) == "P")
		{
			mGainP[0] = item->text(1).toDouble();
			mGainP[1] = item->text(2).toDouble();
			mGainP[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "I")
		{
			mGainI[0] = item->text(1).toDouble();
			mGainI[1] = item->text(2).toDouble();
			mGainI[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "D")
		{
			mGainD[0] = item->text(1).toDouble();
			mGainD[1] = item->text(2).toDouble();
			mGainD[2] = item->text(3).toDouble();
		}
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone control config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void PhoneInterface::applyControllerSysConfig(QTreeWidgetItem *root)
{
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
		if(item->text(0) == "system file")
			mCntlSysFile = item->text(1).toStdString();
		else if(item->text(0) == "gain x")
			mGainCntlSys[0] = item->text(1).toFloat();
		else if(item->text(0) == "gain y")
			mGainCntlSys[1] = item->text(1).toFloat();
		else if(item->text(0) == "gain z")
			mGainCntlSys[2] = item->text(1).toFloat();
		else if(item->text(0) == "gain xDot")
			mGainCntlSys[3] = item->text(1).toFloat();
		else if(item->text(0) == "gain yDot")
			mGainCntlSys[4] = item->text(1).toFloat();
		else if(item->text(0) == "gain zDot")
			mGainCntlSys[5] = item->text(1).toFloat();
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone motor config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void PhoneInterface::applyMotorConfig(QTreeWidgetItem *root)
{
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
		if(item->text(0) == "Trim")
		{
			mMotorTrim[0] = item->text(1).toInt();
			mMotorTrim[1] = item->text(2).toInt();
			mMotorTrim[2] = item->text(3).toInt();
			mMotorTrim[3] = item->text(4).toInt();
		}
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone motor config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void PhoneInterface::applyObserverConfig(QTreeWidgetItem *root)
{
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
		if(item->text(0) == "Kp")
			mObserverGainP = item->text(1).toDouble();
		else if(item->text(0) == "Ki")
			mObserverGainI = item->text(1).toDouble();
		else if(item->text(0) == "Accel Weight")
			mObserverWeights[0] = item->text(1).toDouble();
		else if(item->text(0) == "Mag Weight")
			mObserverWeights[1] = item->text(1).toDouble();
//		else if(item->text(0) == "Grav BW")
//			mGravBandwidth = item->text(1).toDouble();
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone motor config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void PhoneInterface::applyIbvsConfig(QTreeWidgetItem *root)
{
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
		if(item->text(0) == "Img")
		{
			mIbvsGainImg[0] = item->text(1).toDouble();
		 	mIbvsGainImg[1] = item->text(2).toDouble();
			mIbvsGainImg[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "Flow") 	
		{
			mIbvsGainFlow[0] = item->text(1).toDouble();
			mIbvsGainFlow[1] = item->text(2).toDouble();
			mIbvsGainFlow[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "Flow Int") 	
		{
			mIbvsGainFlowInt[0] = item->text(1).toDouble();
			mIbvsGainFlowInt[1] = item->text(2).toDouble();
			mIbvsGainFlowInt[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "Feedforward")
		{
			mIbvsGainFF[0] = item->text(1).toDouble();
			mIbvsGainFF[1] = item->text(2).toDouble();
			mIbvsGainFF[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "Att Cmd Offset")
		{
			mAttCmdOffset[0] = item->text(1).toDouble();
			mAttCmdOffset[1] = item->text(2).toDouble();
			mAttCmdOffset[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "omega") 	
		{
			mIbvsGainAngularRate[0] = item->text(1).toDouble();
			mIbvsGainAngularRate[1] = item->text(2).toDouble();
			mIbvsGainAngularRate[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "angle") 	mIbvsGainAngle = item->text(1).toDouble();
		else if(item->text(0) == "dynamic") mIbvsGainDynamic = item->text(1).toDouble();
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone IBVS config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void PhoneInterface::applyKalmanFilterConfig(QTreeWidgetItem *root)
 {
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
		if(item->text(0) == "Att Bias")
		{
			mAttBias[0][0] = item->text(1).toDouble();
			mAttBias[1][0] = item->text(2).toDouble();
			mAttBias[2][0] = item->text(3).toDouble();
		}
		else if(item->text(0) == "Att Bias Gain")
			mAttBiasGain = item->text(1).toDouble();
		else if(item->text(0) == "Force Scaling Gain")
			mForceScalingGain = item->text(1).toDouble();
		else if(item->text(0) == "Pos Meas StdDev")
			mKfPosMeasStdDev   = item->text(1).toDouble();
		else if(item->text(0) == "Vel Meas StdDev")
			mKfVelMeasStdDev   = item->text(1).toDouble();
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone Kalman Filter config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void PhoneInterface::applyLogConfig(QTreeWidgetItem *root)
{
	mLogMask = 0;
	for(int i=0; i<root->childCount(); i++)
		mLogMask |= root->child(i)->checkState(1) == Qt::Checked ? 1 << i : 0;
}

void PhoneInterface::populateConfigTree()
{
	mMutex_data.lock();
	while(treePhoneConfig->topLevelItemCount() > 0)
		treePhoneConfig->takeTopLevelItem(0);

	QStringList headers;
	headers << "1" << "2" << "3" << "4" << "5" << "6";
	treePhoneConfig->setHeaderItem(new QTreeWidgetItem((QTreeWidget*)0,headers));

	QStringList commHeadings;
	commHeadings << "Communication" << "";
	QTreeWidgetItem *commRoot = new QTreeWidgetItem((QTreeWidget*)0,commHeadings);
		commRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("IP")))); 
		commRoot->child(commRoot->childCount()-1)->setText(1,QString(mIP.c_str()));
		commRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Port")))); 
		commRoot->child(commRoot->childCount()-1)->setText(1,QString::number(mPort));

	QStringList phoneCntlHeadings;
	phoneCntlHeadings << "Controller" << "Roll" << "Pitch" << "Yaw";		
	QTreeWidgetItem *cntlRoot = new QTreeWidgetItem((QTreeWidget*)0,phoneCntlHeadings);
		cntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("P"))));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(1,QString::number(mGainP[0]));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(2,QString::number(mGainP[1]));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(3,QString::number(mGainP[2]));
		cntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("I"))));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(1,QString::number(mGainI[0]));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(2,QString::number(mGainI[1]));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(3,QString::number(mGainI[2]));
		cntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("D"))));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(1,QString::number(mGainD[0]));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(2,QString::number(mGainD[1]));
			cntlRoot->child(cntlRoot->childCount()-1)->setText(3,QString::number(mGainD[2]));

	QStringList muCntlHeadings;
	muCntlHeadings << "Controller System";
	QTreeWidgetItem *muCntlRoot = new QTreeWidgetItem((QTreeWidget*)0,muCntlHeadings);
		muCntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("system file"))));
			muCntlRoot->child(muCntlRoot->childCount()-1)->setText(1,QString(mCntlSysFile.c_str()));
		muCntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("gain x"))));
			muCntlRoot->child(muCntlRoot->childCount()-1)->setText(1,QString::number(mGainCntlSys[0]));
		muCntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("gain y"))));
			muCntlRoot->child(muCntlRoot->childCount()-1)->setText(1,QString::number(mGainCntlSys[1]));
		muCntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("gain z"))));
			muCntlRoot->child(muCntlRoot->childCount()-1)->setText(1,QString::number(mGainCntlSys[2]));
		muCntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("gain xDot"))));
			muCntlRoot->child(muCntlRoot->childCount()-1)->setText(1,QString::number(mGainCntlSys[3]));
		muCntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("gain yDot"))));
			muCntlRoot->child(muCntlRoot->childCount()-1)->setText(1,QString::number(mGainCntlSys[4]));
		muCntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("gain zDot"))));
			muCntlRoot->child(muCntlRoot->childCount()-1)->setText(1,QString::number(mGainCntlSys[5]));

	QStringList motorsHeadings;
	motorsHeadings << "Motors" << "N" << "E" << "S" << "W";
	QTreeWidgetItem *motorRoot = new QTreeWidgetItem((QTreeWidget*)0,motorsHeadings);
	motorRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Trim"))));
		motorRoot->child(motorRoot->childCount()-1)->setText(1,QString::number((int)mMotorTrim[0]));
		motorRoot->child(motorRoot->childCount()-1)->setText(2,QString::number((int)mMotorTrim[1]));
		motorRoot->child(motorRoot->childCount()-1)->setText(3,QString::number((int)mMotorTrim[2]));
		motorRoot->child(motorRoot->childCount()-1)->setText(4,QString::number((int)mMotorTrim[3]));

	QStringList observerHeadings;
	observerHeadings << "Observer";
	QTreeWidgetItem *observerRoot = new QTreeWidgetItem((QTreeWidget*)0,observerHeadings);
		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Kp"))));
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mObserverGainP));
		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Ki"))));
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mObserverGainI));
		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Accel Weight"))));
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mObserverWeights[0]));
		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Mag Weight"))));
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mObserverWeights[1]));
//		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Grav BW"))));
//			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mGravBandwidth));

	QStringList ibvsHeadings; ibvsHeadings << "IBVS Gains" << "x" << "y" << "z";
	QTreeWidgetItem *ibvsRoot = new QTreeWidgetItem((QTreeWidget*)0,ibvsHeadings);
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Img"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainImg[0]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainImg[1]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainImg[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Flow"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainFlow[0]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainFlow[1]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainFlow[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Flow Int"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainFlowInt[0]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainFlowInt[1]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainFlowInt[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Feedforward"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainFF[0]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainFF[1]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainFF[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Att Cmd Offset"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mAttCmdOffset[0]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mAttCmdOffset[1]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mAttCmdOffset[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("omega"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainAngularRate[0]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainAngularRate[1]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainAngularRate[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("angle"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainAngle));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("dynamic"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainDynamic));

	// Kalman Filter
	QStringList kfHeadings;
	kfHeadings << "Kalman Filter";
	QTreeWidgetItem *kfRoot = new QTreeWidgetItem((QTreeWidget*)0,kfHeadings);
		kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Att Bias"))));
			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mAttBias[0][0]));
			kfRoot->child(kfRoot->childCount()-1)->setText(2,QString::number(mAttBias[1][0]));
			kfRoot->child(kfRoot->childCount()-1)->setText(3,QString::number(mAttBias[2][0]));
		kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Att Bias Gain"))));
			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mAttBiasGain));
		kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Force Scaling Gain"))));
			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mForceScalingGain));
		kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Pos Meas StdDev"))));
			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mKfPosMeasStdDev));
			kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Vel Meas StdDev"))));
			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mKfVelMeasStdDev));

	QStringList logHeadings;
	logHeadings << "Logging";
	QTreeWidgetItem *logRoot = new QTreeWidgetItem((QTreeWidget*)0,logHeadings);
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("State"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Des State"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Motors"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("PC Comm"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Obsv Update"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Obsv Bias"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Magnometer"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Accelerometer"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Gyroscope"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Camera Results"))));
		logRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Camera Images"))));
		uint32 logFlags[] = {STATE, STATE_DES, MOTORS, PC_UPDATES, OBSV_UPDATE, OBSV_BIAS,
							 MAGNOMETER, ACCEL, GYRO, CAM_RESULTS, CAM_IMAGES};
		for(int i=0; i<logRoot->childCount(); i++)
		{
			QTreeWidgetItem *itm = logRoot->child(i);
			itm->setFlags(itm->flags() | Qt::ItemIsUserCheckable);
			if(mLogMask & logFlags[i])
				itm->setCheckState(1,Qt::Checked);
			else
				itm->setCheckState(1,Qt::Unchecked);
		}

	
	mMutex_data.unlock();

	treePhoneConfig->addTopLevelItem(commRoot);
	treePhoneConfig->addTopLevelItem(cntlRoot);
	treePhoneConfig->addTopLevelItem(muCntlRoot);
	treePhoneConfig->addTopLevelItem(motorRoot);
	treePhoneConfig->addTopLevelItem(observerRoot);
	treePhoneConfig->addTopLevelItem(ibvsRoot);
	treePhoneConfig->addTopLevelItem(kfRoot);
	treePhoneConfig->addTopLevelItem(logRoot);
	treePhoneConfig->expandAll();

	formatTree(commRoot);
	formatTree(cntlRoot);
	formatTree(muCntlRoot);
	formatTree(motorRoot);
	formatTree(observerRoot);
	formatTree(ibvsRoot);
	formatTree(kfRoot);
	formatTree(logRoot);


	int width = 0;
	for(int i=0; i<treePhoneConfig->columnCount(); i++)
	{
		commRoot->setBackground(i,QColor(50,0,0,50));
		cntlRoot->setBackground(i,QColor(50,0,0,50));
		muCntlRoot->setBackground(i,QColor(50,0,0,50));
		motorRoot->setBackground(i,QColor(50,0,0,50));
		observerRoot->setBackground(i,QColor(50,0,0,50));
		ibvsRoot->setBackground(i,QColor(50,0,0,50));
		kfRoot->setBackground(i,QColor(50,0,0,50));
		logRoot->setBackground(i,QColor(50,0,0,50));
		treePhoneConfig->resizeColumnToContents(i);
//		treePhoneConfig->setColumnWidth(i,treePhoneConfig->columnWidth(i)+4);
		width += treePhoneConfig->columnWidth(i);
	}

	treePhoneConfig->setHeaderHidden(true);
	treePhoneConfig->setMinimumSize(width,treePhoneConfig->height());
}

void PhoneInterface::formatTree(QTreeWidgetItem *root)
{
	root->setFlags(root->flags() | Qt::ItemIsEditable);
	for(int i=1; i<root->columnCount(); i++)
		root->setTextAlignment(i,Qt::AlignHCenter);

	if(root->childCount() > 0)
		for(int i=0; i<root->childCount(); i++)
			formatTree(root->child(i));
}

Collection<double> PhoneInterface::getDesiredImageMoment(double z)
{
	mMutex_data.lock();
		Collection<double> mom(3);
		mom[0] = z*(mDesiredStateImage[0]-160);
		mom[1] = z*(mDesiredStateImage[1]-120);
		mom[2] = z;
	mMutex_data.unlock();
	
	return mom;
}

void PhoneInterface::toggleIbvs()
{
	chkUseIbvsController->setChecked(!chkUseIbvsController->isChecked());
	onChkUseIbvsController_clicked();
}

int PhoneInterface::receiveTCP(Socket::ptr socket, tbyte* data, int size)
{
	int received;
	mMutex_socketTCP.lock();
	try
	{
		received= socket->receive(data,size);
		while(received > 0 && received < size)
		{
			cout << "saving the world" << endl;
			received += socket->receive(data+received, size-received);
		}
	}
	catch (...)
	{
		received= -1;
		socket->close();
		socket = NULL;
		socket = Socket::ptr(Socket::createTCPSocket());
	}
	if(received!= size)
		cout << "Read failure:" << received<< " vs " << size <<  endl;
	mMutex_socketTCP.unlock();

	return received;
}

int PhoneInterface::receiveUDP(Socket::ptr socket, tbyte* data, int size)
{
	uint32 addr;
	int port;
	int received = socket->receiveFrom(data, size, addr, port);
	
	return received;
}

bool PhoneInterface::receivePacket(Socket::ptr socket, Packet &pck, int size)
{
	Collection<tbyte> buff(size);

	int result = receiveUDP(socket, buff, size);
	if(result > 0)
		pck.deserialize(buff);

	if(pck.size != result)
		cout << "Only received " << result << " vs " << pck.size << endl;;

	return pck.size == result;
}

void PhoneInterface::receiveLogFile(Socket::ptr socket, string filename)
{
	fstream file(filename.c_str(), fstream::out);
	int length;
	mMutex_socketTCP.lock();
	while(receiveTCP(socket, (tbyte*)&length,sizeof(length)) && length > 0)
	{
		cout << "Length is " << length << endl;
		char *buff = new char[length];
		receiveTCP(socket, (tbyte*)buff, length);
		file.write(buff, length);
		delete buff;
	}
	mMutex_socketTCP.unlock();
	cout << "Terminate length is " << length << endl;

	file.close();
}

}
}
