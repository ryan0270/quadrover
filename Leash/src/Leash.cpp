#include "Leash.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

Leash::Leash(QWidget *parent) : 
	QWidget(parent),
	ui(new Ui::Leash),
	mAttBias(3,1,0.0),
	mIntMemory(3,1,0.0),
	mAttObsvDirWeights(2,0.5),
	mAttObsvNominalMag(3,0),
	mKalmanMeasVar(6,1),
	mKalmanDynVar(6,0),
	mKalmanAttBias(3,0),
	mKalmanAttBiasAdaptGain(3,0)
{
	ui->setupUi(this);

	mIP = string("0.0.0.0");
	mPort = 13120;

	mCntlGainTransP[0] = mCntlGainTransP[1] = mCntlGainTransP[2] = 0;
	mCntlGainTransD[0] = mCntlGainTransD[1] = mCntlGainTransD[2] = 0;
	mCntlGainTransI[0] = mCntlGainTransI[1] = mCntlGainTransI[2] = 0;
	mCntlGainTransILimit[0] = mCntlGainTransILimit[1] = mCntlGainTransILimit[2] = 0;
	mCntlGainAttP[0] = mCntlGainAttP[1] = mCntlGainAttP[2] = 0;
	mCntlGainAttD[0] = mCntlGainAttD[1] = mCntlGainAttD[2] = 0;
	mAttObsvGainP = 1;
	mAttObsvGainI = 1;

	mMotorValues[0] = mMotorValues[1] = mMotorValues[2] = mMotorValues[3] = 0;
	mTimeMS = 0;

	mMotorTrim[0] = mMotorTrim[1] = mMotorTrim[2] = mMotorTrim[3] = 0;

	mCntlCalcTimeUS = 0;
	mImgProcTimeUS = 0;

	mLogMask = PC_UPDATES;

//	mUseMotors = false;
	mUseIbvs = false;

	mKalmanForceGainAdaptGain = 0;

	mKalmanForceGainAdaptGain = 0;

	mMotorForceGain = 0;
	mMotorTorqueGain = 0;
	mMotorArmLength = 1;
	mTotalMass = 1;
}

Leash::~Leash()
{
	if(mSocketTCP != NULL)
		mSocketTCP->close();
	if(mSocketUDP != NULL)
		mSocketUDP->close();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Public functions
////////////////////////////////////////////////////////////////////////////////////////////////////
void Leash::initialize()
{
	connect(ui->btnLoadConfig,SIGNAL(clicked()),this,SLOT(onBtnLoadConfig_clicked()));
	connect(ui->btnSaveConfig,SIGNAL(clicked()),this,SLOT(onBtnSaveConfig_clicked()));
	connect(ui->btnResetConfig,SIGNAL(clicked()),this,SLOT(onBtnResetConfig_clicked()));
	connect(ui->btnApply,SIGNAL(clicked()),this,SLOT(onBtnApply_clicked()));
//	connect(btnPhoneResetConfig,SIGNAL(clicked()),this,SLOT(onBtnResetConfig_clicked()));
//	connect(btnPhoneLoadFromFile,SIGNAL(clicked()),this,SLOT(onBtnLoadFromFile_clicked()));
//	connect(btnPhoneSaveToFile,SIGNAL(clicked()),this,SLOT(onBtnSaveToFile_clicked()));
	connect(ui->btnConnect,SIGNAL(clicked()),this,SLOT(onBtnConnect_clicked()));
//	connect(btnSendParameters,SIGNAL(clicked()),this,SLOT(onBtnSendParams_clicked()));
//	connect(btnResetObserver,SIGNAL(clicked()),this,SLOT(onBtnResetObserver_clicked()));
//	connect(btnSyncTime,SIGNAL(clicked()),this,SLOT(onBtnSyncTime_clicked()));
//	connect(btnRequestLogFile,SIGNAL(clicked()),this,SLOT(onBtnRequestLogFile_clicked()));
//	connect(btnSendMuCntl,SIGNAL(clicked()),this,SLOT(onBtnSendMuCntl_clicked()));
//	connect(btnClearLog,SIGNAL(clicked()),this,SLOT(onBtnClearLog_clicked()));
//	connect(chkUseMuCntl,SIGNAL(clicked()),this,SLOT(onChkUseMuCntl_clicked()));
//	connect(chkViewBinarizedImage,SIGNAL(clicked()),this,SLOT(onChkViewBinarizedImage_clicked()));
//	connect(chkUseIbvsController,SIGNAL(clicked()),this,SLOT(onChkUseIbvsController_clicked()));
//	connect(btnResetDesImgMoment,SIGNAL(clicked()),this,SLOT(onBtnResetDesImgMoment_clicked()));
//	connect(btnConfirmDesImgMoment,SIGNAL(clicked()),this,SLOT(onBtnConfirmDesImgMoment_clicked()));
//	connect(btnSetYawZero,SIGNAL(clicked()),this,SLOT(onBtnSetYawZero_clicked()));

	mSocketTCP = Socket::ptr(Socket::createTCPSocket());
	mSocketTCP = NULL;
//	mSocketUDP = Socket::ptr(Socket::createUDPSocket());
	mSocketUDP = NULL;

//	mIbvsGainImg.resize(3); mIbvsGainImg[0] = mIbvsGainImg[1] = mIbvsGainImg[2] = 0;
//	mIbvsGainFlow.resize(3); mIbvsGainFlow[0] = mIbvsGainFlow[1] = mIbvsGainFlow[2] = 0;
//	mIbvsGainFlowInt.resize(3); mIbvsGainFlowInt[0] = mIbvsGainFlowInt[1] = mIbvsGainFlowInt[2] = 0;
//	mIbvsGainFF.resize(3); mIbvsGainFF[0] = mIbvsGainFF[1] = mIbvsGainFF[2] = 0;
//	mAttCmdOffset.resize(3); mAttCmdOffset[0] = mAttCmdOffset[1] = mAttCmdOffset[2] = 0;	
	mIbvsGainAngularRate.resize(3); mIbvsGainAngularRate[0] = mIbvsGainAngularRate[1] = mIbvsGainAngularRate[2] = 1;
	mIbvsGainAngle = 1;
//	mIbvsGainDynamic = 0.1;

	onBtnResetDesImgMoment_clicked();

	// Views for onboard data
	vector<QList<QStandardItem*>* > data;
	data.push_back(&mAttData);
	data.push_back(&mPosData);
	data.push_back(&mVelData);
	data.push_back(&mDesAttData);
	data.push_back(&mDesPosData);
	data.push_back(&mDesVelData);
	data.push_back(&mGyroData);
	data.push_back(&mGyroBiasData);
	data.push_back(&mAccelData);
	data.push_back(&mMagData);
	data.push_back(&mPosIntData);
	data.push_back(&mTorqueIntData);
	vector<QTableView**> views;
	views.push_back(&ui->vwAtt);
	views.push_back(&ui->vwPos);
	views.push_back(&ui->vwVel);
	views.push_back(&ui->vwDesAtt);
	views.push_back(&ui->vwDesPos);
	views.push_back(&ui->vwDesVel);
	views.push_back(&ui->vwGyro);
	views.push_back(&ui->vwGyroBias);
	views.push_back(&ui->vwAccel);
	views.push_back(&ui->vwMag);
	views.push_back(&ui->vwPosInt);
	views.push_back(&ui->vwTorqueInt);
	QStringList attLabels;
	attLabels << "Roll" << "Pitch" << "Yaw";
	QStringList xyzLabels;
	xyzLabels << "x" << "y" << "z";

	for(int mdl=0; mdl<(int)data.size(); mdl++)
	{
		for(int i=0; i<3; i++)
		{
			data[mdl]->push_back(new QStandardItem(QString("%1").arg(QString::number(0, 'f', 1),10)));
			data[mdl]->back()->setData(Qt::AlignCenter, Qt::TextAlignmentRole);
		}
  		QStandardItemModel *model = new QStandardItemModel(this);
		model->appendRow(*(data[mdl]));
		if(data[mdl] == &mAttData || data[mdl] == &mTorqueIntData)
			model->setHorizontalHeaderLabels(attLabels);
		else
			model->setHorizontalHeaderLabels(xyzLabels);
		(*views[mdl])->setModel(model);

		(*views[mdl])->resizeColumnsToContents();
		(*views[mdl])->resizeRowsToContents();
		(*views[mdl])->verticalHeader()->hide();

		int totWidth = 0;
		int totHeight = 0;
		for(int i=0; i<(*views[mdl])->model()->columnCount(); i++)
			totWidth += (*views[mdl])->columnWidth(i);
		totHeight = (*views[mdl])->rowHeight(0)+(*views[mdl])->horizontalHeader()->height();// + (*views[mdl])->horizontalHeader()->frameWidth();
		(*views[mdl])->setMaximumSize(totWidth, totHeight);
		(*views[mdl])->setMinimumSize(totWidth, totHeight);
		(*views[mdl])->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		(*views[mdl])->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	}

	for(int i=0; i<4; i++)
	{
		mMotorData.push_back(new QStandardItem(QString("%1").arg("0000",4)));
		mMotorData.back()->setData(Qt::AlignCenter, Qt::TextAlignmentRole);
	}
	QStandardItemModel *motorModel = new QStandardItemModel(this);
	motorModel->appendRow(mMotorData);
	QStringList motorHeadings;
	motorHeadings << "N" << "E" << "S" << "W";
	motorModel->setHorizontalHeaderLabels(motorHeadings);
	ui->vwMotors->setModel(motorModel);
	ui->vwMotors->resizeColumnsToContents();
	ui->vwMotors->resizeRowsToContents();
	ui->vwMotors->verticalHeader()->hide();

	int totWidth = 0;
	int totHeight = 0;
	for(int i=0; i<motorModel->columnCount(); i++)
		totWidth += ui->vwMotors->columnWidth(i);
	totHeight = ui->vwMotors->rowHeight(0)+ui->vwMotors->horizontalHeader()->height();// + ui->vwMotors->horizontalHeader()->frameWidth();
	ui->vwMotors->setMaximumSize(totWidth, totHeight);
	ui->vwMotors->setMinimumSize(totWidth, totHeight);
	ui->vwMotors->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	ui->vwMotors->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	vector<QTableWidget*> tables;
	for(int i=0; i<6; i++)
	{
		ui->tblKalmanVar->item(0,i)->setText("-00.0000");
		ui->tblKalmanVar->item(1,i)->setText("-00.0000");
	}
	tables.push_back(ui->tblKalmanVar);

	for(int i=0; i<3; i++)
	{
		ui->tblKalmanAttBias->item(0,i)->setText("-00.0000");
		ui->tblKalmanAttBias->item(1,i)->setText("-00.0000");
	}
	tables.push_back(ui->tblKalmanAttBias);

	for(int i=0; i<2; i++)
		ui->tblAttObsvGains->item(0,i)->setText("-00.0000");
	tables.push_back(ui->tblAttObsvGains);
	
	for(int i=0; i<2; i++)
		ui->tblAttObsvDirWeights->item(0,i)->setText("-00.00");
	tables.push_back(ui->tblAttObsvDirWeights);

	for(int i=0; i<3; i++)
		ui->tblAttObsvNomMag->item(0,i)->setText("-000.00");
	tables.push_back(ui->tblAttObsvNomMag);

	for(int tbl=0; tbl<tables.size(); tbl++)
	{
		tables[tbl]->resizeColumnsToContents();
		tables[tbl]->resizeRowsToContents();
		resizeTableWidget(tables[tbl]);
		setVerticalTabOrder(tables[tbl]);
	}

	cv::Mat img(240,320,CV_8UC3,cv::Scalar(0));
	ui->lblImageDisplay->setPixmap(QPixmap::fromImage(cvMat2QImage(img)));
	ui->lblImageDisplay->setMaximumSize(img.size().width, img.size().height);
	ui->lblImageDisplay->setMinimumSize(img.size().width, img.size().height);

	populateUI();
}

void Leash::pollUDP()
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
						if(pck.dataInt32[0] == 0)
							ui->lblArduinoStatus->setText("Disconnected");
						else
							ui->lblArduinoStatus->setText("Connected");
						break;
					case COMM_USE_MOTORS:
						if(pck.dataBool[0])
							ui->lblMotorStatus->setText("On");
						else
							ui->lblMotorStatus->setText("Off");
						break;
					case COMM_STATE_PHONE:
						for(int i=0; i<3; i++)
						{
							mAttData[i]->setData(QString::number(pck.dataFloat[i],'f',2), Qt::DisplayRole);
							// angular velocity is just the gyro measurement (minus bias, which is small)
							mPosData[i]->setData(QString::number(pck.dataFloat[i+6],'f',3), Qt::DisplayRole);
							mVelData[i]->setData(QString::number(pck.dataFloat[i+9],'f',3), Qt::DisplayRole);
						}
						break;
					case COMM_DESIRED_STATE:
						for(int i=0; i<3; i++)
						{
							mDesAttData[i]->setData(QString::number(pck.dataFloat[i],'f',2), Qt::DisplayRole);
							// angular velocity is just the gyro measurement (minus bias, which is small)
							mDesPosData[i]->setData(QString::number(pck.dataFloat[i+6],'f',3), Qt::DisplayRole);
							mDesVelData[i]->setData(QString::number(pck.dataFloat[i+9],'f',3), Qt::DisplayRole);
						}
						break;
//					case COMM_IMAGE_STATE:
//						{
//							double an = pck.dataFloat[2];
//							mStateImage[0] = pck.dataFloat[0]/an;
//							mStateImage[1] = pck.dataFloat[1]/an;
//							mStateImage[2] = mDesiredStateImage[2]/pow(an/mDesiredHeight,2);
//							for(int i=3; i<6; i++)
//								mStateImage[i] = pck.dataFloat[i];
//						}
//						break;
//					case COMM_DESIRED_IMAGE_STATE:
//						for(int i=3; i<6; i++)
//							mDesiredStateImage[i] = pck.dataFloat[i];
//						break;
					case COMM_GYRO:
						for(int i=0; i<3; i++)
							mGyroData[i]->setData(QString::number(pck.dataFloat[i],'f',2), Qt::DisplayRole);
						break;
					case COMM_ACCEL:
						for(int i=0; i<3; i++)
							mAccelData[i]->setData(QString::number(pck.dataFloat[i],'f',2), Qt::DisplayRole);
						break;
					case COMM_MAGNOMETER:
						for(int i=0; i<3; i++)
							mMagData[i]->setData(QString::number(pck.dataFloat[i],'f',2), Qt::DisplayRole);
						break;
					case COMM_OBSV_BIAS:
						for(int i=0; i<3; i++)
							mGyroBiasData[i]->setData(QString::number(pck.dataFloat[i],'f',3), Qt::DisplayRole);
						break;
					case COMM_MOTOR_VAL:
						for(int i=0; i<4; i++)
							mMotorData[i]->setData(QString::number(pck.dataInt32[i]), Qt::DisplayRole);
						break;
//					case COMM_MOTOR_VAL_IBVS:
//						for(int i=0; i<4; i++)
//							mMotorValuesIbvs[i] = pck.dataInt32[i];
//						break;
					case COMM_INT_MEM_POS:
						for(int i=0; i<3; i++)
							mPosIntData[i]->setData(QString::number(pck.dataFloat[i],'f',3), Qt::DisplayRole);
						break;
					case COMM_INT_MEM_TORQUE:
						for(int i=0; i<3; i++)
							mTorqueIntData[i]->setData(QString::number(pck.dataFloat[i],'f',3), Qt::DisplayRole);
						break;
//					case COMM_CNTL_TYPE:
//						mCurCntlType = pck.dataInt32[0];
//						break;
//					case COMM_CNTL_CALC_TIME:
//						mCntlCalcTimeUS = pck.dataInt32[0];
//						break;
					case COMM_IMGPROC_TIME_US:
						ui->lblImgProcTime->setText(QString::number(pck.dataInt32[0]/1.0e3,'f',0));
						break;
					case COMM_USE_IBVS:
						if(pck.dataBool[0])
							ui->lblIbvsStatus->setText("On");
						else
							ui->lblIbvsStatus->setText("Off");
//						chkUseIbvsController->setChecked(mUseIbvs);
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

void Leash::pollTCP()
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
							cv::Mat img = cv::imdecode(imgData,CV_LOAD_IMAGE_COLOR);
							ui->lblImageDisplay->setPixmap(QPixmap::fromImage(cvMat2QImage(img)));
							ui->lblImageDisplay->setMaximumSize(img.size().width, img.size().height);
							ui->lblImageDisplay->setMinimumSize(img.size().width, img.size().height);
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

void Leash::updateDisplay()
{
	double time = (mSys.mtime() - mStartTimeUniverseMS)/1.0e3;

	////////////////// Temp just to keep the link alive /////////////////////
	Packet pState;
	pState.type = COMM_STATE_VICON;
	pState.time = mSys.mtime()-mStartTimeUniverseMS;
	pState.dataFloat.resize(12);
	for(int i=0; i<pState.dataFloat.size(); i++)
		pState.dataFloat[i] = 0;
	Collection<tbyte> buff;
	pState.serialize(buff);
	sendUDP(buff.begin(), buff.size());
	/////////////////////////////////////////////////////////////////////////
	if(mSocketTCP != NULL)
	{
		pollUDP();
		pollTCP();
	}
}

QImage Leash::cvMat2QImage(const cv::Mat &mat)
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

bool Leash::sendUDP(tbyte* data, int size)
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

bool Leash::sendTCP(tbyte* data, int size)
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

bool Leash::sendMotorStop()
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

bool Leash::sendMotorStart()
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

bool Leash::sendParams()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send phone params because I'm currently not connected to the phone.");
		box.exec();
		return false;
	}

	mMutex_data.lock();
	int code;
	bool result = true;

	code = COMM_MOTOR_TRIM;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)mMotorTrim, 4*sizeof(int));

	code = COMM_OBSV_GAIN;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result =result && sendTCP((tbyte*)&mAttObsvGainP,sizeof(mAttObsvGainP));
	if(result) result =result && sendTCP((tbyte*)&mAttObsvGainI,sizeof(mAttObsvGainI));
	if(result) result =result && sendTCP((tbyte*)&(mAttObsvDirWeights[0]), sizeof(mAttObsvDirWeights[0]));
	if(result) result =result && sendTCP((tbyte*)&(mAttObsvDirWeights[1]), sizeof(mAttObsvDirWeights[1]));

	code = COMM_LOG_MASK; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&mLogMask, sizeof(mLogMask));

	float scale = mMotorForceGain;
	code = COMM_MOTOR_FORCE_SCALING; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&scale,sizeof(scale));

	float torqueScale = mMotorTorqueGain;
	code = COMM_MOTOR_TORQUE_SCALING;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&torqueScale,sizeof(torqueScale));

//	code = COMM_KALMANFILTER_POS_MEAS_STD;
//	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
//	if(result) result = result && sendTCP((tbyte*)&mKfPosMeasStdDev,sizeof(mKfPosMeasStdDev));
//
//	code = COMM_KALMANFILTER_VEL_MEAS_STD;
//	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
//	if(result) result = result && sendTCP((tbyte*)&mKfVelMeasStdDev,sizeof(mKfVelMeasStdDev));

	float m = mTotalMass;
	code = COMM_MASS; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&m,sizeof(m));

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

//	float attBiasGain = mAttBiasGain;
//	code = COMM_ATT_BIAS_GAIN;
//	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
//	if(result) result = result && sendTCP((tbyte*)&attBiasGain,sizeof(attBiasGain));

	float forceScalingGain = mKalmanForceGainAdaptGain;
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

bool Leash::loadConfigFromFile(string filename)
{
	cout << "Loading config from " << filename << endl;
	FILE *fp = fopen((filename).c_str(),"r");
	if(fp == NULL)
	{
		populateUI(); // with whatever is in there right now
		return false;
	}

	mxml_node_t *xmlRoot;
	xmlRoot = mxmlLoadFile(NULL,fp,MXML_TEXT_CALLBACK);
	fclose(fp);

	if(xmlRoot == NULL)
	{
		populateUI(); // with whatever is in there right now
		return false;
	}

	mxml_node_t *cntlRoot = mxmlFindElement(xmlRoot, xmlRoot, "Controller", NULL, NULL, MXML_DESCEND);
	if(cntlRoot != NULL)
		loadControllerConfig(cntlRoot);
	else
		cout << "Controller section not found in config file" << endl;

	mxml_node_t *hdwRoot = mxmlFindElement(xmlRoot, xmlRoot, "Hardware", NULL, NULL, MXML_DESCEND);
	if(hdwRoot != NULL)
		loadHardwareConfig(hdwRoot);
	else
		cout << "Hardware section not found in config file" << endl;

	mxml_node_t *obsvRoot = mxmlFindElement(xmlRoot, xmlRoot, "Observer", NULL, NULL, MXML_DESCEND);
	if(obsvRoot != NULL)
		loadObserverConfig(obsvRoot);
	else
		cout << "Observer section not found in config file" << endl;

	populateUI();
	return true;
}

void Leash::loadControllerConfig(mxml_node_t *cntlRoot)
{
	mMutex_data.lock();
	mxml_node_t *transCntl = mxmlFindElement(cntlRoot, cntlRoot, "Translation", NULL, NULL, MXML_DESCEND);
	if(transCntl != NULL)
	{
		mxml_node_t *p = mxmlFindElement(transCntl, transCntl, "P", NULL, NULL, MXML_DESCEND);
		mxml_node_t *d = mxmlFindElement(transCntl, transCntl, "D", NULL, NULL, MXML_DESCEND);
		mxml_node_t *i = mxmlFindElement(transCntl, transCntl, "I", NULL, NULL, MXML_DESCEND);
		mxml_node_t *iLimit = mxmlFindElement(transCntl, transCntl, "ILimit", NULL, NULL, MXML_DESCEND);

		if(p != NULL)
		{
			mxml_node_t *x = mxmlFindElement(p, p, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *y = mxmlFindElement(p, p, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *z = mxmlFindElement(p, p, "z", NULL, NULL, MXML_DESCEND);

			if(x != NULL) stringstream(x->child->value.text.string) >> mCntlGainTransP[0];
			if(y != NULL) stringstream(y->child->value.text.string) >> mCntlGainTransP[1];
			if(z != NULL) stringstream(z->child->value.text.string) >> mCntlGainTransP[2];
		}

		if(d != NULL)
		{
			mxml_node_t *x = mxmlFindElement(d, d, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *y = mxmlFindElement(d, d, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *z = mxmlFindElement(d, d, "z", NULL, NULL, MXML_DESCEND);

			if(x != NULL) stringstream(x->child->value.text.string) >> mCntlGainTransD[0];
			if(y != NULL) stringstream(y->child->value.text.string) >> mCntlGainTransD[1];
			if(z != NULL) stringstream(z->child->value.text.string) >> mCntlGainTransD[2];
		}

		if(i != NULL)
		{
			mxml_node_t *x = mxmlFindElement(i, i, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *y = mxmlFindElement(i, i, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *z = mxmlFindElement(i, i, "z", NULL, NULL, MXML_DESCEND);

			if(x != NULL) stringstream(x->child->value.text.string) >> mCntlGainTransI[0];
			if(y != NULL) stringstream(y->child->value.text.string) >> mCntlGainTransI[1];
			if(z != NULL) stringstream(z->child->value.text.string) >> mCntlGainTransI[2];
		}

		if(iLimit != NULL)
		{
			mxml_node_t *x = mxmlFindElement(iLimit, iLimit, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *y = mxmlFindElement(iLimit, iLimit, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *z = mxmlFindElement(iLimit, iLimit, "z", NULL, NULL, MXML_DESCEND);

			if(x != NULL) stringstream(x->child->value.text.string) >> mCntlGainTransILimit[0];
			if(y != NULL) stringstream(y->child->value.text.string) >> mCntlGainTransILimit[1];
			if(z != NULL) stringstream(z->child->value.text.string) >> mCntlGainTransILimit[2];
		}
	}
	else
		cout << "Translation controller section not found in config file" << endl;

	mxml_node_t *attCntl = mxmlFindElement(cntlRoot, cntlRoot, "Attitude", NULL, NULL, MXML_DESCEND);
	if(attCntl != NULL)
	{
		mxml_node_t *p = mxmlFindElement(attCntl, attCntl, "P", NULL, NULL, MXML_DESCEND);
		mxml_node_t *d = mxmlFindElement(attCntl, attCntl, "D", NULL, NULL, MXML_DESCEND);

		if(p != NULL)
		{
			mxml_node_t *x = mxmlFindElement(p, p, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *y = mxmlFindElement(p, p, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *z = mxmlFindElement(p, p, "z", NULL, NULL, MXML_DESCEND);

			if(x != NULL) stringstream(x->child->value.text.string) >> mCntlGainAttP[0];
			if(y != NULL) stringstream(y->child->value.text.string) >> mCntlGainAttP[1];
			if(z != NULL) stringstream(z->child->value.text.string) >> mCntlGainAttP[2];
		}

		if(d != NULL)
		{
			mxml_node_t *x = mxmlFindElement(d, d, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *y = mxmlFindElement(d, d, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *z = mxmlFindElement(d, d, "z", NULL, NULL, MXML_DESCEND);

			if(x != NULL) stringstream(x->child->value.text.string) >> mCntlGainAttD[0];
			if(y != NULL) stringstream(y->child->value.text.string) >> mCntlGainAttD[1];
			if(z != NULL) stringstream(z->child->value.text.string) >> mCntlGainAttD[2];
		}
	}
	else
		cout << "Attitdue controller section not found in config file" << endl;

	mMutex_data.unlock();
}

void Leash::loadObserverConfig(mxml_node_t *obsvRoot)
{
	mMutex_data.lock();
	mxml_node_t *transNode = mxmlFindElement(obsvRoot, obsvRoot, "Translation", NULL, NULL, MXML_DESCEND);
	if(transNode != NULL)
	{
		mxml_node_t *measVarNode = mxmlFindElement(transNode, transNode, "MeasVar", NULL, NULL, MXML_DESCEND);
		if(measVarNode != NULL)
		{
			mxml_node_t *xNode = mxmlFindElement(measVarNode, measVarNode, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yNode = mxmlFindElement(measVarNode, measVarNode, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *zNode = mxmlFindElement(measVarNode, measVarNode, "z", NULL, NULL, MXML_DESCEND);
			mxml_node_t *xVelNode = mxmlFindElement(measVarNode, measVarNode, "xVel", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yVelNode = mxmlFindElement(measVarNode, measVarNode, "yVel", NULL, NULL, MXML_DESCEND);
			mxml_node_t *zVelNode = mxmlFindElement(measVarNode, measVarNode, "zVel", NULL, NULL, MXML_DESCEND);

			if(xNode != NULL) stringstream(xNode->child->value.text.string) >> mKalmanMeasVar[0];
			if(yNode != NULL) stringstream(yNode->child->value.text.string) >> mKalmanMeasVar[1];
			if(zNode != NULL) stringstream(zNode->child->value.text.string) >> mKalmanMeasVar[2];
			if(xVelNode != NULL) stringstream(xNode->child->value.text.string) >> mKalmanMeasVar[3];
			if(yVelNode != NULL) stringstream(yNode->child->value.text.string) >> mKalmanMeasVar[4];
			if(zVelNode != NULL) stringstream(zNode->child->value.text.string) >> mKalmanMeasVar[5];
		}

		mxml_node_t *dynVarNode = mxmlFindElement(transNode, transNode, "DynVar", NULL, NULL, MXML_DESCEND);
		if(dynVarNode != NULL)
		{
			mxml_node_t *xNode = mxmlFindElement(dynVarNode, dynVarNode, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yNode = mxmlFindElement(dynVarNode, dynVarNode, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *zNode = mxmlFindElement(dynVarNode, dynVarNode, "z", NULL, NULL, MXML_DESCEND);
			mxml_node_t *xVelNode = mxmlFindElement(dynVarNode, dynVarNode, "xVel", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yVelNode = mxmlFindElement(dynVarNode, dynVarNode, "yVel", NULL, NULL, MXML_DESCEND);
			mxml_node_t *zVelNode = mxmlFindElement(dynVarNode, dynVarNode, "zVel", NULL, NULL, MXML_DESCEND);

			if(xNode != NULL) stringstream(xNode->child->value.text.string) >> mKalmanDynVar[0];
			if(yNode != NULL) stringstream(yNode->child->value.text.string) >> mKalmanDynVar[1];
			if(zNode != NULL) stringstream(zNode->child->value.text.string) >> mKalmanDynVar[2];
			if(xVelNode != NULL) stringstream(xNode->child->value.text.string) >> mKalmanDynVar[3];
			if(yVelNode != NULL) stringstream(yNode->child->value.text.string) >> mKalmanDynVar[4];
			if(zVelNode != NULL) stringstream(zNode->child->value.text.string) >> mKalmanDynVar[5];
		}

		mxml_node_t *attBiasNode = mxmlFindElement(transNode, transNode, "AttBias", NULL, NULL, MXML_DESCEND);
		if(attBiasNode != NULL)
		{
			mxml_node_t *rollNode = mxmlFindElement(attBiasNode, attBiasNode, "roll", NULL, NULL, MXML_DESCEND);
			mxml_node_t *pitchNode = mxmlFindElement(attBiasNode, attBiasNode, "pitch", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yawNode = mxmlFindElement(attBiasNode, attBiasNode, "yaw", NULL, NULL, MXML_DESCEND);

			if(rollNode != NULL) stringstream(rollNode->child->value.text.string) >> mKalmanAttBias[0];
			if(pitchNode != NULL) stringstream(pitchNode->child->value.text.string) >> mKalmanAttBias[1];
			if(yawNode != NULL) stringstream(yawNode->child->value.text.string) >> mKalmanAttBias[2];
		}
		else
			cout << "Kalman att bias node not found in config file" << endl;

		mxml_node_t *attBiasAdaptGainNode = mxmlFindElement(transNode, transNode, "AttBiasAdaptGain", NULL, NULL, MXML_DESCEND);
		if(attBiasAdaptGainNode != NULL)
		{
			mxml_node_t *rollNode = mxmlFindElement(attBiasAdaptGainNode, attBiasAdaptGainNode, "roll", NULL, NULL, MXML_DESCEND);
			mxml_node_t *pitchNode = mxmlFindElement(attBiasAdaptGainNode, attBiasAdaptGainNode, "pitch", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yawNode = mxmlFindElement(attBiasAdaptGainNode, attBiasAdaptGainNode, "yaw", NULL, NULL, MXML_DESCEND);

			if(rollNode != NULL) stringstream(rollNode->child->value.text.string) >> mKalmanAttBiasAdaptGain[0];
			if(pitchNode != NULL) stringstream(pitchNode->child->value.text.string) >> mKalmanAttBiasAdaptGain[1];
			if(yawNode != NULL) stringstream(yawNode->child->value.text.string) >> mKalmanAttBiasAdaptGain[2];
		}

		mxml_node_t *forceScalingAdaptGainNode = mxmlFindElement(transNode, transNode, "ForceGainAdaptGain", NULL, NULL, MXML_DESCEND);
		if(forceScalingAdaptGainNode!= NULL) stringstream(forceScalingAdaptGainNode->child->value.text.string) >> mKalmanForceGainAdaptGain;
	}
	else
		cout << "Translation observer section not found in config file" << endl;

	mxml_node_t *attNode = mxmlFindElement(obsvRoot, obsvRoot, "Attitude", NULL, NULL, MXML_DESCEND);
	if(attNode != NULL)
	{
		mxml_node_t *gainsNode = mxmlFindElement(attNode, attNode, "Gain", NULL, NULL, MXML_DESCEND);
		if(gainsNode!= NULL)
		{
			mxml_node_t *pNode = mxmlFindElement(gainsNode, gainsNode, "P", NULL, NULL, MXML_DESCEND);
			mxml_node_t *iNode = mxmlFindElement(gainsNode, gainsNode, "I", NULL, NULL, MXML_DESCEND);

			if(pNode != NULL) stringstream(pNode->child->value.text.string) >> mAttObsvGainP;
			if(iNode != NULL) stringstream(iNode->child->value.text.string) >> mAttObsvGainI;
		}

		mxml_node_t *dirWeightsNode = mxmlFindElement(attNode, attNode, "DirWeight", NULL, NULL, MXML_DESCEND);
		if(dirWeightsNode!= NULL)
		{
			mxml_node_t *accelNode = mxmlFindElement(dirWeightsNode, dirWeightsNode, "Accel", NULL, NULL, MXML_DESCEND);
			mxml_node_t *magNode = mxmlFindElement(dirWeightsNode, dirWeightsNode, "Mag", NULL, NULL, MXML_DESCEND);

			if(accelNode != NULL) stringstream(accelNode->child->value.text.string) >> mAttObsvDirWeights[0];
			if(magNode != NULL) stringstream(magNode->child->value.text.string) >> mAttObsvDirWeights[1];
		}

		mxml_node_t *nomMagNode = mxmlFindElement(attNode, attNode, "NomMag", NULL, NULL, MXML_DESCEND);
		if(nomMagNode != NULL)
		{
			mxml_node_t *xNode = mxmlFindElement(nomMagNode, nomMagNode, "x", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yNode = mxmlFindElement(nomMagNode, nomMagNode, "y", NULL, NULL, MXML_DESCEND);
			mxml_node_t *zNode = mxmlFindElement(nomMagNode, nomMagNode, "z", NULL, NULL, MXML_DESCEND);

			if(xNode != NULL) stringstream(xNode->child->value.text.string) >> mAttObsvNominalMag[0];
			if(yNode != NULL) stringstream(yNode->child->value.text.string) >> mAttObsvNominalMag[1];
			if(zNode != NULL) stringstream(zNode->child->value.text.string) >> mAttObsvNominalMag[2];
		}
	}
	else
		cout << "Attitude observer section not found in config file" << endl;
	mMutex_data.unlock();
}

void Leash::loadHardwareConfig(mxml_node_t *hdwRoot)
{
	mMutex_data.lock();
	mxml_node_t *forceGainNode = mxmlFindElement(hdwRoot, hdwRoot, "MotorForceGain", NULL, NULL, MXML_DESCEND);
	if(forceGainNode != NULL) stringstream(forceGainNode->child->value.text.string) >> mMotorForceGain;

	mxml_node_t *torqueGainNode = mxmlFindElement(hdwRoot, hdwRoot, "MotorTorqueGain", NULL, NULL, MXML_DESCEND);
	if(torqueGainNode != NULL) stringstream(torqueGainNode->child->value.text.string) >> mMotorTorqueGain;

	mxml_node_t *armLengthNode = mxmlFindElement(hdwRoot, hdwRoot, "MotorArmLength", NULL, NULL, MXML_DESCEND);
	if(armLengthNode != NULL) stringstream(armLengthNode->child->value.text.string) >> mMotorArmLength;

	mxml_node_t *massNode = mxmlFindElement(hdwRoot, hdwRoot, "TotalMass", NULL, NULL, MXML_DESCEND);
	if(massNode != NULL) stringstream(massNode->child->value.text.string) >> mTotalMass;
	mMutex_data.unlock();
}

void Leash::saveConfigToFile(string filename)
{
	mxml_node_t *xmlRoot = mxmlNewXML("1.0");
	mxml_node_t *cntlRoot = mxmlNewElement(xmlRoot,"Controller");
	mxml_node_t *obsvRoot = mxmlNewElement(xmlRoot,"Observer");
	mxml_node_t *hdwRoot = mxmlNewElement(xmlRoot,"Hardware");
	saveControllerConfig(cntlRoot);
	saveObserverConfig(obsvRoot);
	saveHardwareConfig(hdwRoot);

	FILE *fp = fopen((filename).c_str(),"w");
	mxmlSaveFile(xmlRoot, fp, ICSL::XmlUtils::whitespaceCallback);
	fclose(fp);
}

void Leash::saveControllerConfig(mxml_node_t *cntlRoot)
{
	mMutex_data.lock();
	mxml_node_t *transNode = mxmlNewElement(cntlRoot,"Translation");
	{
		mxml_node_t *pNode = mxmlNewElement(transNode,"P");
			mxmlNewReal(mxmlNewElement(pNode,"x"), mCntlGainTransP[0]);
			mxmlNewReal(mxmlNewElement(pNode,"y"), mCntlGainTransP[1]);
			mxmlNewReal(mxmlNewElement(pNode,"z"), mCntlGainTransP[2]);
		mxml_node_t *dNode = mxmlNewElement(transNode,"D");
			mxmlNewReal(mxmlNewElement(dNode,"x"), mCntlGainTransD[0]);
			mxmlNewReal(mxmlNewElement(dNode,"y"), mCntlGainTransD[1]);
			mxmlNewReal(mxmlNewElement(dNode,"z"), mCntlGainTransD[2]);
		mxml_node_t *iNode = mxmlNewElement(transNode,"I");
			mxmlNewReal(mxmlNewElement(iNode,"x"), mCntlGainTransI[0]);
			mxmlNewReal(mxmlNewElement(iNode,"y"), mCntlGainTransI[1]);
			mxmlNewReal(mxmlNewElement(iNode,"z"), mCntlGainTransI[2]);
		mxml_node_t *iLimitNode = mxmlNewElement(transNode,"ILimit");
			mxmlNewReal(mxmlNewElement(iLimitNode,"x"), mCntlGainTransILimit[0]);
			mxmlNewReal(mxmlNewElement(iLimitNode,"y"), mCntlGainTransILimit[1]);
			mxmlNewReal(mxmlNewElement(iLimitNode,"z"), mCntlGainTransILimit[2]);
	}
	mxml_node_t *attNode = mxmlNewElement(cntlRoot,"Attitude");
	{
		mxml_node_t *pNode = mxmlNewElement(attNode,"P");
			mxmlNewReal(mxmlNewElement(pNode,"x"), mCntlGainAttP[0]);
			mxmlNewReal(mxmlNewElement(pNode,"y"), mCntlGainAttP[1]);
			mxmlNewReal(mxmlNewElement(pNode,"z"), mCntlGainAttP[2]);
		mxml_node_t *dNode = mxmlNewElement(attNode,"D");
			mxmlNewReal(mxmlNewElement(dNode,"x"), mCntlGainAttD[0]);
			mxmlNewReal(mxmlNewElement(dNode,"y"), mCntlGainAttD[1]);
			mxmlNewReal(mxmlNewElement(dNode,"z"), mCntlGainAttD[2]);
	}
	mMutex_data.unlock();
}

void Leash::saveObserverConfig(mxml_node_t *obsvRoot)
{
	string xyzLabels[] = {"x", "y", "z", "xVel", "yVel", "zVel"};
	string attLabels[] = {"roll", "pitch", "yaw"};
	mMutex_data.lock();
	mxml_node_t *transNode = mxmlNewElement(obsvRoot,"Translation");
	{
		mxml_node_t *measVarNode = mxmlNewElement(transNode, "MeasVar");
		mxml_node_t *dynVarNode = mxmlNewElement(transNode, "DynVar");
		for(int i=0; i<6; i++)
		{
			mxmlNewReal(mxmlNewElement(measVarNode,xyzLabels[i].c_str()), mKalmanMeasVar[i]);
			mxmlNewReal(mxmlNewElement(dynVarNode,xyzLabels[i].c_str()), mKalmanDynVar[i]);
		}

		mxml_node_t *attBiasNode = mxmlNewElement(transNode, "AttBias");
		mxml_node_t *attBiasAdaptGainNode = mxmlNewElement(transNode, "AttBiasAdaptGain");
		for(int i=0; i<3; i++)
		{
			mxmlNewReal(mxmlNewElement(attBiasNode,attLabels[i].c_str()), mKalmanAttBias[i]);
			mxmlNewReal(mxmlNewElement(attBiasAdaptGainNode,attLabels[i].c_str()), mKalmanAttBiasAdaptGain[i]);
		}

		mxmlNewReal(mxmlNewElement(transNode,"ForceGainAdaptGain"), mKalmanForceGainAdaptGain);
	}

	mxml_node_t *attNode = mxmlNewElement(obsvRoot, "Attitude");
	{
		mxml_node_t *gainNode = mxmlNewElement(attNode, "Gain");
			mxmlNewReal(mxmlNewElement(gainNode, "P"), mAttObsvGainP);
			mxmlNewReal(mxmlNewElement(gainNode, "I"), mAttObsvGainI);
		mxml_node_t *dirWeightNode = mxmlNewElement(attNode, "DirWeight");
			mxmlNewReal(mxmlNewElement(dirWeightNode, "Accel"), mAttObsvDirWeights[0]);
			mxmlNewReal(mxmlNewElement(dirWeightNode, "Mag"), mAttObsvDirWeights[1]);
		mxml_node_t *nomMagNode = mxmlNewElement(attNode, "NomMag");
		for(int i=0; i<3; i++)
			mxmlNewReal(mxmlNewElement(nomMagNode, xyzLabels[i].c_str()), mAttObsvNominalMag[i]);
	}
	mMutex_data.unlock();
}

void Leash::saveHardwareConfig(mxml_node_t *hdwRoot)
{
	mMutex_data.lock();
	mxmlNewReal(mxmlNewElement(hdwRoot, "MotorForceGain"), mMotorForceGain);
	mxmlNewReal(mxmlNewElement(hdwRoot, "MotorTorqueGain"), mMotorTorqueGain);
	mxmlNewReal(mxmlNewElement(hdwRoot, "MotorArmLength"), mMotorArmLength);
	mxmlNewReal(mxmlNewElement(hdwRoot, "TotalMass"), mTotalMass);
	mMutex_data.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Event Handlers
////////////////////////////////////////////////////////////////////////////////////////////////////
void Leash::onBtnApply_clicked()
{
	applyControllerConfig();
	applyObserverConfig();
	applyHardwareConfig();
	mMutex_data.lock();
	try
	{
//		while(treePhoneConfig->topLevelItemCount() > 0)
		while(false)
		{
//			QTreeWidgetItem *item = treePhoneConfig->takeTopLevelItem(0);
			QTreeWidgetItem *item = NULL;
			if(item->text(0) == "Communication")
				applyCommConfig(item);
			else if(item->text(0) == "Controller")
				applyControlConfig(item);
			else if(item->text(0) == "Motors")
				applyMotorConfig(item);
//			else if(item->text(0) == "Observer")
//				applyObserverConfig(item);
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

//		mFiltBoxColorMin[0] = spnBox0Min->value();
//		mFiltBoxColorMax[0] = spnBox0Max->value();
//		mFiltBoxColorMin[1] = spnBox1Min->value();
//		mFiltBoxColorMax[1] = spnBox1Max->value();
//		mFiltBoxColorMin[2] = spnBox2Min->value();
//		mFiltBoxColorMax[2] = spnBox2Max->value();
//		mFiltBoxColorMin[3] = spnBox3Min->value();
//		mFiltBoxColorMax[3] = spnBox3Max->value();
//		mFiltSatMin = spnSatMin->value();
//		mFiltSatMax = spnSatMax->value();
//		mFiltValMin = spnValMin->value();
//		mFiltValMax = spnValMax->value();
//		mFiltCircMin = spnCircMin->value();
//		mFiltCircMax = spnCircMax->value();
//		mFiltConvMin = spnConvMin->value();
//		mFiltConvMax = spnConvMax->value();
//		mFiltAreaMin = spnAreaMin->value();
//		mFiltAreaMax = spnAreaMax->value();

	}
	catch(const char* exStr)
	{ 
		QMessageBox box(QMessageBox::Warning, "Config Error", exStr);
		box.exec();
	}
	mMutex_data.unlock();

	populateUI();
}

void Leash::applyControllerConfig()
{
	for(int i=0; i<3; i++)
	{
		mCntlGainTransP[i] = ui->tblTransCntl->item(i,0)->text().toDouble();
		mCntlGainTransD[i] = ui->tblTransCntl->item(i,1)->text().toDouble();
		mCntlGainTransI[i] = ui->tblTransCntl->item(i,2)->text().toDouble();
		mCntlGainTransILimit[i] = ui->tblTransCntl->item(i,3)->text().toDouble();
		mCntlGainAttP[i] = ui->tblAttCntl->item(i,0)->text().toDouble();
		mCntlGainAttD[i] = ui->tblAttCntl->item(i,1)->text().toDouble();
	}
}

void Leash::applyObserverConfig()
{
	for(int i=0; i<6; i++)
	{
		mKalmanMeasVar[i] = ui->tblKalmanVar->item(0,i)->text().toDouble();
		mKalmanDynVar[i] = ui->tblKalmanVar->item(1,i)->text().toDouble();
	}
	for(int i=0; i<3; i++)
	{
		mKalmanAttBias[i] = ui->tblKalmanAttBias->item(0,i)->text().toDouble();
		mKalmanAttBiasAdaptGain[i] = ui->tblKalmanAttBias->item(1,i)->text().toDouble();
	}

	mKalmanForceGainAdaptGain = ui->txtKalmanForceGainAdaptGain->text().toDouble();

	mAttObsvGainP = ui->tblAttObsvGains->item(0,0)->text().toDouble();
	mAttObsvGainI = ui->tblAttObsvGains->item(0,1)->text().toDouble();

	for(int i=0; i<3; i++)
		mAttObsvNominalMag[i] = ui->tblAttObsvNomMag->item(0,i)->text().toDouble();

	for(int i=0; i<2; i++)
		mAttObsvDirWeights[i] = ui->tblAttObsvDirWeights->item(0,i)->text().toDouble();
}

void Leash::applyHardwareConfig()
{
	mMotorForceGain = ui->txtMotorForceGain->text().toDouble();
	mMotorTorqueGain = ui->txtMotorTorqueGain->text().toDouble();
	mMotorArmLength = ui->txtMotorArmLength->text().toDouble();
	mTotalMass = ui->txtTotalMass->text().toDouble();
}

void Leash::onBtnResetConfig_clicked()
{
	populateUI();
}

void Leash::onBtnLoadConfig_clicked()
{
	QFileDialog fileDialog(this,"Open config","../","Rover leash config files (*.leashConfig)");
	fileDialog.setDefaultSuffix("leashConfig");
	fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
	string filename;
	if(fileDialog.exec())
		filename = fileDialog.selectedFiles().front().toStdString();
	else
		return;

	loadConfigFromFile(filename);
}

void Leash::onBtnSaveConfig_clicked()
{
	QFileDialog fileDialog(this,"Save config","../","Rover leash config files (*.leashConfig)");
	fileDialog.setDefaultSuffix("leashConfig");
	fileDialog.setAcceptMode(QFileDialog::AcceptSave);
	string filename;
	if(fileDialog.exec())
		filename = fileDialog.selectedFiles().front().toStdString();
	else
		return;

	saveConfigToFile(filename);
}

void Leash::onBtnConnect_clicked()
{
	if(ui->btnConnect->text() == "Connect")
	{
		mIP = ui->txtIP->toPlainText().toStdString();
		mPort = ui->txtPort->toPlainText().toInt();
		cout << "Connecting to phone at " << mIP << ":" << mPort << " ... ";
//		lblPhoneStatus->setText("Connecting ...");
		try
		{
			if(mSocketTCP == NULL)
				mSocketTCP = Socket::ptr(Socket::createTCPSocket());
			mSocketTCP->connect(mIP.c_str(),mPort);
			if(mSocketTCP == NULL)
				throw(Exception(" connection error"));
			if(mSocketUDP != NULL)
				mSocketUDP->close();
			mSocketUDP = Socket::ptr(Socket::createUDPSocket());
			mSocketUDP->bind(mPort);
			cout << "done." << endl;
			ui->btnConnect->setText("Disconnect");

			sendParams();
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onPhoneConnected();
		}
		catch (...)
		{ 
			if(mSocketTCP != NULL)
				mSocketTCP->close();
			if(mSocketUDP != NULL)
				mSocketUDP->close();
			mSocketTCP = NULL;
			mSocketUDP = NULL;
			cout << "Failed to connect" << endl; 
		}

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
		ui->btnConnect->setText("Connect");
	}		
}

void Leash::onBtnSendParams_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send motor start because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	sendParams();
}

void Leash::onBtnResetObserver_clicked()
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

void Leash::onBtnSyncTime_clicked()
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

void Leash::onBtnRequestLogFile_clicked()
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

// void Leash::onBtnSendMuCntl_clicked()
// {
// 	if(mSocketTCP == NULL)
// 	{
// 		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send controller because I'm currently not connected to the phone.");
// 		box.exec();
// 		return;
// 	}
// 	int code = COMM_SEND_CNTL_SYSTEM;
// 
// 	cout << "Sending mu controller" << endl;
// 	try
// 	{
// 		SystemModelLinear sys;
// 		sys.loadFromFile(mCntlSysFile.c_str());
// 		cout << "System loaded from " << mCntlSysFile << endl;
// 		vector<tbyte> buff;
// 		sys.serialize(buff);
// 		cout << buff.size() << " bytes serialized" << endl;
// 		uint32 size = (uint32)buff.size();
// 		sendTCP((tbyte*)&code, sizeof(code));
// 		sendTCP((tbyte*)&size, sizeof(size));
// 		sendTCP(&(buff[0]), size);
// 	}
// 	catch(exception e)
// 	{
// 		QMessageBox box(QMessageBox::Warning, "Send Error", QString("Failed to send mu controller. ")+e.what());
// 		box.exec();
// 	}
// }

void Leash::onBtnClearLog_clicked()
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

// void Leash::onChkUseMuCntl_clicked()
// {
// 	if(mSocketTCP == NULL)
// 	{
// 		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't enable mu controller because I'm currently not connected to the phone.");
// 		box.exec();
// 		return;
// 	}
// 
// 	int code = COMM_CNTL_TYPE;
// //	uint16 cntlType = chkUseMuCntl->isChecked() ? CNTL_TRANSLATION_SYS : CNTL_TRANSLATION_PID;
// 	uint16 cntlType = CNTL_TRANSLATION_PID;
// 
// 	cout << "Setting control type" << endl;
// 	sendTCP((tbyte*)&code, sizeof(code));
// 	sendTCP((tbyte*)&cntlType, sizeof(cntlType));
// }

void Leash::onChkViewBinarizedImage_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't change view because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	int code = COMM_IMGVIEW_TYPE;
//	uint16 viewType = chkViewBinarizedImage->isChecked() ? 1 : 0;
	uint16 viewType = 0;

	cout << "Setting image view type" << endl;
	sendTCP((tbyte*)&code, sizeof(code));
	sendTCP((tbyte*)&viewType, sizeof(viewType));
}

void Leash::onChkUseIbvsController_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't tell phone to use IBVS controller because I'm currently not connected to the phone.");
		box.exec();
		return;
	}

	int code = COMM_USE_IBVS;
//	uint16 useIbvs = chkUseIbvsController->isChecked() ? 1 : 0;
	uint16 useIbvs = 0;

	cout << "Setting IBVS usage to " << useIbvs << endl;
	sendTCP((tbyte*)&code, sizeof(code));
	sendTCP((tbyte*)&useIbvs, sizeof(useIbvs));
}

void Leash::onBtnResetDesImgMoment_clicked()
{
//	for(int i=0; i<2; i++)
//		tblImgState->item(1,i)->setText(QString::number(mDesiredStateImage[i],'f',3));
}

void Leash::onBtnConfirmDesImgMoment_clicked()
{
//	mDesiredStateImage[0] = tblImgState->item(1,0)->text().toDouble();
//	mDesiredStateImage[1] = tblImgState->item(1,1)->text().toDouble();
//	mDesiredStateImage[2] = tblImgState->item(1,2)->text().toDouble();
}

void Leash::onBtnSetYawZero_clicked()
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
void Leash::applyCommConfig(QTreeWidgetItem *root)
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

void Leash::applyControlConfig(QTreeWidgetItem *root)
{
//	while(root->childCount() > 0)
//	{
//		QTreeWidgetItem *item = root->takeChild(0);
//		if(item->text(0) == "P")
//		{
//			mGainP[0] = item->text(1).toDouble();
//			mGainP[1] = item->text(2).toDouble();
//			mGainP[2] = item->text(3).toDouble();
//		}
//		else if(item->text(0) == "I")
//		{
//			mGainI[0] = item->text(1).toDouble();
//			mGainI[1] = item->text(2).toDouble();
//			mGainI[2] = item->text(3).toDouble();
//		}
//		else if(item->text(0) == "D")
//		{
//			mGainD[0] = item->text(1).toDouble();
//			mGainD[1] = item->text(2).toDouble();
//			mGainD[2] = item->text(3).toDouble();
//		}
//		else
//		{
//			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone control config item: " + item->text(0));
//			box.exec();
//			throw("Unknown communication config item: " + item->text(0));
//		}
//	}
}

void Leash::applyMotorConfig(QTreeWidgetItem *root)
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

// void Leash::applyObserverConfig(QTreeWidgetItem *root)
// {
// 	while(root->childCount() > 0)
// 	{
// 		QTreeWidgetItem *item = root->takeChild(0);
// 		if(item->text(0) == "Kp")
// 			mAttObsvGainP = item->text(1).toDouble();
// 		else if(item->text(0) == "Ki")
// 			mAttObsvGainI = item->text(1).toDouble();
// 		else if(item->text(0) == "Accel Weight")
// 			mAttObsvDirWeights[0] = item->text(1).toDouble();
// 		else if(item->text(0) == "Mag Weight")
// 			mAttObsvDirWeights[1] = item->text(1).toDouble();
// 		else
// 		{
// 			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone motor config item: " + item->text(0));
// 			box.exec();
// 			throw("Unknown communication config item: " + item->text(0));
// 		}
// 	}
// }

void Leash::applyIbvsConfig(QTreeWidgetItem *root)
{
	while(root->childCount() > 0)
	{
		QTreeWidgetItem *item = root->takeChild(0);
//		if(item->text(0) == "Img")
//		{
//			mIbvsGainImg[0] = item->text(1).toDouble();
//		 	mIbvsGainImg[1] = item->text(2).toDouble();
//			mIbvsGainImg[2] = item->text(3).toDouble();
//		}
//		else if(item->text(0) == "Flow") 	
//		{
//			mIbvsGainFlow[0] = item->text(1).toDouble();
//			mIbvsGainFlow[1] = item->text(2).toDouble();
//			mIbvsGainFlow[2] = item->text(3).toDouble();
//		}
//		else if(item->text(0) == "Flow Int") 	
//		{
//			mIbvsGainFlowInt[0] = item->text(1).toDouble();
//			mIbvsGainFlowInt[1] = item->text(2).toDouble();
//			mIbvsGainFlowInt[2] = item->text(3).toDouble();
//		}
//		else if(item->text(0) == "Feedforward")
//		{
//			mIbvsGainFF[0] = item->text(1).toDouble();
//			mIbvsGainFF[1] = item->text(2).toDouble();
//			mIbvsGainFF[2] = item->text(3).toDouble();
//		}
//		else if(item->text(0) == "Att Cmd Offset")
//		{
//			mAttCmdOffset[0] = item->text(1).toDouble();
//			mAttCmdOffset[1] = item->text(2).toDouble();
//			mAttCmdOffset[2] = item->text(3).toDouble();
//		}
		if(item->text(0) == "omega") 	
		{
			mIbvsGainAngularRate[0] = item->text(1).toDouble();
			mIbvsGainAngularRate[1] = item->text(2).toDouble();
			mIbvsGainAngularRate[2] = item->text(3).toDouble();
		}
		else if(item->text(0) == "angle") 	mIbvsGainAngle = item->text(1).toDouble();
//		else if(item->text(0) == "dynamic") mIbvsGainDynamic = item->text(1).toDouble();
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone IBVS config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void Leash::applyKalmanFilterConfig(QTreeWidgetItem *root)
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
//		else if(item->text(0) == "Att Bias Gain")
//			mAttBiasGain = item->text(1).toDouble();
		else if(item->text(0) == "Force Scaling Gain")
			mKalmanForceGainAdaptGain= item->text(1).toDouble();
		else
		{
			QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown phone Kalman Filter config item: " + item->text(0));
			box.exec();
			throw("Unknown communication config item: " + item->text(0));
		}
	}
}

void Leash::applyLogConfig(QTreeWidgetItem *root)
{
	mLogMask = 0;
	for(int i=0; i<root->childCount(); i++)
		mLogMask |= root->child(i)->checkState(1) == Qt::Checked ? 1 << i : 0;
}

void Leash::populateUI()
{
	populateControlUI();
	populateObserverUI();
	populateHardwareUI();
	return;

	mMutex_data.lock();
//	while(treePhoneConfig->topLevelItemCount() > 0)
//		treePhoneConfig->takeTopLevelItem(0);

//	QStringList headers;
//	headers << "1" << "2" << "3" << "4" << "5" << "6";
//	treePhoneConfig->setHeaderItem(new QTreeWidgetItem((QTreeWidget*)0,headers));

	QStringList commHeadings;
	commHeadings << "Communication" << "";
	QTreeWidgetItem *commRoot = new QTreeWidgetItem((QTreeWidget*)0,commHeadings);
		commRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("IP")))); 
		commRoot->child(commRoot->childCount()-1)->setText(1,QString(mIP.c_str()));
		commRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Port")))); 
		commRoot->child(commRoot->childCount()-1)->setText(1,QString::number(mPort));

//	QStringList phoneCntlHeadings;
//	phoneCntlHeadings << "Controller" << "Roll" << "Pitch" << "Yaw";		
//	QTreeWidgetItem *cntlRoot = new QTreeWidgetItem((QTreeWidget*)0,phoneCntlHeadings);
//		cntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("P"))));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(1,QString::number(mGainP[0]));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(2,QString::number(mGainP[1]));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(3,QString::number(mGainP[2]));
//		cntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("I"))));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(1,QString::number(mGainI[0]));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(2,QString::number(mGainI[1]));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(3,QString::number(mGainI[2]));
//		cntlRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("D"))));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(1,QString::number(mGainD[0]));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(2,QString::number(mGainD[1]));
//			cntlRoot->child(cntlRoot->childCount()-1)->setText(3,QString::number(mGainD[2]));

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
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mAttObsvGainP));
		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Ki"))));
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mAttObsvGainI));
		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Accel Weight"))));
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mAttObsvDirWeights[0]));
		observerRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Mag Weight"))));
			observerRoot->child(observerRoot->childCount()-1)->setText(1,QString::number(mAttObsvDirWeights[1]));

	QStringList ibvsHeadings; ibvsHeadings << "IBVS Gains" << "x" << "y" << "z";
	QTreeWidgetItem *ibvsRoot = new QTreeWidgetItem((QTreeWidget*)0,ibvsHeadings);
//		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Img"))));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainImg[0]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainImg[1]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainImg[2]));
//		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Flow"))));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainFlow[0]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainFlow[1]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainFlow[2]));
//		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Flow Int"))));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainFlowInt[0]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainFlowInt[1]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainFlowInt[2]));
//		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Feedforward"))));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainFF[0]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainFF[1]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainFF[2]));
//		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Att Cmd Offset"))));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mAttCmdOffset[0]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mAttCmdOffset[1]));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mAttCmdOffset[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("omega"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainAngularRate[0]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(2,QString::number(mIbvsGainAngularRate[1]));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(3,QString::number(mIbvsGainAngularRate[2]));
		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("angle"))));
			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainAngle));
//		ibvsRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("dynamic"))));
//			ibvsRoot->child(ibvsRoot->childCount()-1)->setText(1,QString::number(mIbvsGainDynamic));

	// Kalman Filter
	QStringList kfHeadings;
	kfHeadings << "Kalman Filter";
	QTreeWidgetItem *kfRoot = new QTreeWidgetItem((QTreeWidget*)0,kfHeadings);
		kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Att Bias"))));
			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mAttBias[0][0]));
			kfRoot->child(kfRoot->childCount()-1)->setText(2,QString::number(mAttBias[1][0]));
			kfRoot->child(kfRoot->childCount()-1)->setText(3,QString::number(mAttBias[2][0]));
//		kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Att Bias Gain"))));
//			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mAttBiasGain));
		kfRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Force Scaling Gain"))));
			kfRoot->child(kfRoot->childCount()-1)->setText(1,QString::number(mKalmanForceGainAdaptGain));

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
}

void Leash::populateControlUI()
{
	for(int i=0; i<3; i++)
	{
		ui->tblTransCntl->item(i,0)->setText(QString::number(mCntlGainTransP[i]));
		ui->tblTransCntl->item(i,1)->setText(QString::number(mCntlGainTransD[i]));
		ui->tblTransCntl->item(i,2)->setText(QString::number(mCntlGainTransI[i]));
		ui->tblTransCntl->item(i,3)->setText(QString::number(mCntlGainTransILimit[i]));
		ui->tblAttCntl->item(i,0)->setText(QString::number(mCntlGainAttP[i]));
		ui->tblAttCntl->item(i,1)->setText(QString::number(mCntlGainAttD[i]));
	}

	resizeTableWidget(ui->tblTransCntl);
	resizeTableWidget(ui->tblAttCntl);
}

void Leash::populateObserverUI()
{
	for(int i=0; i<6; i++)
	{
		ui->tblKalmanVar->item(0,i)->setText(QString::number(mKalmanMeasVar[i]));
		ui->tblKalmanVar->item(1,i)->setText(QString::number(mKalmanDynVar[i]));
	}

	for(int i=0; i<3; i++)
	{
		ui->tblKalmanAttBias->item(0,i)->setText(QString::number(mKalmanAttBias[i]));
		ui->tblKalmanAttBias->item(1,i)->setText(QString::number(mKalmanAttBiasAdaptGain[i]));
	}

	ui->txtKalmanForceGainAdaptGain->setText(QString::number(mKalmanForceGainAdaptGain));
	
	ui->tblAttObsvGains->item(0,0)->setText(QString::number(mAttObsvGainP));
	ui->tblAttObsvGains->item(0,1)->setText(QString::number(mAttObsvGainI));

	for(int i=0; i<3; i++)
		ui->tblAttObsvNomMag->item(0,i)->setText(QString::number(mAttObsvNominalMag[i]));
	for(int i=0; i<2; i++)
		ui->tblAttObsvDirWeights->item(0,i)->setText(QString::number(mAttObsvDirWeights[i]));

	resizeTableWidget(ui->tblKalmanVar);
	resizeTableWidget(ui->tblKalmanAttBias);
	resizeTableWidget(ui->tblAttObsvGains);
	resizeTableWidget(ui->tblAttObsvNomMag);
	resizeTableWidget(ui->tblAttObsvDirWeights);
}

void Leash::populateHardwareUI()
{
	ui->txtMotorForceGain->setText(QString::number(mMotorForceGain));
	ui->txtMotorTorqueGain->setText(QString::number(mMotorTorqueGain));
	ui->txtMotorArmLength->setText(QString::number(mMotorArmLength));
	ui->txtTotalMass->setText(QString::number(mTotalMass));
}

void Leash::formatTree(QTreeWidgetItem *root)
{
	root->setFlags(root->flags() | Qt::ItemIsEditable);
	for(int i=1; i<root->columnCount(); i++)
		root->setTextAlignment(i,Qt::AlignHCenter);

	if(root->childCount() > 0)
		for(int i=0; i<root->childCount(); i++)
			formatTree(root->child(i));
}

void Leash::toggleIbvs()
{
//	chkUseIbvsController->setChecked(!chkUseIbvsController->isChecked());
//	onChkUseIbvsController_clicked();
}

int Leash::receiveTCP(Socket::ptr socket, tbyte* data, int size)
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

int Leash::receiveUDP(Socket::ptr socket, tbyte* data, int size)
{
	uint32 addr;
	int port;
	int received = socket->receiveFrom(data, size, addr, port);
	
	return received;
}

bool Leash::receivePacket(Socket::ptr socket, Packet &pck, int size)
{
	Collection<tbyte> buff(size);

	int result = receiveUDP(socket, buff, size);
	if(result > 0)
		pck.deserialize(buff);

	if(pck.size != result)
		cout << "Only received " << result << " vs " << pck.size << endl;;

	return pck.size == result;
}

void Leash::receiveLogFile(Socket::ptr socket, string filename)
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

void Leash::resizeTableWidget(QTableWidget *tbl)
{
	int totWidth = 0;
	int totHeight = 0;
	totWidth = tbl->horizontalHeader()->length();
	totWidth += tbl->verticalHeader()->width();

	totHeight = tbl->verticalHeader()->length();
	totHeight += tbl->horizontalHeader()->height();
	tbl->setMaximumSize(totWidth, totHeight);
	tbl->setMinimumSize(totWidth, totHeight);
	tbl->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	tbl->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}

void Leash::setVerticalTabOrder(QTableWidget *tbl)
{
	for(int j=0; j<tbl->columnCount(); j++)
	{
		for(int i=0; i<tbl->rowCount()-1; i++)
			QWidget::setTabOrder(tbl->cellWidget(i,j), tbl->cellWidget(i+1,j));
		if(j != tbl->columnCount()-1)
			QWidget::setTabOrder(tbl->cellWidget(tbl->rowCount()-1,j), tbl->cellWidget(0,j+1));
		else
			QWidget::setTabOrder(tbl->cellWidget(tbl->rowCount()-1,j), tbl->cellWidget(0,0));
	}
}

}
}
