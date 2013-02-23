#include "Leash.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

Leash::Leash(QWidget *parent) : 
	QMainWindow(parent),
	ui(new Ui::Leash),
	mAttBias(3,1,0.0),
	mIntMemory(3,1,0.0),
	mAttObsvDirWeights(2,0.5),
	mAttObsvNominalMag(3,0),
	mKalmanMeasVar(6,1),
	mKalmanDynVar(6,0),
	mKalmanAttBias(3,0),
	mKalmanAttBiasAdaptRate(3,0),
	mState(12,1,0.0),
	mDesState(12,1,0.0),
	mViconState(12,1,0.0)
{
	ui->setupUi(this);
	mTmrGui = new QTimer(this);

	QSettings settings("ICSL", "QuadRover Leash");
	restoreGeometry(settings.value("geometry").toByteArray());
	int uiVersion= 1;
	restoreState(settings.value("state").toByteArray(),uiVersion);

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

	mLogMask = PC_UPDATES;

//	mUseMotors = false;
	mUseIbvs = false;

	mKalmanForceGainAdaptRate = 0;

	mMotorForceGain = 0;
	mMotorTorqueGain = 0;
	mMotorArmLength = 1;
	mTotalMass = 1;

	mNewViconDataReady = false;

	mFirstDraw = true;
}

Leash::~Leash()
{
	if(mSocketTCP != NULL)
		mSocketTCP->close();
	if(mSocketUDP != NULL)
		mSocketUDP->close();

	QSettings settings("ICSL", "QuadRover Leash");
	settings.setValue("geometry", saveGeometry());
	settings.setValue("state", saveState(1));

	mTelemVicon.stopMonitor();
	mTelemVicon.disconnect();

	if(mTmrGui != NULL)
		delete mTmrGui;

	delete mScStartMotors;
	delete mScStopMotors;
	delete mScQuit;

	delete mScIncreaseHeight;
	delete mScDecreaseHeight;
	delete mScMoveLeft;
	delete mScMoveRight;
	delete mScMoveForward;
	delete mScMoveBackward;
	delete mScToggleIbvs;
}

void Leash::initialize()
{
	connect(mTmrGui, SIGNAL(timeout()), this, SLOT(updateDisplay()));
	connect(ui->btnStartMotors, SIGNAL(clicked()), this, SLOT(onBtnStartMotors_clicked()));
	connect(ui->btnStopMotors,  SIGNAL(clicked()), this, SLOT(onBtnStopMotors_clicked()));
	connect(ui->btnQuit, SIGNAL(clicked()), this, SLOT(onBtnQuit_clicked()));
	connect(ui->btnLoadConfig,SIGNAL(clicked()),this,SLOT(onBtnLoadConfig_clicked()));
	connect(ui->btnSaveConfig,SIGNAL(clicked()),this,SLOT(onBtnSaveConfig_clicked()));
	connect(ui->btnResetConfig,SIGNAL(clicked()),this,SLOT(onBtnResetConfig_clicked()));
	connect(ui->btnApply,SIGNAL(clicked()),this,SLOT(onBtnApply_clicked()));
	connect(ui->btnConnect,SIGNAL(clicked()),this,SLOT(onBtnConnect_clicked()));
	connect(ui->btnClearPhoneLog,SIGNAL(clicked()),this,SLOT(onBtnClearPhoneLog_clicked()));
	connect(ui->btnClearLocalLog,SIGNAL(clicked()),this,SLOT(onBtnClearLocalLog_clicked()));
	connect(ui->btnGetPhoneLog,SIGNAL(clicked()),this,SLOT(onBtnGetPhoneLog_clicked()));
	connect(ui->btnSendParams,SIGNAL(clicked()),this,SLOT(onBtnSendParams_clicked()));
	connect(ui->btnSyncTime,SIGNAL(clicked()),this,SLOT(onBtnSyncTime_clicked()));

	mScStartMotors = new QShortcut(Qt::Key_W, this);
	mScStopMotors = new QShortcut(Qt::Key_Space, this);
	mScQuit = new QShortcut(Qt::Key_Q, this);
	connect(mScStartMotors,SIGNAL(activated()), this, SLOT(onBtnStartMotors_clicked()));
	connect(mScStopMotors,SIGNAL(activated()), this, SLOT(onBtnStopMotors_clicked()));
	connect(mScQuit,SIGNAL(activated()), this, SLOT(onBtnQuit_clicked()));

	mScIncreaseHeight = new QShortcut(Qt::Key_9, this);
	mScDecreaseHeight = new QShortcut(Qt::Key_7, this);
	mScMoveLeft = new QShortcut(Qt::Key_4, this);
	mScMoveRight = new QShortcut(Qt::Key_6, this);
	mScMoveForward = new QShortcut(Qt::Key_8, this);
	mScMoveBackward = new QShortcut(Qt::Key_2, this);
	connect(mScIncreaseHeight,SIGNAL(activated()), this, SLOT(onIncreaseHeight()));
	connect(mScDecreaseHeight,SIGNAL(activated()), this, SLOT(onDecreaseHeight()));
	connect(mScMoveLeft,SIGNAL(activated()), this, SLOT(onMoveLeft()));
	connect(mScMoveRight,SIGNAL(activated()), this, SLOT(onMoveRight()));
	connect(mScMoveForward,SIGNAL(activated()), this, SLOT(onMoveForward()));
	connect(mScMoveBackward,SIGNAL(activated()), this, SLOT(onMoveBackward()));

	mScToggleIbvs = new QShortcut(Qt::Key_V, this);

	loadConfigFromFile("../quad0.leashConfig");

	cout << "Connecting to Vicon ... ";
	try
	{
		mTelemVicon.setOriginPosition(Array2D<double>(3,1,0.0));
		mTelemVicon.initializeMonitor();
//		mTelemVicon.connect("147.46.243.133");
//		mTelemVicon.connect("192.168.100.108");
		mTelemVicon.connect("localhost");
	}
	catch(const TelemetryViconException& ex)	{ cout << "Failure" << endl; throw(ex); }
	cout << "Success" << endl;
	mTelemVicon.addTrackedQuadrotor("quadMikroPhone");
	mTelemVicon.addListener(this);

	mSocketTCP = NULL;
	mSocketUDP = NULL;

	// Views for onboard data
	vector<QList<QStandardItem*>* > data;
	data.push_back(&mGyroData);
	data.push_back(&mGyroBiasData);
	data.push_back(&mAccelData);
	data.push_back(&mMagData);
	data.push_back(&mPosIntData);
	data.push_back(&mTorqueIntData);
	vector<QTableView**> views;
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
			data[mdl]->push_back(new QStandardItem(QString("-000.0000")));
			data[mdl]->back()->setData(Qt::AlignCenter, Qt::TextAlignmentRole);
		}
  		QStandardItemModel *model = new QStandardItemModel(this);
		model->appendRow(*(data[mdl]));
		if(data[mdl] == &mTorqueIntData)
			model->setHorizontalHeaderLabels(attLabels);
		else
			model->setHorizontalHeaderLabels(xyzLabels);
		(*views[mdl])->setModel(model);

		(*views[mdl])->resizeColumnsToContents();
		(*views[mdl])->resizeRowsToContents();
		(*views[mdl])->verticalHeader()->hide();
		resizeTable(*views[mdl]);
	}

	data.clear();
	data.push_back(&mStateData);
	data.push_back(&mDesStateData);
	data.push_back(&mViconStateData);
	views.clear();
	views.push_back(&ui->vwState);
	views.push_back(&ui->vwDesState);
	views.push_back(&ui->vwViconState);
	string prefix[] = {"Cur ", "Des ", "Vicon "};
	for(int mdl=0; mdl<data.size(); mdl++)
	{
		QStandardItemModel *model= new QStandardItemModel(this);
		for(int i=0; i<4; i++)
		{
			QList<QStandardItem*> row;
			for(int j=0; j<3; j++)
			{
				data[mdl]->push_back(new QStandardItem(QString("-00.0000")));
				data[mdl]->back()->setData(Qt::AlignCenter, Qt::TextAlignmentRole);
				row.push_back(data[mdl]->back());
			}
			model->appendRow(row);
		}
		QStringList colHeadings, rowHeadings;
		colHeadings << "x" << "y" << "z";
		rowHeadings << (prefix[mdl]+"Att").c_str() << (prefix[mdl]+"Att Rate").c_str() << ( prefix[mdl]+"Pos").c_str() << (prefix[mdl]+"Vel").c_str();
		model->setHorizontalHeaderLabels(colHeadings);
		model->setVerticalHeaderLabels(rowHeadings);
		(*views[mdl])->setModel(model);
		(*views[mdl])->resizeColumnsToContents();
		(*views[mdl])->resizeRowsToContents();
		(*views[mdl])->horizontalHeader()->setDefaultAlignment(Qt::AlignHCenter);
		(*views[mdl])->verticalHeader()->setDefaultAlignment(Qt::AlignRight);
		(*views[mdl])->setRowHidden(1,true);
		if(data[mdl] == &mViconStateData)
			(*views[mdl])->setRowHidden(3,true);
		resizeTable(*views[mdl]);
	}

	for(int i=0; i<4; i++)
	{
		mMotorData.push_back(new QStandardItem(QString("000000")));
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
	ui->vwMotors->horizontalHeader()->show();
	ui->vwMotors->verticalHeader()->hide();
	resizeTable(ui->vwMotors);

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

	for(int i=0; i<4; i++)
		ui->tblMotorTrim->item(0,i)->setText("00000");
	tables.push_back(ui->tblMotorTrim);

	for(int tbl=0; tbl<tables.size(); tbl++)
	{
		tables[tbl]->resizeColumnsToContents();
		tables[tbl]->resizeRowsToContents();
		setVerticalTabOrder(tables[tbl]);
	}

	cv::Mat img(240,320,CV_8UC3,cv::Scalar(0));
	ui->lblImageDisplay->setPixmap(QPixmap::fromImage(cvMat2QImage(img)));
	ui->lblImageDisplay->setMaximumSize(img.size().width, img.size().height);
	ui->lblImageDisplay->setMinimumSize(img.size().width, img.size().height);

	// need to draw everything before tables get properly resized
	for(int i=0; i<ui->tabWidget->count(); i++)
	{
		ui->tabWidget->setCurrentIndex(i);
		populateUI();
	}
	ui->tabWidget->setCurrentIndex(0);

	resizeTable(ui->vwGyro);
	resizeTable(ui->vwGyroBias);
	resizeTable(ui->vwAccel);
	resizeTable(ui->vwMag);
	resizeTable(ui->vwPosInt);
	resizeTable(ui->vwTorqueInt);
	resizeTable(ui->vwMotors);
	resizeTable(ui->vwState);
	resizeTable(ui->vwDesState);
	resizeTable(ui->vwViconState);

	setVerticalTabOrder(ui->tblTransCntl);

	populateUI();
}

void Leash::shutdown()
{
	sendMotorStop(false);
	saveLogData("../data","data0.log");

	if(mSocketTCP != NULL)
	{
		int code = COMM_CLIENT_EXIT;
		sendTCP((tbyte*)&code, sizeof(code));
	}
}

void Leash::run()
{
	mTelemVicon.startMonitor();
	mStartTimeUniverseMS = mSys.mtime();
	mTmrGui->start(50);
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
						for(int i=0; i<pck.dataFloat.size(); i++)
						{
							mStateData[i]->setData(QString::number(pck.dataFloat[i],'f',3), Qt::DisplayRole);
							mState[i][0] = pck.dataFloat[i];
						}
						break;
					case COMM_DESIRED_STATE:
						for(int i=0; i<12; i++)
						{
							mDesStateData[i]->setData(QString::number(pck.dataFloat[i],'f',3), Qt::DisplayRole);
							mDesState[i][0] = pck.dataFloat[i];
						}
						break;
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
					case COMM_INT_MEM_POS:
						for(int i=0; i<3; i++)
							mPosIntData[i]->setData(QString::number(pck.dataFloat[i],'f',3), Qt::DisplayRole);
						break;
					case COMM_INT_MEM_TORQUE:
						for(int i=0; i<3; i++)
							mTorqueIntData[i]->setData(QString::number(pck.dataFloat[i],'f',3), Qt::DisplayRole);
						break;
					case COMM_IMGPROC_TIME_US:
						ui->lblImgProcTime->setText(QString::number(pck.dataInt32[0]/1.0e3,'f',0));
						break;
					case COMM_USE_IBVS:
						if(pck.dataBool[0])
							ui->lblIbvsStatus->setText("On");
						else
							ui->lblIbvsStatus->setText("Off");
						break;
					case COMM_HOST_TIME_MS:
						{
							float time = pck.dataInt32[0]/1.0e3;
							ui->lblHostTime->setText(QString::number(time,'f',0));
						}
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
					case COMM_LOG_FILE_DATA:
						receiveLogFile(mSocketTCP,"../runData/phoneLog.txt");
						break;
					case COMM_IMG_DATA:
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
					case COMM_HOST_EXIT:
						cout << "Host exiting, closing connection" << endl;
						mSocketTCP->close();
						mSocketTCP = NULL;
						mMutex_socketUDP.lock();
						mSocketUDP->close();
						mSocketUDP = NULL;
						mMutex_socketUDP.unlock();
						ui->btnConnect->setText("Connect");
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
	ui->lblRunTime->setText(QString::number(time,'f',0));

	if(mFirstDraw)
	{
		// HACK because widgets don't have the right size yet during startup
		populateUI();

		ui->tabWidget->setCurrentIndex(2);
		resizeTable(ui->vwGyro);
		resizeTable(ui->vwGyroBias);
		resizeTable(ui->vwAccel);
		resizeTable(ui->vwMag);
		resizeTable(ui->vwPosInt);
		resizeTable(ui->vwTorqueInt);
		resizeTable(ui->vwMotors);
		resizeTable(ui->vwState);
		resizeTable(ui->vwDesState);
		resizeTable(ui->vwViconState);

		mFirstDraw = false;
	}

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

bool Leash::sendMotorStop(bool warnIfDisconnected)
{
	if(mSocketTCP == NULL)
	{
		if(warnIfDisconnected)
		{
			QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send motor start because I'm currently not connected to the phone.");
			box.exec();
		}
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

	code = COMM_ATT_OBSV_GAIN;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result =result && sendTCP((tbyte*)&mAttObsvGainP,sizeof(mAttObsvGainP));
	if(result) result =result && sendTCP((tbyte*)&mAttObsvGainI,sizeof(mAttObsvGainI));
	if(result) result =result && sendTCP((tbyte*)&(mAttObsvDirWeights[0]), sizeof(mAttObsvDirWeights[0]));
	if(result) result =result && sendTCP((tbyte*)&(mAttObsvDirWeights[1]), sizeof(mAttObsvDirWeights[1]));

	code = COMM_LOG_MASK; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&mLogMask, sizeof(mLogMask));

	float forceGain= mMotorForceGain;
	code = COMM_MOTOR_FORCE_GAIN; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&forceGain,sizeof(forceGain));

	float torqueScale = mMotorTorqueGain;
	code = COMM_MOTOR_TORQUE_GAIN;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&torqueScale,sizeof(torqueScale));

	float m = mTotalMass;
	code = COMM_MASS; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&m,sizeof(m));

	code = COMM_KALMANFILTER_MEAS_VAR;
	int measVarSize = mKalmanMeasVar.size();
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&measVarSize,sizeof(measVarSize));
	if(result) result = result && sendTCP((tbyte*)&(mKalmanMeasVar[0]),measVarSize*sizeof(float));

	code = COMM_KALMANFILTER_DYN_VAR;
	int dynVarSize = mKalmanDynVar.size();
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&dynVarSize,sizeof(dynVarSize));
	if(result) result = result && sendTCP((tbyte*)&(mKalmanDynVar[0]),dynVarSize*sizeof(float));

	Collection<float> attGains;
	for(int i=0; i<3; i++)
		attGains.push_back(mCntlGainAttP[i]);
	for(int i=0; i<3; i++)
		attGains.push_back(mCntlGainAttD[i]);
	int attGainSize = attGains.size();
	code = COMM_CNTL_ATT_GAINS;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&attGainSize,sizeof(attGainSize));
	if(result) result = result && sendTCP((tbyte*)&(attGains[0]),attGainSize*sizeof(float));

	Collection<float> transGains;
	for(int i=0; i<3; i++)
		transGains.push_back(mCntlGainTransP[i]);
	for(int i=0; i<3; i++)
		transGains.push_back(mCntlGainTransD[i]);
	for(int i=0; i<3; i++)
		transGains.push_back(mCntlGainTransI[i]);
	for(int i=0; i<3; i++)
		transGains.push_back(mCntlGainTransILimit[i]);
	int transGainSize = transGains.size();
	code = COMM_CNTL_TRANS_GAINS;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&transGainSize,sizeof(transGainSize));
	if(result) result = result && sendTCP((tbyte*)&(transGains[0]),transGainSize*sizeof(float));

	Collection<float> attBias(mKalmanAttBias);
	int attBiasSize = attBias.size();
	code = COMM_KALMAN_ATT_BIAS; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&attBiasSize,sizeof(attBiasSize));
	if(result) result = result && sendTCP((tbyte*)&(attBias[0]),attBiasSize*sizeof(float));

	Collection<float> attBiasAdaptRate(mKalmanAttBiasAdaptRate);
	int attBiasAdaptRateSize = attBiasAdaptRate.size();
	code = COMM_KALMAN_ATT_BIAS_ADAPT_RATE; 
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&attBiasAdaptRateSize,sizeof(attBiasAdaptRateSize));
	if(result) result = result && sendTCP((tbyte*)&(attBiasAdaptRate[0]),attBiasAdaptRateSize*sizeof(float));

	float forceGainAdaptRate = mKalmanForceGainAdaptRate;
	code = COMM_KALMAN_FORCE_SCALING_ADAPT_RATE;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&forceGainAdaptRate, sizeof(float));

	Collection<float> nomMag(mAttObsvNominalMag);
	int nomMagSize = nomMag.size();
	code = COMM_NOMINAL_MAG;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&nomMagSize,sizeof(nomMagSize));
	if(result) result = result && sendTCP((tbyte*)&(nomMag[0]),nomMagSize*sizeof(float));

	float armLength= mMotorArmLength;
	code = COMM_MOTOR_ARM_LENGTH;
	if(result) result = result && sendTCP((tbyte*)&code,sizeof(code));
	if(result) result = result && sendTCP((tbyte*)&armLength,sizeof(armLength));


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

	mxml_node_t *logRoot = mxmlFindElement(xmlRoot, xmlRoot, "LogMask", NULL, NULL, MXML_DESCEND);
	if(logRoot != NULL) stringstream(logRoot->child->value.text.string) >> mLogMask;

	mxml_node_t *ipRoot = mxmlFindElement(xmlRoot, xmlRoot, "IP", NULL, NULL, MXML_DESCEND);
	if(ipRoot != NULL) mIP = ipRoot->child->value.text.string;

	mxml_node_t *portRoot = mxmlFindElement(xmlRoot, xmlRoot, "Port", NULL, NULL, MXML_DESCEND);
	if(portRoot != NULL) stringstream(portRoot->child->value.text.string) >> mPort;

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
			if(xVelNode != NULL) stringstream(xVelNode->child->value.text.string) >> mKalmanMeasVar[3];
			if(yVelNode != NULL) stringstream(yVelNode->child->value.text.string) >> mKalmanMeasVar[4];
			if(zVelNode != NULL) stringstream(zVelNode->child->value.text.string) >> mKalmanMeasVar[5];
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
			if(xVelNode != NULL) stringstream(xVelNode->child->value.text.string) >> mKalmanDynVar[3];
			if(yVelNode != NULL) stringstream(yVelNode->child->value.text.string) >> mKalmanDynVar[4];
			if(zVelNode != NULL) stringstream(zVelNode->child->value.text.string) >> mKalmanDynVar[5];
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

		mxml_node_t *attBiasAdaptRateNode = mxmlFindElement(transNode, transNode, "AttBiasAdaptRate", NULL, NULL, MXML_DESCEND);
		if(attBiasAdaptRateNode != NULL)
		{
			mxml_node_t *rollNode = mxmlFindElement(attBiasAdaptRateNode, attBiasAdaptRateNode, "roll", NULL, NULL, MXML_DESCEND);
			mxml_node_t *pitchNode = mxmlFindElement(attBiasAdaptRateNode, attBiasAdaptRateNode, "pitch", NULL, NULL, MXML_DESCEND);
			mxml_node_t *yawNode = mxmlFindElement(attBiasAdaptRateNode, attBiasAdaptRateNode, "yaw", NULL, NULL, MXML_DESCEND);

			if(rollNode != NULL) stringstream(rollNode->child->value.text.string) >> mKalmanAttBiasAdaptRate[0];
			if(pitchNode != NULL) stringstream(pitchNode->child->value.text.string) >> mKalmanAttBiasAdaptRate[1];
			if(yawNode != NULL) stringstream(yawNode->child->value.text.string) >> mKalmanAttBiasAdaptRate[2];
		}

		mxml_node_t *forceScalingAdaptRateNode = mxmlFindElement(transNode, transNode, "ForceGainAdaptRate", NULL, NULL, MXML_DESCEND);
		if(forceScalingAdaptRateNode!= NULL) stringstream(forceScalingAdaptRateNode->child->value.text.string) >> mKalmanForceGainAdaptRate;
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

	mxml_node_t *motorTrimNode = mxmlFindElement(hdwRoot, hdwRoot, "MotorTrim", NULL, NULL, MXML_DESCEND);
	if(motorTrimNode != NULL)
	{
		mxml_node_t *nNode = mxmlFindElement(motorTrimNode, motorTrimNode, "North", NULL, NULL, MXML_DESCEND);
		mxml_node_t *eNode = mxmlFindElement(motorTrimNode, motorTrimNode, "East", NULL, NULL, MXML_DESCEND);
		mxml_node_t *sNode = mxmlFindElement(motorTrimNode, motorTrimNode, "South", NULL, NULL, MXML_DESCEND);
		mxml_node_t *wNode = mxmlFindElement(motorTrimNode, motorTrimNode, "West", NULL, NULL, MXML_DESCEND);

		if(nNode != NULL) stringstream(nNode->child->value.text.string) >> mMotorTrim[0];
		if(eNode != NULL) stringstream(eNode->child->value.text.string) >> mMotorTrim[1];
		if(sNode != NULL) stringstream(sNode->child->value.text.string) >> mMotorTrim[2];
		if(wNode != NULL) stringstream(wNode->child->value.text.string) >> mMotorTrim[3];
	}
	mMutex_data.unlock();
}

void Leash::saveConfigToFile(string filename)
{
	mxml_node_t *xmlRoot = mxmlNewXML("1.0");
	mxml_node_t *cntlRoot = mxmlNewElement(xmlRoot,"Controller");
	mxml_node_t *obsvRoot = mxmlNewElement(xmlRoot,"Observer");
	mxml_node_t *hdwRoot = mxmlNewElement(xmlRoot,"Hardware");
	mxml_node_t *logRoot = mxmlNewElement(xmlRoot,"LogMask");
	mxml_node_t *ipRoot = mxmlNewElement(xmlRoot,"IP");
	mxml_node_t *portRoot = mxmlNewElement(xmlRoot,"Port");

	saveControllerConfig(cntlRoot);
	saveObserverConfig(obsvRoot);
	saveHardwareConfig(hdwRoot);

	mxmlNewInteger(logRoot,mLogMask);
	mxmlNewText(ipRoot,0,mIP.c_str());
	mxmlNewInteger(portRoot,mPort);

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
		mxml_node_t *attBiasAdaptRateNode = mxmlNewElement(transNode, "AttBiasAdaptRate");
		for(int i=0; i<3; i++)
		{
			mxmlNewReal(mxmlNewElement(attBiasNode,attLabels[i].c_str()), mKalmanAttBias[i]);
			mxmlNewReal(mxmlNewElement(attBiasAdaptRateNode,attLabels[i].c_str()), mKalmanAttBiasAdaptRate[i]);
		}

		mxmlNewReal(mxmlNewElement(transNode,"ForceGainAdaptRate"), mKalmanForceGainAdaptRate);
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
	mxml_node_t *motorTrimNode = mxmlNewElement(hdwRoot, "MotorTrim");
			mxmlNewReal(mxmlNewElement(motorTrimNode, "North"), mMotorTrim[0]);
			mxmlNewReal(mxmlNewElement(motorTrimNode, "East"),  mMotorTrim[1]);
			mxmlNewReal(mxmlNewElement(motorTrimNode, "South"), mMotorTrim[2]);
			mxmlNewReal(mxmlNewElement(motorTrimNode, "West"),  mMotorTrim[3]);
	mMutex_data.unlock();
}

void Leash::onBtnApply_clicked()
{
	applyControllerConfig();
	applyObserverConfig();
	applyHardwareConfig();
	applyDataLoggingConfig();

	mIP = ui->txtIP->text().toStdString();
	mPort = ui->txtPort->text().toInt();

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
		mKalmanAttBiasAdaptRate[i] = ui->tblKalmanAttBias->item(1,i)->text().toDouble();
	}

	mKalmanForceGainAdaptRate = ui->txtKalmanForceGainAdaptRate->text().toDouble();

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

	for(int i=0; i<4; i++)
		mMotorTrim[i] = ui->tblMotorTrim->item(0,i)->text().toInt();
}

void Leash::applyDataLoggingConfig()
{
	mLogMask = 0;
	if(ui->chkLogState->isChecked()) mLogMask |= STATE;
	if(ui->chkLogDesiredState->isChecked()) mLogMask |= STATE_DES;
	if(ui->chkLogMotorCmds->isChecked()) mLogMask |= MOTORS;
	if(ui->chkLogPcUpdates->isChecked()) mLogMask |= PC_UPDATES;
	if(ui->chkLogObserverUpdates->isChecked()) mLogMask |= OBSV_UPDATE;
	if(ui->chkLogObserverBias->isChecked()) mLogMask |= OBSV_BIAS;
	if(ui->chkLogMagnometer->isChecked()) mLogMask |= MAGNOMETER;
	if(ui->chkLogAccelerometer->isChecked()) mLogMask |= ACCEL;
	if(ui->chkLogGyroscope->isChecked()) mLogMask |= GYRO;
	if(ui->chkLogCameraResults->isChecked()) mLogMask |= CAM_RESULTS;
	if(ui->chkLogCameraImages->isChecked()) mLogMask |= CAM_IMAGES;
	if(ui->chkLogPressure->isChecked()) mLogMask |= PRESSURE;
	if(ui->chkLogPhoneTemperature->isChecked()) mLogMask |= PHONE_TEMP;
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
		mIP = ui->txtIP->text().toStdString();
		mPort = ui->txtPort->text().toInt();
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
			int code = COMM_CLIENT_EXIT;
			sendTCP((tbyte*)&code, sizeof(code));
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

void Leash::onBtnGetPhoneLog_clicked()
{
	if(mSocketTCP == NULL)
	{
		QMessageBox box(QMessageBox::Warning, "Connection Error", "Can't send command because I'm currently not connected to the phone.");
		box.exec();
		return;
	}
	int code = COMM_LOG_FILE_REQUEST;

	sendTCP((tbyte*)&code, sizeof(code));
}

void Leash::onBtnClearPhoneLog_clicked()
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

void Leash::onBtnStartMotors_clicked()
{
	sendParams();
	sendMotorStart();
}

void Leash::onBtnStopMotors_clicked()
{
	sendMotorStop(true);
}

void Leash::onBtnQuit_clicked()
{
	shutdown();
	qApp->quit(); // qApp is a global I think
}

void Leash::onIncreaseHeight()
{
	mDesState[8][0] = min(1.0, mDesState[8][0]+0.050);
	mDesStateData[8]->setData(QString::number(mDesState[8][0],'f',3), Qt::DisplayRole);

	sendDesiredState();
}

void Leash::onDecreaseHeight()
{
	mDesState[8][0] = max(0.0, mDesState[8][0]-0.050);
	mDesStateData[8]->setData(QString::number(mDesState[8][0],'f',3), Qt::DisplayRole);

	sendDesiredState();
}

void Leash::onMoveLeft()
{
	mDesState[6][0] = mDesState[6][0]-0.200;
	mDesStateData[6]->setData(QString::number(mDesState[6][0],'f',3), Qt::DisplayRole);

	sendDesiredState();
}

void Leash::onMoveRight()
{
	mDesState[6][0] = mDesState[6][0]+0.200;
	mDesStateData[6]->setData(QString::number(mDesState[6][0],'f',3), Qt::DisplayRole);

	sendDesiredState();
}

void Leash::onMoveForward()
{
	mDesState[7][0] = mDesState[7][0]+0.200;
	mDesStateData[7]->setData(QString::number(mDesState[7][0],'f',3), Qt::DisplayRole);

	sendDesiredState();
}

void Leash::onMoveBackward()
{
	mDesState[7][0] = mDesState[7][0]-0.200;
	mDesStateData[7]->setData(QString::number(mDesState[7][0],'f',3), Qt::DisplayRole);

	sendDesiredState();
}

void Leash::populateUI()
{
	populateControlUI();
	populateObserverUI();
	populateHardwareUI();
	populateDataLoggingUI();

	ui->txtIP->setText(mIP.c_str());
	ui->txtPort->setText(QString::number(mPort));
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
	setVerticalTabOrder(ui->tblTransCntl);
	setVerticalTabOrder(ui->tblAttCntl);
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
		ui->tblKalmanAttBias->item(1,i)->setText(QString::number(mKalmanAttBiasAdaptRate[i]));
	}

	ui->txtKalmanForceGainAdaptRate->setText(QString::number(mKalmanForceGainAdaptRate));
	
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
	setVerticalTabOrder(ui->tblKalmanVar);
	setVerticalTabOrder(ui->tblKalmanAttBias);
	setVerticalTabOrder(ui->tblAttObsvGains);
	setVerticalTabOrder(ui->tblAttObsvNomMag);
	setVerticalTabOrder(ui->tblAttObsvDirWeights);
}

void Leash::populateHardwareUI()
{
	ui->txtMotorForceGain->setText(QString::number(mMotorForceGain));
	ui->txtMotorTorqueGain->setText(QString::number(mMotorTorqueGain));
	ui->txtMotorArmLength->setText(QString::number(mMotorArmLength));
	ui->txtTotalMass->setText(QString::number(mTotalMass));

	for(int i=0; i<4; i++)
		ui->tblMotorTrim->item(0,i)->setText(QString::number(mMotorTrim[i]));

	resizeTableWidget(ui->tblMotorTrim);
	setVerticalTabOrder(ui->tblMotorTrim);
}

void Leash::populateDataLoggingUI()
{
	ui->chkLogState->setChecked( (mLogMask & STATE) > 0);
	ui->chkLogDesiredState->setChecked( (mLogMask & STATE_DES) > 0);
	ui->chkLogMotorCmds->setChecked( (mLogMask & MOTORS) > 0);
	ui->chkLogPcUpdates->setChecked( (mLogMask & PC_UPDATES) > 0);
	ui->chkLogObserverUpdates->setChecked( (mLogMask & OBSV_UPDATE) > 0);
	ui->chkLogObserverBias->setChecked( (mLogMask & OBSV_BIAS) > 0);
	ui->chkLogMagnometer->setChecked( (mLogMask & MAGNOMETER) > 0);
	ui->chkLogAccelerometer->setChecked( (mLogMask & ACCEL) > 0);
	ui->chkLogGyroscope->setChecked( (mLogMask & GYRO) > 0);
	ui->chkLogCameraResults->setChecked( (mLogMask & CAM_RESULTS) > 0);
	ui->chkLogCameraImages->setChecked( (mLogMask & CAM_IMAGES) > 0);
	ui->chkLogPressure->setChecked( (mLogMask & PRESSURE) > 0);
	ui->chkLogPhoneTemperature->setChecked( (mLogMask & PHONE_TEMP) > 0);
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

void Leash::resizeTable(QTableView *tbl)
{
	int totWidth = 0;
	int totHeight = 0;
	totWidth = tbl->horizontalHeader()->length();
	if(tbl->verticalHeader()->isVisible())
		totWidth += tbl->verticalHeader()->width();

	totHeight = tbl->verticalHeader()->length();
	if(tbl->horizontalHeader()->isVisible())
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

void Leash::onTelemetryUpdated(TelemetryViconDataRecord const &rec)
{
	if(strcmp(rec.objectID.c_str(), "quadMikroPhone")==0)
	{
		mMutex_data.lock();
		mViconStateData[0]->setData(QString::number(rec.roll,'f',3), Qt::DisplayRole);
		mViconStateData[1]->setData(QString::number(rec.pitch,'f',3), Qt::DisplayRole);
		mViconStateData[2]->setData(QString::number(rec.yaw,'f',3), Qt::DisplayRole);
		mViconStateData[3]->setData(QString::number(0,'f',3), Qt::DisplayRole);
		mViconStateData[4]->setData(QString::number(0,'f',3), Qt::DisplayRole);
		mViconStateData[5]->setData(QString::number(0,'f',3), Qt::DisplayRole);
		mViconStateData[6]->setData(QString::number(rec.x,'f',3), Qt::DisplayRole);
		mViconStateData[7]->setData(QString::number(rec.y,'f',3), Qt::DisplayRole);
		mViconStateData[8]->setData(QString::number(rec.z,'f',3), Qt::DisplayRole);
		mViconStateData[9]->setData(QString::number(0,'f',3), Qt::DisplayRole);
		mViconStateData[10]->setData(QString::number(0,'f',3), Qt::DisplayRole);
		mViconStateData[11]->setData(QString::number(0,'f',3), Qt::DisplayRole);

		mViconState[0][0] = rec.roll;
		mViconState[1][0] = rec.pitch;
		mViconState[2][0] = rec.yaw;
		mViconState[6][0] = rec.x;
		mViconState[7][0] = rec.y;
		mViconState[8][0] = rec.z;
		mMutex_data.unlock();
	}

	String s = String();
	for(int i=0; i<12; i++)
		s = s+mViconState[i][0]+"\t";
	mMutex_logBuffer.lock();
	mLogData.push_back(LogItem(mSys.mtime()-mStartTimeUniverseMS, s, LOG_TYPE_VICON_STATE));
	mMutex_logBuffer.unlock();

	Packet pState;
	pState.type = COMM_STATE_VICON;
	pState.time = mSys.mtime()-mStartTimeUniverseMS;
	pState.dataFloat.resize(mViconState.dim1());
	for(int i=0; i<mViconState.dim1(); i++)
		pState.dataFloat[i] = mViconState[i][0];
	Collection<tbyte> buff;
	pState.serialize(buff);
	sendUDP(buff.begin(), buff.size());
}

void Leash::sendDesiredState()
{
	Packet pState;
	pState.type = COMM_SET_DESIRED_STATE;
	pState.time = mSys.mtime()-mStartTimeUniverseMS;
	for(int i=0; i<mDesState.dim1(); i++)
		pState.dataFloat.push_back(mDesState[i][0]);
	// need to tack on desired acceleration
	for(int i=0; i<3; i++)
		pState.dataFloat.push_back(0);
	Collection<tbyte> buff;
	pState.serialize(buff);
	sendUDP(buff.begin(), buff.size());
}

void Leash::saveLogData(string dir, string filename)
{
	fstream dataStream((dir+"/"+filename).c_str(), fstream::out);
	if(dataStream.is_open())
	{
		mMutex_logBuffer.lock();
		list<LogItem>::iterator iter = mLogData.begin();
		while(iter != mLogData.end())
		{
			LogItem *item = (LogItem*)&(*iter);
			dataStream << item->time << "\t" << item->type << "\t" << item->line.c_str() << endl; 
			iter++;
		}
		dataStream.close();
		mMutex_logBuffer.unlock();
	}
}

} // namespace Quadrotor
} // namespace ICSL
