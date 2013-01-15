#include "QuadrotorInterface.h"


namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

	QuadrotorInterface::QuadrotorInterface(QWidget *parent) : 
		QWidget(parent),
		mProfileStartPoint(12,1,0.0)
	{
		setupUi(this);

		mFlightMode = QUAD_IDLE;
		mCommStatus = FREE;

		mInitialized = false;

//		mProfileStartPoint = Array2D<double>(12,1,0.0);

//		mStateBuffer.clear();
		mStateRefBuffer.clear();
		mStateVelRefBuffer.clear();
		mErrorMemorySlidingModeBuffer.clear();
		mControlBuffer.clear();
		mTimeBuffer.clear();
		mFlightModeBuffer.clear();

		mName = "quadGaui5";

		for(int i=0; i<4; i++)
		{
			mChannelMin[i] = -1;
			mChannelMax[i] = 1;
			mChannelZero[i] = 0;
			mChannelPolarity[i] = -1;
		}
		mChannelMax[11] = 600;

		mDynamicModel = new SystemModelQuadrotor();
		int chad=0;
	}

	QuadrotorInterface::~QuadrotorInterface()
	{
		delete mDynamicModel;
		delete mPhoneInterface;
	}

	void QuadrotorInterface::initialize()
	{
		mStartTimeUniverseMS = mSys.mtime();

		connect(btnApply,SIGNAL(clicked()),this,SLOT(onBtnApply_clicked()));
		connect(btnReset,SIGNAL(clicked()),this,SLOT(onBtnReset_clicked()));
		connect(btnLoadFromFile,SIGNAL(clicked()),this,SLOT(onBtnLoadFromFile_clicked()));
		connect(btnSaveToFile,SIGNAL(clicked()),this,SLOT(onBtnSaveToFile_clicked()));
		connect(btnSetXyTarget,SIGNAL(clicked()),this,SLOT(onBtnSetXyTarget_clicked()));
		connect(btnResetIntegrators,SIGNAL(clicked()),this,SLOT(onBtnResetIntegrators_clicked()));		
		connect(rdCntl_DirectionThrust,SIGNAL(clicked()),this,SLOT(onRdCntl_clicked()));
		connect(rdCntl_FeedbackLin,SIGNAL(clicked()),this,SLOT(onRdCntl_clicked()));
		connect(chkUseFilteredStates,SIGNAL(clicked()),this,SLOT(onChkUseFilteredStates_clicked()));

		mDynamicModel->setName(mName);

		mDynamicModel->setMass(0.800);
		double k = 0.8;
		mDynamicModel->setForceScaling(k*0.0081, k*0.0081, k*0.0081, k*0.0081);
		mDynamicModel->setTorqueScaling(0.0010, 0.0010, 0.0010, 0.0010);

		Array2D<double> inertiaMat(3,3,0.0);
		inertiaMat[0][0] = 8e-3; inertiaMat[1][1] = 8e-3; inertiaMat[2][2] = 1.9e-2;
		mDynamicModel->setInertiaMat(inertiaMat);

		Array2D<double> forceDir(3,1,0.0);
		forceDir[2][0] = -1;
		mDynamicModel->setForceDir(forceDir, forceDir, forceDir, forceDir);

		Array2D<double> torqueDirNS(3,1,0.0), torqueDirEW(3,1,0.0);
		torqueDirNS[2][0] = -1; torqueDirEW[2][0] = 1;
		mDynamicModel->setTorqueDir(torqueDirNS, torqueDirEW, torqueDirNS, torqueDirEW);

		Array2D<double> motorPosN(3,1,0.0), motorPosE(3,1,0.0), motorPosS(3,1,0.0), motorPosW(3,1,0.0);
		motorPosN[0][0] = 0.180; motorPosE[1][0] = 0.180; motorPosS[0][0] = -0.180; motorPosW[1][0] = -0.180;
		mDynamicModel->setMotorPos(motorPosN, motorPosE, motorPosS, motorPosW);


		mControllerHierarchical.setSystemModel(mDynamicModel);
		mControllerHierarchical.setIntegratorGain(0);
		mControllerHierarchical.setIntegratorSatLimit(0);
		mControllerHierarchical.setIntegratorPosLimit(0);
		mControllerHierarchical.setDesiredState(Array2D<double>(12,1,0.0));
		mControllerHierarchical.useFilteredStates(false);
		
		mControllerFeedbackLin.setSystemModel(mDynamicModel);
		mControllerFeedbackLin.setIntegratorGain(Array2D<double>(12,1,0.0));
		mControllerFeedbackLin.setIntegratorLimit(Array2D<double>(12,1,0.0));
		mControllerFeedbackLin.setDesiredState(Array2D<double>(12,1,0.0));
		mControllerFeedbackLin.setDesiredAccel(Array2D<double>(3,1,0.0));
		mControllerFeedbackLin.useFilteredStates(false);

		rdCntl_DirectionThrust->click();
//		rdCntl_FeedbackLin->click();

		mPhoneInterface = new PhoneInterface(this);
		mPhoneInterface->initialize();
		mPhoneInterface->setForceScaling(mDynamicModel->getAvgForceScaling());
		mPhoneInterface->setTorqueScaling(mDynamicModel->getAvgTorqueScaling());
		mPhoneInterface->setMass(mDynamicModel->getMass());
		layPhone->addWidget(mPhoneInterface);
		layPhone->addItem(new QSpacerItem(40,20,QSizePolicy::Expanding, QSizePolicy::Minimum));
		mPhoneInterface->addListener(this);


		mInitialized = true;
		populateConfigTree();
	}

	void QuadrotorInterface::setDesiredState(TNT::Array2D<double> const &state)
	{
		mMutex_dataAccess.lock();
		mControllerHierarchical.setDesiredState(state); 
		mControllerFeedbackLin.setDesiredState(state); 
		mMutex_dataAccess.unlock();
	}

	void QuadrotorInterface::setDesiredAccel(Array2D<double> const &accel)
	{
		mMutex_dataAccess.lock();
		mControllerFeedbackLin.setDesiredAccel(accel);
		mMutex_dataAccess.unlock();
	}

	void QuadrotorInterface::updateDisplay()
	{
		// position
		Array2D<double> curState;
		if(chkUseFilteredStates->isChecked())
			curState = mDynamicModel->getCurStateFiltered();
		else
			curState = mDynamicModel->getCurState();
		for(int i=0; i<3; i++)
		{
			tblState->item(i,1)->setText(QString::number(curState[i][0]*RAD2DEG,'f',1)+QChar(0x00B0));
			tblState->item(i,4)->setText(QString::number(curState[i+3][0]*RAD2DEG,'f',1)+QChar(0x00B0)+"/s");
		}
		for(int i=6; i<9; i++)
		{
			tblState->item(i-3,1)->setText(QString::number(curState[i][0],'f',3)+'m');
			tblState->item(i-3,4)->setText(QString::number(curState[i+3][0],'f',3)+'m');
		}

		Array2D<double> targetState;
		mMutex_dataAccess.lock();
		if(mControlType == CNTL_DIRECTION_THRUST)
			targetState = mControllerHierarchical.getDesiredState();
		else
			targetState = mControllerFeedbackLin.getDesiredState();
		for(int i=0; i<3; i++)
			tblState->item(i,0)->setText(QString::number(targetState[i][0]*RAD2DEG,'f',1)+QChar(0x00B0));
		for(int i=6; i<9; i++)
			tblState->item(i-3,0)->setText(QString::number(targetState[i][0],'f',3)+'m');

		for(int i=0; i<3; i++)
			tblState->item(i,2)->setText(QString::number((curState[i][0]-targetState[i][0])*RAD2DEG,'f',1)+QChar(0x00B0));
		for(int i=6; i<9; i++)
			tblState->item(i-3,2)->setText(QString::number(curState[i][0]-targetState[i][0],'f',3)+'m');

		// integrators
		Array2D<double> errInt;
		if(mControlType == CNTL_DIRECTION_THRUST)
		{
			errInt = mControllerHierarchical.getErrorMemory();
			for(int i=0; i<3; i++)
				tblState->item(i+3,3)->setText(QString::number(errInt[i][0],'f',3));
			for(int i=3; i<6; i++)
				tblState->item(i,3)->setText(QString::number(errInt[i][0],'f',3));
		}
		else
		{
			errInt = mControllerFeedbackLin.getErrorMemory();
			for(int i=0; i<3; i++)
				tblState->item(i,3)->setText(QString::number(errInt[i][0],'f',3));
			for(int i=3; i<6; i++)
				tblState->item(i,3)->setText(QString::number(errInt[i+3][0],'f',3));
		}
		mMutex_dataAccess.unlock();

		mMutex_buffers.lock();
		if(mControlBuffer.size() > 0)
		{
			lblCommandThrottle->setText(QString::number(mControlBuffer.back()[0][0],'f',1));
			lblCommandRoll->setText(QString::number(mControlBuffer.back()[1][0],'f',2));
			lblCommandPitch->setText(QString::number(mControlBuffer.back()[2][0],'f',2));
			lblCommandYaw->setText(QString::number(mControlBuffer.back()[3][0],'f',2));
		}
		mMutex_buffers.unlock();

		mPhoneInterface->updateDisplay();
	}

	void QuadrotorInterface::calcControl()
	{
		if(!mInitialized)
			throw("Quadrotor interface is not initialized.");

		unsigned long cntlStartTime = mTmr.getCurTimeMS();

		mMutex_dataAccess.lock();
		mDynamicModel->estimateFullState();

		if(mPhoneInterface->isUsingIbvs())
		{
			Array2D<double> curState = mDynamicModel->getCurState();
			Array2D<double> desState = mControllerHierarchical.getDesiredState();
			desState[6][0] = curState[6][0];
			desState[7][0] = curState[7][0];
			setDesiredState(desState);				
		}

		mMutex_buffers.lock();
			// target velocity should already have been set through its appropriate function
			Array2D<double> control;
			if(mControlType == CNTL_DIRECTION_THRUST)
			{
				control = mControllerHierarchical.calcControl();
				mStateRefBuffer.push_back(mControllerHierarchical.getDesiredState().copy());
			}
			else if(mControlType == CNTL_FEEDBACK_LIN)
			{
				control = mControllerFeedbackLin.calcControl();
				mStateRefBuffer.push_back(mControllerFeedbackLin.getDesiredState().copy());
			}
			else
				cout << "Unknown controller type: " << mControlType << endl;
			
			mControlBuffer.push_back(control.copy());
			mTimeBuffer.push_back(mTmr.getCurTimeMS()-mStartTimeUniverseMS);
			mFlightModeBuffer.push_back(mFlightMode);
		mMutex_buffers.unlock();
		mMutex_dataAccess.unlock();
	}

	void QuadrotorInterface::sendControl()
	{

		mMutex_buffers.lock();
		if(mControlBuffer.size() == 0)
		{
			mMutex_buffers.unlock();
			return;
		}

		// This doesn't work right now because the controllers are sending rate commands instead of
		// motor commands
// 		Array2D<double> curMotor(4,1);
// 		curMotor[0][0] = mControlBuffer.back()[0][0] + mControlBuffer.back()[2][0] - mControlBuffer.back()[3][0];
// 		curMotor[1][0] = mControlBuffer.back()[0][0] - mControlBuffer.back()[1][0] + mControlBuffer.back()[3][0];
// 		curMotor[2][0] = mControlBuffer.back()[0][0] - mControlBuffer.back()[2][0] - mControlBuffer.back()[3][0];
// 		curMotor[3][0] = mControlBuffer.back()[0][0] + mControlBuffer.back()[1][0] + mControlBuffer.back()[3][0];
// 		mDynamicModel->setCurActuation(curMotor);

		Packet p;
		p.type = COMM_RATE_CMD;
		p.time = mSys.mtime()-mStartTimeUniverseMS;
		p.dataFloat.resize(4);

			p.dataFloat[0] = mControlBuffer.back()[0][0];
			p.dataFloat[1] = mControlBuffer.back()[1][0];
			p.dataFloat[2] = mControlBuffer.back()[2][0];
			p.dataFloat[3] = mControlBuffer.back()[3][0];
		mMutex_buffers.unlock();

		Collection<tbyte> buff;
		p.serialize(buff);
		mPhoneInterface->sendUDP(buff.begin(), buff.size());

		// also send state now
		// TODO: Maybe move this to a separate function
		Array2D<double> curState;
		if(chkUseFilteredStates->isChecked())
			curState = mDynamicModel->getCurStateFiltered();
		else
			curState = mDynamicModel->getCurState();
		Packet pState;
		pState.type = COMM_STATE_VICON;
		pState.time = p.time;
		pState.dataFloat.resize(curState.dim1());
		for(int i=0; i<pState.dataFloat.size(); i++)
		{
			if(i >= 6)
			{
				// Box-Muller method for generating random numbers
				float U = rand()%1000+1;
				float V = rand()%1000+1;
				U *= 1.0/1000;
				V *= 1.0/1000;
				double noise = sqrt(-2*log(U))*cos(2*PI*V);
				double stdDev = 1*0.02;
				pState.dataFloat[i] = curState[i][0]+stdDev*noise;
			}
			else
				pState.dataFloat[i] = curState[i][0];
		}

		pState.serialize(buff);
		mPhoneInterface->sendUDP(buff.begin(), buff.size());

		// and the desired state for onboard position controller
		Array2D<double> desState = mControllerFeedbackLin.getDesiredState();
		Array2D<double> accel = mControllerFeedbackLin.getDesiredAccel();
		Packet pDesState;
		pState.type = COMM_DES_STATE;
		pState.time = p.time;
		pState.dataFloat.resize(15);
		for(int i=0; i<12; i++)
			pState.dataFloat[i] = desState[i][0]; 
		for(int i=0; i<3; i++)
			pState.dataFloat[i+12] = accel[i][0];
		pState.serialize(buff);
		mPhoneInterface->sendUDP(buff.begin(), buff.size());

		// IBVS stuff
//		Packet pDesMoment;
//		pDesMoment.type = COMM_DESIRED_IMAGE_MOMENT;
//		pDesMoment.time = p.time;
//		pDesMoment.dataFloat.resize(5);
//		
//		mMutex_dataAccess.lock();
//		double desZ;
//		if(mControlType == CNTL_DIRECTION_THRUST)
//			desZ = mControllerHierarchical.getDesiredState()[8][0];
//		else
//			desZ = mControllerFeedbackLin.getDesiredState()[8][0];
//		mMutex_dataAccess.unlock();
//
//		Collection<double> desMoment = mPhoneInterface->getDesiredImageMoment(desZ);
//		for(int i=0; i<3; i++)
//			pDesMoment.dataFloat[i] = desMoment[i];
//		pDesMoment.dataFloat[3] = desZ;
//		pDesMoment.dataFloat[4] = mPhoneInterface->getDesiredImageArea();
//		pDesMoment.serialize(buff);
//		mPhoneInterface->sendUDP(buff.begin(), buff.size());
	}

	void QuadrotorInterface::sendStopAndShutdown()
	{
		mPhoneInterface->sendMotorStop();
	}

	void QuadrotorInterface::sendMotorStart()
	{
		mPhoneInterface->sendMotorStart();
		sendPosControllerGains();
	}

	void QuadrotorInterface::sendParamsToPhone()
	{
		mPhoneInterface->sendParams();
		sendPosControllerGains();
	}

	void QuadrotorInterface::onBtnApply_clicked()
	{
		try
		{
			while(treeConfig->topLevelItemCount() > 0)
			{
				QTreeWidgetItem *item = treeConfig->takeTopLevelItem(0);
				if(item->text(0) == "ID")
					mName = item->text(1).toStdString();
				else if(item->text(0) == "Hardware")
				{
					mDynamicModel->applyConfigTree(item);
					if(mDynamicModel->stateFilterIsInitialized())
					{
						chkUseFilteredStates->setEnabled(true);
						chkUseFilteredStates->setCheckable(true);
					}
					else
					{
						chkUseFilteredStates->setEnabled(false);
						chkUseFilteredStates->setChecked(false);
						chkUseFilteredStates->setCheckable(false);
					}
				}
				else if(item->text(0) == "Controller")
					mControllerHierarchical.applyConfigTree(item);
				else if(item->text(0) == "Feedback Lin Controller")
					mControllerFeedbackLin.applyConfigTree(item);
				else if(item->text(0) == "Limits")
					applyLimitsConfig(item);
				else
				{
					QMessageBox box(QMessageBox::Warning, "Config Error", "Unknown config tree item " + item->text(0));
					box.exec();
					throw("Unknown config tree item: " + item->text(0));
				}
			}

			sendPosControllerGains();
		}
		catch(const char* exStr)
		{ 
			QMessageBox box(QMessageBox::Warning, "Config Error", exStr);
			box.exec();
		}

		populateConfigTree();
		clearAllBuffers();
	}

	void QuadrotorInterface::onBtnReset_clicked()
	{
		populateConfigTree();
	}

	void QuadrotorInterface::onBtnLoadFromFile_clicked()
	{
		QFileDialog fileDialog(this,"Open config","../","Quadrotor config files (*.quadConfig)");
		fileDialog.setDefaultSuffix("quadConfig");
		fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
		string filename;
		if(fileDialog.exec())
			filename = fileDialog.selectedFiles().front().toStdString();
		else
			return;

		loadConfigFromFile(filename);
	}

	void QuadrotorInterface::onBtnSaveToFile_clicked()
	{
		QFileDialog fileDialog(this,"Save config","../","Quadrotor config files (*.quadConfig)");
		fileDialog.setDefaultSuffix("quadConfig");
		fileDialog.setAcceptMode(QFileDialog::AcceptSave);
		string filename;
		if(fileDialog.exec())
			filename = fileDialog.selectedFiles().front().toStdString();
		else
			return;

		saveConfigToFile(filename);
	}

	void QuadrotorInterface::onBtnSetXyTarget_clicked()
	{
		Array2D<double> curState = mDynamicModel->getCurState();
		Array2D<double> targetState;
		if(mControlType == CNTL_DIRECTION_THRUST)
			targetState = mControllerHierarchical.getDesiredState();
		else
			targetState = mControllerFeedbackLin.getDesiredState();

		targetState[6][0] = curState[6][0];
		targetState[7][0] = curState[7][0];
		mControllerHierarchical.setDesiredState(targetState);
		mControllerFeedbackLin.setDesiredState(targetState);
		mProfileStartPoint = targetState.copy();
	}

	void QuadrotorInterface::onBtnResetIntegrators_clicked()
	{
		resetErrorMemory();
	}

	void QuadrotorInterface::onRdCntl_clicked()
	{
		if(rdCntl_DirectionThrust->isChecked())
		{
			mControlType = CNTL_DIRECTION_THRUST;
			cout << "Controller set to direction-thrust" << endl;
		}
		else if(rdCntl_FeedbackLin->isChecked())
		{
			mControlType = CNTL_FEEDBACK_LIN;
			cout << "Controller set to feedback linearizing" << endl;
		}
		else
		{
			QMessageBox box(QMessageBox::Warning,"Controller Error", "Error setting the controller");
			box.exec();
			throw("Error setting the controller");
		}

		mControllerHierarchical.resetErrorMemory();
		mControllerFeedbackLin.resetErrorMemory();
	}

	void QuadrotorInterface::onChkUseFilteredStates_clicked()
	{
		if(chkUseFilteredStates->isChecked())
		{
			mControllerHierarchical.useFilteredStates(true);
			mControllerFeedbackLin.useFilteredStates(true);
		}
		else
		{
			mControllerHierarchical.useFilteredStates(false);
			mControllerFeedbackLin.useFilteredStates(false);
		}
	}

	bool QuadrotorInterface::loadConfigFromFile(string filename)
	{
		cout << "Loading config from " << filename << endl;
		FILE *fp = fopen(filename.c_str(),"r");

		if(fp == NULL)
		{
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
		
		QString id = QString(mxmlFindElement(xmlRoot,xmlRoot,"ID",NULL,NULL,MXML_DESCEND)->child->value.text.string);
		mName = id.toStdString();
		mDynamicModel->setName(mName);

		mxml_node_t *hdwRoot = mxmlFindElement(xmlRoot,xmlRoot,"DynamicModel",NULL,NULL,MXML_DESCEND);
		if(hdwRoot != NULL)
			mDynamicModel->loadConfig(hdwRoot);
		if(mDynamicModel->stateFilterIsInitialized())
		{
			chkUseFilteredStates->setEnabled(true);
			chkUseFilteredStates->setCheckable(true);
		}
		else
		{
			chkUseFilteredStates->setEnabled(false);
			chkUseFilteredStates->setChecked(false);
			chkUseFilteredStates->setCheckable(false);
		}
		mPhoneInterface->setForceScaling(mDynamicModel->getAvgForceScaling());
		mPhoneInterface->setTorqueScaling(mDynamicModel->getAvgTorqueScaling());
		mPhoneInterface->setMass(mDynamicModel->getMass());

		mxml_node_t *cntlRoot = mxmlFindElement(xmlRoot,xmlRoot,"Controller",NULL,NULL,MXML_DESCEND);
		if(cntlRoot != NULL)
			mControllerHierarchical.loadConfig(cntlRoot);
		mControllerHierarchical.resetErrorMemory();
		mControllerHierarchical.setSystemModel(mDynamicModel);

		mxml_node_t *cntlRoot_FL = mxmlFindElement(xmlRoot,xmlRoot,"FeedbackLinController",NULL,NULL,MXML_DESCEND);
		if(cntlRoot_FL != NULL)
			mControllerFeedbackLin.loadConfig(cntlRoot_FL);
		mControllerFeedbackLin.resetErrorMemory();
		mControllerFeedbackLin.setSystemModel(mDynamicModel);

		mxml_node_t *limitsRoot = mxmlFindElement(xmlRoot,xmlRoot,"Limits",NULL,NULL,MXML_DESCEND);
		if(limitsRoot != NULL)
		{
			mxml_node_t *chanMinRoot = mxmlFindElement(limitsRoot,limitsRoot,"ChannelMin",NULL,NULL,MXML_DESCEND);
				mChannelMin[0] = QString(mxmlFindElement(chanMinRoot,chanMinRoot,"channel0",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelMin[1] = QString(mxmlFindElement(chanMinRoot,chanMinRoot,"channel1",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelMin[2] = QString(mxmlFindElement(chanMinRoot,chanMinRoot,"channel2",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelMin[3] = QString(mxmlFindElement(chanMinRoot,chanMinRoot,"channel3",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
			mxml_node_t *chanMaxRoot = mxmlFindElement(limitsRoot,limitsRoot,"ChannelMax",NULL,NULL,MXML_DESCEND);
				mChannelMax[0] = QString(mxmlFindElement(chanMaxRoot,chanMaxRoot,"channel0",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelMax[1] = QString(mxmlFindElement(chanMaxRoot,chanMaxRoot,"channel1",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelMax[2] = QString(mxmlFindElement(chanMaxRoot,chanMaxRoot,"channel2",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelMax[3] = QString(mxmlFindElement(chanMaxRoot,chanMaxRoot,"channel3",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
			mxml_node_t *chanZeroRoot = mxmlFindElement(limitsRoot,limitsRoot,"ChannelZero",NULL,NULL,MXML_DESCEND);
				mChannelZero[0] = QString(mxmlFindElement(chanZeroRoot,chanZeroRoot,"channel0",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelZero[1] = QString(mxmlFindElement(chanZeroRoot,chanZeroRoot,"channel1",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelZero[2] = QString(mxmlFindElement(chanZeroRoot,chanZeroRoot,"channel2",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelZero[3] = QString(mxmlFindElement(chanZeroRoot,chanZeroRoot,"channel3",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
			mxml_node_t *chanPolarityRoot = mxmlFindElement(limitsRoot,limitsRoot,"ChannelPolarity",NULL,NULL,MXML_DESCEND);
				mChannelPolarity[0] = QString(mxmlFindElement(chanPolarityRoot,chanPolarityRoot,"channel0",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelPolarity[1] = QString(mxmlFindElement(chanPolarityRoot,chanPolarityRoot,"channel1",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelPolarity[2] = QString(mxmlFindElement(chanPolarityRoot,chanPolarityRoot,"channel2",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
				mChannelPolarity[3] = QString(mxmlFindElement(chanPolarityRoot,chanPolarityRoot,"channel3",NULL,NULL,MXML_DESCEND)->child->value.text.string).toInt();
		}

		populateConfigTree();
		clearAllBuffers();

		sendPosControllerGains();

		return true;
	}

	void QuadrotorInterface::saveConfigToFile(string filename)
	{
		mxml_node_t *xmlRoot = mxmlNewXML("1.0");
		mxmlNewText(mxmlNewElement(xmlRoot,"ID"),0,mName.c_str());

		mxml_node_t *hdwNode = mxmlNewElement(xmlRoot,"DynamicModel");
		mDynamicModel->saveConfig(hdwNode);

		mxml_node_t *cntlNode = mxmlNewElement(xmlRoot, "Controller");
		mControllerHierarchical.saveConfig(cntlNode);

		mxml_node_t *cntlNode_FL = mxmlNewElement(xmlRoot, "FeedbackLinController");
		mControllerFeedbackLin.saveConfig(cntlNode_FL);
		
		mxml_node_t *cmdRoot = mxmlNewElement(xmlRoot, "Limits");
			mxml_node_t *chanMinNode = mxmlNewElement(cmdRoot,"ChannelMin");
				mxmlNewInteger(mxmlNewElement(chanMinNode,"channel0"),mChannelMin[0]);
				mxmlNewInteger(mxmlNewElement(chanMinNode,"channel1"),mChannelMin[1]);
				mxmlNewInteger(mxmlNewElement(chanMinNode,"channel2"),mChannelMin[2]);
				mxmlNewInteger(mxmlNewElement(chanMinNode,"channel3"),mChannelMin[3]);
			mxml_node_t *chanMaxNode = mxmlNewElement(cmdRoot,"ChannelMax");
				mxmlNewInteger(mxmlNewElement(chanMaxNode,"channel0"),mChannelMax[0]);
				mxmlNewInteger(mxmlNewElement(chanMaxNode,"channel1"),mChannelMax[1]);
				mxmlNewInteger(mxmlNewElement(chanMaxNode,"channel2"),mChannelMax[2]);
				mxmlNewInteger(mxmlNewElement(chanMaxNode,"channel3"),mChannelMax[3]);
			mxml_node_t *chanZeroNode = mxmlNewElement(cmdRoot,"ChannelZero");
				mxmlNewInteger(mxmlNewElement(chanZeroNode,"channel0"),mChannelZero[0]);
				mxmlNewInteger(mxmlNewElement(chanZeroNode,"channel1"),mChannelZero[1]);
				mxmlNewInteger(mxmlNewElement(chanZeroNode,"channel2"),mChannelZero[2]);
				mxmlNewInteger(mxmlNewElement(chanZeroNode,"channel3"),mChannelZero[3]);
			mxml_node_t *chanPolarityNode = mxmlNewElement(cmdRoot,"ChannelPolarity");
				mxmlNewInteger(mxmlNewElement(chanPolarityNode,"channel0"),mChannelPolarity[0]);
				mxmlNewInteger(mxmlNewElement(chanPolarityNode,"channel1"),mChannelPolarity[1]);
				mxmlNewInteger(mxmlNewElement(chanPolarityNode,"channel2"),mChannelPolarity[2]);
				mxmlNewInteger(mxmlNewElement(chanPolarityNode,"channel3"),mChannelPolarity[3]);
			
		FILE *fp = fopen((filename).c_str(),"w");
		mxmlSaveFile(xmlRoot, fp, ICSL::XmlUtils::whitespaceCallback);
		fclose(fp);
	}
	
	void QuadrotorInterface::clearAllBuffers()
	{
		mMutex_buffers.lock();
			mStateRefBuffer.clear();
			mStateVelRefBuffer.clear();
			mErrorMemorySlidingModeBuffer.clear();
			mControlBuffer.clear();
			mTimeBuffer.clear();
			mFlightModeBuffer.clear();
			mDynamicModel->clearAllBuffers();
		mMutex_buffers.unlock();
	}

	void QuadrotorInterface::resetAll()
	{
		resetErrorMemory();
		mDynamicModel->resetState();
	}

////////////////////////////////////////////////////////////////////////////////////////////////////

	inline double QuadrotorInterface::constrain(double val, double minVal, double maxVal)
	{
		return min(maxVal, max(minVal, val));
	}

	inline double QuadrotorInterface::roundToPrecision(double val, int precision)
	{
		return ((int)(val*pow(10.0,precision)+0.5))/pow(10.0,precision);
	}

	Array2D<double> QuadrotorInterface::convertTelemViconToState(TelemetryViconDataRecord const &curRec)
	{
		Array2D<double> state(12,1,0.0);
		state[0][0] = curRec.roll; state[1][0] = curRec.pitch; state[2][0] = curRec.yaw;
		state[6][0] = curRec.x; state[7][0] = curRec.y; state[8][0] = curRec.z;

		return state;
	}

// 	Array2D<double> QuadrotorInterface::estimateState(TelemetryViconDataRecord const &curRec, list<Array2D<double> > const & stateBuffer, double deltaT)
// 	{
// 		Array2D<double> state = convertTelemViconToState(curRec);
// 		return estimateState(state, stateBuffer, deltaT);
// 	}
// 
// 	/*!
// 	* Derivative filters taken from http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
// 	*
// 	* \param state a 12 element state vector of the form
// 	*	[roll pitch yaw rollRate pitchRate yawRate x y z xRate y Rate zRate]
// 	* where all the rates are ignored
// 	* \param stateBuffer vector of the estimated full states(from oldest to newest)
// 	* \param deltaT time step size
// 	* \return the 12 element state vector with filtered velocity estimates
// 	* */
// 	Array2D<double> QuadrotorInterface::estimateState(Array2D<double> &state, list<Array2D<double> > const & stateBuffer, double deltaT)
// 	{
// 		// filterCoeff[0][0] is coefficient for newest record
// 		// filterCoeff[end][0] is coefficient for oldest record
// 		Array2D<double> filterCoeff(min((int)(stateBuffer.size()+1),8),1);
// 		switch(filterCoeff.dim1())
// 		{
// 			case 1:
// 				filterCoeff[0][0] = 0;
// 				break;
// 			case 2:
// 				filterCoeff[0][0] = 1; filterCoeff[1][0] = -1;
// 				filterCoeff = 1.0/deltaT*filterCoeff;
// 				break;
// 			case 3:
// 				filterCoeff = Array2D<double>(2,1); // there is no entry for this so just use the 2 point average
// 				filterCoeff[0][0] = 1; filterCoeff[1][0] = -1;
// 				filterCoeff = 1.0/deltaT*filterCoeff;
// 				break;
// 			case 4:
// 				filterCoeff[0][0] = 2; filterCoeff[1][0] = -1; filterCoeff[2][0] = -2; filterCoeff[3][0] = 1;
// 				filterCoeff = 1.0/2.0/deltaT*filterCoeff;
// 				break;
// 			case 5:
// 				filterCoeff[0][0] = 7; filterCoeff[1][0] = 1; filterCoeff[2][0] = -10; filterCoeff[3][0] = -1;
// 				filterCoeff[4][0] = 3;
// 				filterCoeff = 1.0/10.0/deltaT*filterCoeff;
// 				break;
// 			case 6:
// 				filterCoeff[0][0] = 16; filterCoeff[1][0] = 1; filterCoeff[2][0] = -10; filterCoeff[3][0] = -10;
// 				filterCoeff[4][0] = -6; filterCoeff[5][0] = 9;
// 				filterCoeff = 1.0/28.0/deltaT*filterCoeff;
// 				break;
// 			case 7:
// 				filterCoeff[0][0] = 12; filterCoeff[1][0] = 5; filterCoeff[2][0] = -8; filterCoeff[3][0] = -6;
// 				filterCoeff[4][0] = -10; filterCoeff[5][0] = 1; filterCoeff[6][0] = 6;
// 				filterCoeff = 1.0/28.0/deltaT*filterCoeff;
// 				break;
// 			case 8:
// 				filterCoeff[0][0] = 22; filterCoeff[1][0] = 7; filterCoeff[2][0] = -6; filterCoeff[3][0] = -11;
// 				filterCoeff[4][0] = -14; filterCoeff[5][0] = -9; filterCoeff[6][0] = -2; filterCoeff[7][0] = 13; 
// 				filterCoeff = 1.0/60.0/deltaT*filterCoeff;
// 				break;
// 			default:
// 				QMessageBox box(QMessageBox::Warning, "State Estimation error", "Unknown filter size: " + QString::number(filterCoeff.dim1()));
// 				box.exec();
// 				throw("Unknown filter size: " + filterCoeff.dim1());
// 		}
// 
// 		state[3][0] = filterCoeff[0][0]*state[0][0];
// 		state[4][0] = filterCoeff[0][0]*state[1][0];
// 		state[5][0] = filterCoeff[0][0]*state[2][0];
// 		state[9][0] = filterCoeff[0][0]*state[6][0];
// 		state[10][0] = filterCoeff[0][0]*state[7][0];
// 		state[11][0] = filterCoeff[0][0]*state[8][0];
// 
// 		list<Array2D<double> >::const_iterator iter_state = stateBuffer.end();
// 		Array2D<double> stateTemp;
// 		for(int i=1; i<filterCoeff.dim1(); i++)
// 		{
// 			stateTemp = *(--iter_state);
// 			for(int st=3; st<6; st++)
// 				state[st][0] += filterCoeff[i][0]*stateTemp[st-3][0];
// 			for(int st=9; st<12; st++)
// 				state[st][0] += filterCoeff[i][0]*stateTemp[st-3][0];
// 		}
// 		return state.copy();
// 	}

	void QuadrotorInterface::resetErrorMemory()
	{
		mControllerHierarchical.resetErrorMemory();
		mControllerFeedbackLin.resetErrorMemory();
	}

	void QuadrotorInterface::saveData(string dir, string filename)
	{
		const string stateNames[] = {"roll",	"pitch", "yaw", "roll rate", "pitch rate", "yaw rate",
			"x", "y", "z", "x vel", "y vel", "z vel"};
        fstream dataStream;
        dataStream.open((dir+"/"+filename).c_str(),fstream::out); 
        dataStream << "frame,\tflight mode,\ttime,\tCmdThrottle,\tCmdRoll,\tCmdPitch,\tCmdYaw"; 
        for(int j=0; j<12; j++)
        	dataStream << ",\tDesired " << stateNames[j];
        for(int j=0; j<12; j++)
        	dataStream << ",\tActual " << stateNames[j];
		for(int j=0; j<12; j++)
			dataStream << ",\tFiltered " << stateNames[j];
        dataStream << endl; 

        // save data
		mMutex_buffers.lock();
			list<int>::iterator iter_flightMode = mFlightModeBuffer.begin();
			list<unsigned long>::iterator iter_time = mTimeBuffer.begin();
			list<Array2D<double> >::iterator iter_cntl = mControlBuffer.begin();
			list<Array2D<double> >::iterator iter_stateRef = mStateRefBuffer.begin();
			list<Array2D<double> >::const_iterator iter_state = mDynamicModel->getStateBuffer()->begin();
			list<Array2D<double> >::const_iterator iter_stateFilt = mDynamicModel->getStateFilteredBuffer()->begin();
			if(mTimeBuffer.size() == 0) 
				mTimeBuffer.push_back(1); // size() returns an unsigned int so size()-1 is a very large positive number
			for(int i=0; i<mTimeBuffer.size()-1; i++) // only go up to the second to last data point since sometimes the buffers are off in size by 1
			{
				dataStream << i; 
				dataStream << ",\t" << *(iter_flightMode++);
				dataStream << ",\t" << *(iter_time++);
	
				for(int j=0; j<4; j++) 
					dataStream << ",\t" << (*(iter_cntl))[j][0];
				iter_cntl++;
				for(int j=0; j<12; j++)
					dataStream << ",\t" << (*(iter_stateRef))[j][0];
				iter_stateRef++;
				for(int j=0; j<12; j++)
					dataStream << ",\t" << (*(iter_state))[j][0];
				iter_state++;
				for(int j=0; j<12; j++)
					dataStream << ",\t" << (*(iter_stateFilt))[j][0];
				iter_stateFilt++;
	
				dataStream << endl;
			} 
	
	        dataStream.close();
		mMutex_buffers.unlock();
	}

	void QuadrotorInterface::populateConfigTree()
	{
		while(treeConfig->topLevelItemCount() > 0)
			treeConfig->takeTopLevelItem(0);

		QStringList headers;
		headers << "" << "" << "" << "" << "" << "";
		treeConfig->setHeaderItem(new QTreeWidgetItem((QTreeWidget*)0,headers));
		QTreeWidgetItem *nameRoot = new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("ID")));
		nameRoot->setText(1,QString(mName.c_str()));

		QStringList hdwHeaders;
		hdwHeaders << "Hardware";
		QTreeWidgetItem *hdwRoot = new QTreeWidgetItem((QTreeWidget*)0,hdwHeaders);
		mDynamicModel->populateConfigTree(hdwRoot);

		QStringList controllerHeadings, smcHeadings, ibvsHeadings;
		controllerHeadings << "Controller" ;
		QTreeWidgetItem *cntlRoot = new QTreeWidgetItem((QTreeWidget*)0,controllerHeadings);
		mControllerHierarchical.populateConfigTree(cntlRoot);

		QStringList controllerHeadings_FL;
		controllerHeadings_FL << "Feedback Lin Controller" << "Gain" << "Int Gain" << "Int Limit";
		QTreeWidgetItem *cntlRoot_FL = new QTreeWidgetItem((QTreeWidget*)0,controllerHeadings_FL);
		mControllerFeedbackLin.populateConfigTree(cntlRoot_FL);

		QStringList commandHeadings;
		commandHeadings << "Limits" << "Throttle" << "Roll" << "Pitch" << "Yaw";
		QTreeWidgetItem *cmdRoot = new QTreeWidgetItem((QTreeWidget*)0,commandHeadings);
			cmdRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Min"))));
				for(int i=0; i<4; i++)
					cmdRoot->child(cmdRoot->childCount()-1)->setText(i+1,QString::number(mChannelMin[i]));
			cmdRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Max"))));
				for(int i=0; i<4; i++)
					cmdRoot->child(cmdRoot->childCount()-1)->setText(i+1,QString::number(mChannelMax[i]));
			cmdRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Zero"))));
				for(int i=0; i<4; i++)
					cmdRoot->child(cmdRoot->childCount()-1)->setText(i+1,QString::number(mChannelZero[i]));
			cmdRoot->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Polarity"))));
				for(int i=0; i<4; i++)
					cmdRoot->child(cmdRoot->childCount()-1)->setText(i+1,QString::number(mChannelPolarity[i]));

		treeConfig->addTopLevelItem(hdwRoot);
		treeConfig->addTopLevelItem(nameRoot);
		treeConfig->addTopLevelItem(cntlRoot);
		treeConfig->addTopLevelItem(cntlRoot_FL);
		treeConfig->addTopLevelItem(cmdRoot);
		treeConfig->expandAll();

		formatTree(hdwRoot);
		formatTree(nameRoot);
		formatTree(cntlRoot);
		formatTree(cntlRoot_FL);
		formatTree(cmdRoot);
//		formatTree(phoneRoot);


		int width = 0;
		for(int i=0; i<treeConfig->columnCount(); i++)
		{
			hdwRoot->setBackground(i,QColor(50,0,0,50));
			cntlRoot->setBackground(i,QColor(50,0,0,50));
			cntlRoot_FL->setBackground(i,QColor(50,0,0,50));
			cmdRoot->setBackground(i,QColor(50,0,0,50));
			treeConfig->resizeColumnToContents(i);
//			treeConfig->setColumnWidth(i,treeConfig->columnWidth(i)+4);
			width += treeConfig->columnWidth(i);
		}

		treeConfig->setHeaderHidden(true);
		treeConfig->setMinimumSize(width,treeConfig->height());
	}

	void QuadrotorInterface::formatTree(QTreeWidgetItem *root)
	{
		root->setFlags(root->flags() | Qt::ItemIsEditable);
		for(int i=1; i<root->columnCount(); i++)
			root->setTextAlignment(i,Qt::AlignHCenter);
		
		if(root->childCount() > 0)
			for(int i=0; i<root->childCount(); i++)
				formatTree(root->child(i));
	}

	void QuadrotorInterface::applyLimitsConfig(QTreeWidgetItem *root)
	{
		while(root->childCount() > 0)
		{
			QTreeWidgetItem *item = root->takeChild(0);
			if(item->text(0) == "Min")
			{
				for(int i=0; i<4; i++)
					mChannelMin[i] = item->text(i+1).toDouble();
			}
			else if(item->text(0) == "Max")
			{
				for(int i=0; i<4; i++)
					mChannelMax[i] = item->text(i+1).toDouble();
			}
			else if(item->text(0) == "Zero")
			{
				for(int i=0; i<4; i++)
					mChannelZero[i] = item->text(i+1).toDouble();
			}
			else if(item->text(0) == "Polarity")
			{
				for(int i=0; i<4; i++)
					mChannelPolarity[i] = item->text(i+1).toInt() > 0 ? 1 : -1;
			}
			else
			{
				QMessageBox box(QMessageBox::Warning,"Config Error", "Unknown command config item: " + item->text(0));
				box.exec();
				throw("Unknown command config item: " + item->text(0));
			}
		}
	}
	void QuadrotorInterface::sendPosControllerGains()
	{
		float gainP[12], gainI[12], gainILimit[12];
		Array2D<double> cntlGainP = mControllerFeedbackLin.getGainP();
		Array2D<double> cntlGainI = mControllerFeedbackLin.getGainI();
		Array2D<double> cntlGainILimit = mControllerFeedbackLin.getGainILimit();

		for(int i=0; i<12; i++)
		{
			gainP[i] = cntlGainP[i][0];
			gainI[i] = cntlGainI[i][0];
			gainILimit[i] = cntlGainILimit[i][0];
		}

		float mass = mDynamicModel->getMass();
		float forceScaling = mDynamicModel->getAvgForceScaling();

		int code = COMM_POS_CNTL_GAINS;
		bool result = true;
		result = result && mPhoneInterface->sendTCP((tbyte*)&code, sizeof(code));
		result = result && mPhoneInterface->sendTCP((tbyte*)gainP, 12*sizeof(float));
		result = result && mPhoneInterface->sendTCP((tbyte*)gainI, 12*sizeof(float));
		result = result && mPhoneInterface->sendTCP((tbyte*)gainILimit, 12*sizeof(float));
		result = result && mPhoneInterface->sendTCP((tbyte*)&mass, sizeof(mass));
		result = result && mPhoneInterface->sendTCP((tbyte*)&forceScaling, sizeof(forceScaling));

		cout << "Pos control gains sent" << endl;
	}
}
}

