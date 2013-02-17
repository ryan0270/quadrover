#include "TNT/tnt.h"
#include "FlightInterface.h"

using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{
	using namespace std;
	using namespace ICSL;
	using toadlet::int64;

	FlightInterface::FlightInterface(QWidget *parent)
	{
		setupUi(this);
 		mTmrGui = new QTimer(this);

		mFlightMode = QUAD_IDLE;

		QSettings settings("ICSL", "Quadrotor Phone Interface");
		restoreGeometry(settings.value("geometry").toByteArray());
		int uiVersion= 1;
		restoreState(settings.value("state").toByteArray(),uiVersion);

		mLeash = NULL;
	}

	FlightInterface::~FlightInterface()
	{
		QSettings settings("ICSL", "Quadrotor Phone Interface");
		settings.setValue("geometry", saveGeometry());
		settings.setValue("state", saveState(1));

		mTelemVicon.stopMonitor();
		mTelemVicon.disconnect();

		if(mTmrGui != NULL)
			delete mTmrGui;

		delete mScStartMotors;
		delete mScStopMotors;
		delete mScBeginTracking;
		delete mScQuit;

		delete mScIncreaseHeight;
		delete mScDecreaseHeight;
		delete mScMoveLeft;
		delete mScMoveRight;
		delete mScMoveForward;
		delete mScMoveBackward;
		delete mScToggleIbvs;

//		delete mQuad;

		if(mLeash != NULL) delete mLeash;
	}

	void FlightInterface::initialize()
	{
		connect(mTmrGui, SIGNAL(timeout()), this, SLOT(updateDisplay()));
		connect(btnStartMotors, SIGNAL(clicked()), this, SLOT(onBtnStartMotors_clicked()));
		connect(btnStopMotors,  SIGNAL(clicked()), this, SLOT(onBtnStopMotors_clicked()));
		connect(btnBeginTracking, SIGNAL(clicked()), this, SLOT(onBtnBeginTracking_clicked()));
		connect(btnQuit, SIGNAL(clicked()), this, SLOT(onBtnQuit_clicked()));
		connect(btnClearBuffers,SIGNAL(clicked()),this,SLOT(onBtnClearBuffers_clicked()));

		mScStartMotors = new QShortcut(Qt::Key_W, this);
		mScStopMotors = new QShortcut(Qt::Key_Space, this);
		mScBeginTracking = new QShortcut(Qt::Key_S, this);
		mScQuit = new QShortcut(Qt::Key_Q, this);
		connect(mScStartMotors,SIGNAL(activated()), this, SLOT(onBtnStartMotors_clicked()));
		connect(mScStopMotors,SIGNAL(activated()), this, SLOT(onBtnStopMotors_clicked()));
		connect(mScBeginTracking,SIGNAL(activated()), this, SLOT(onBtnBeginTracking_clicked()));
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
		connect(mScToggleIbvs,SIGNAL(activated()), this, SLOT(onToggleIbvs()));

//		mQuad = new QuadrotorInterface();
		mLeash = new Leash();
		mLeash->initialize();
		mLeash->loadConfigFromFile("../quad0.leashConfig");
		
//		int quadIndex = 0;
		cout << "Initializing quad " << 0 << " ... ";
//		mQuad->initialize(); // this is hopefully redundant with the load from file, but not sure right now
//		string filename = "../quad" + QString::number(0).toStdString() + ".quadConfig";
//		if(mQuad->loadConfigFromFile(filename))
//		{
//			filename = "../"+mQuad->getName()+".phoneConfig";
//			mQuad->getPhoneInterface()->loadConfigFromFile(filename);
//			cout << " success." << endl;
//		}
//		else
//		{
//			cout << filename << " not found. Continuing with defaults." << endl;
//			mQuad->setName("Unk" + QString::number(quadIndex++).toStdString());
//		}		

		layQuadA->addWidget(mLeash);

		cout << "Connecting to Vicon ... ";
		try
		{
			mTelemVicon.setOriginPosition(Array2D<double>(3,1,0.0));
			mTelemVicon.initializeMonitor();
			if(mTelemVicon.connect("localhost:801") == false)
			{
//				mQuad->setSimulated(true);

//				mQuad->setDeltaT(1000); // need something here
//				// Do this so we start out at hover
//				Array2D<double> stateRef = mQuad->getDesiredState();
//				mQuad->setDesiredState(stateRef);
//
//				mQuad->calcControl();
//				mQuad->sendControl();
			}
		}
		catch(const TelemetryViconException& ex)	{ cout << "Failure" << endl; throw(ex); }
		cout << "Success" << endl;
//		mTelemVicon.addTrackedQuadrotor(mQuad->getName());
//		mTelemVicon.addListener(mQuad->getDynamicModel());

//		quadDisplayA = mLeash;
	}

	void FlightInterface::run()
	{
		mTelemVicon.startMonitor();
		mStartTimeUniverseMS = mTmr.getCurTimeMS();
		mLeash->syncStartTime(mStartTimeUniverseMS);
		mTmrGui->start(50);
	}

	void FlightInterface::setDeltaT(double dt)
	{
		mDeltaT = dt;
//		mLeash->setDeltaT(dt);
	}

	void FlightInterface::doEmergencyShutdown()
	{
//		mQuad->sendStopAndShutdown();
	}

	void FlightInterface::doControl()
	{
		unsigned long cntlTime = mTmr.getCurTimeMS();
//		cout << "Control time: " << cntlTime-mLastCntlTime << endl;
		mLastCntlTime = cntlTime;
		Array2D<double> out = runPathPlanner();
		Array2D<double> desState = submat(out,0,11,0,0);
		Array2D<double> desAccel = submat(out,12,14,0,0);
//		mQuad->setDesiredState(desState);
//		mQuad->setDesiredAccel(desAccel);

//		mQuad->calcControl();
//		mQuad->sendControl();
	}

	Array2D<double> FlightInterface::runPathPlanner()
	{
		Array2D<double> desState(12,1,0.0), accel(3,1,0.0);
		switch(mFlightMode)
		{
			case QUAD_IDLE:
//				desState = mQuad->getDesiredState();
				break;
			case QUAD_PROFILE_TRACKING:
				{
					Array2D<double> out = runPathPlannerCircle();
					desState = submat(out,0,11,0,0);
					accel = submat(out,12,14,0,0);
				}
				break;
			default:
				cout << "Unkown flight mode: " << mFlightMode << endl;
		}

		return stackVertical(desState, accel);
	}

	Array2D<double> FlightInterface::runPathPlannerCircle()
	{
		double t = (mTmr.getCurTimeMS()-mProfileStartTime)/1000.0;

		int numCycles = 3;
		double maxVel = 2.0*PI/5; // max angular velocity
//		double cyclePeriod = 1/maxVel; // time for 1 cycle if always at max velocity
		double t1 = 1; // time to ramp speed from rest to max speed
		double t2 = numCycles*2.0*PI/maxVel; // time to start ramp from max speed to rest
 
		double alpha;
		double omega, angle;
		if(t < t1)
		{
			omega = t/t1*maxVel;
			angle = 0.5*t*omega;
			alpha = maxVel/t1;
		}
		else if(t < t2)
		{
			omega = maxVel;
			angle = 0.5*t1*omega+(t-t1)*omega;
			alpha = 0;
		}
		else if(t < t2+t1)
		{
			omega = (t2+t1-t)/t1*maxVel;
			angle = t2*maxVel-0.5*(t2+t1-t)*omega;
			alpha = -maxVel/t1;
		}
		else // we are done with the profile
		{
			omega = 0;
			angle = 0;
			alpha = 0;
		}

		Array2D<double> xDes(12,1,0.0);
		double r = 1;
		xDes[6][0] = r*cos(angle);
		xDes[7][0] = r*sin(angle);
		xDes[9][0] = -omega*xDes[7][0];
		xDes[10][0] = omega*xDes[6][0];

		// changing yaw
//		xDes[2][0] = -0.5*PI*sin(angle);
//		xDes[5][0] = -0.5*PI*omega*cos(angle);

		// Set the z height to a constant at whatever the current target is
//		Array2D<double> curDes = mQuad->getDesiredState();
//		xDes[8][0] = curDes[8][0];
//
//		double xPos = xDes[6][0];
//		double yPos = xDes[7][0];
//		double xVel = xDes[9][0];
//		double yVel = xDes[10][0];
//		Array2D<double> accelDes(3,1,0.0);
//		accelDes[0][0] = -yVel*omega-yPos*alpha;
//		accelDes[1][0] = xVel*omega+xPos*alpha;
//		accelDes[2][0] = 0;
//
//		return stackVertical(xDes, accelDes);
		return Array2D<double>(15,0,0.0);
	}

	void FlightInterface::saveData(string dir, string filename)
	{
		string subDir = dir;
//		mQuad->saveData(subDir, filename);
	}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Slots
	void FlightInterface::updateDisplay()
	{
//		unsigned long dispTimeStart = mTmr.getCurTimeMS();
		lblRunTime->setText(QString::number((int)((mTmr.getCurTimeMS()-mStartTimeUniverseMS)/1000.0 + 0.5)));
//		quadDisplayA->updateDisplay();
		mLeash->updateDisplay();
	}

	void FlightInterface::onBtnStartMotors_clicked()
	{		
		mFlightMode = QUAD_IDLE;
//		mQuad->resetErrorMemory();
//		mQuad->sendMotorStart();
	}

	void FlightInterface::onBtnStopMotors_clicked()
	{
		mFlightMode = QUAD_IDLE;
//		mQuad->sendStopAndShutdown();
	}

	void FlightInterface::onBtnBeginTracking_clicked()
	{
//		mQuad->setFlightMode(QUAD_PROFILE_TRACKING);

		mFlightMode = QUAD_PROFILE_TRACKING;
		mProfileStartTime = mTmr.getCurTimeMS();
	}

	void FlightInterface::onBtnQuit_clicked()
	{
cout << "1" << endl;
		mFlightMode = QUAD_IDLE;
cout << "2" << endl;
//		mQuad->sendStopAndShutdown();
cout << "3" << endl;
		mTmrGui->stop();
cout << "4" << endl;
		qApp->quit();
cout << "5" << endl;
	}

	void FlightInterface::onBtnClearBuffers_clicked()
	{
//		mQuad->clearAllBuffers();
	}

	void FlightInterface::onIncreaseHeight()
	{
//		Array2D<double> state = mQuad->getDesiredState();
//		state[8][0] = min(state[8][0]+0.050,1.0);
//		mQuad->setDesiredState(state);
	}

	void FlightInterface::onDecreaseHeight()
	{
//		Array2D<double> state = mQuad->getDesiredState();
//		state[8][0] = max(state[8][0]-0.050,0.0);
//		mQuad->setDesiredState(state);
	}

	void FlightInterface::onMoveLeft()
	{
//		Array2D<double> state = mQuad->getDesiredState();
//		state[6][0] -=0.200;
//		mQuad->setDesiredState(state);
	}

	void FlightInterface::onMoveRight()
	{
//		Array2D<double> state = mQuad->getDesiredState();
//		state[6][0] += 0.200;
//		mQuad->setDesiredState(state);
	}

	void FlightInterface::onMoveForward()
	{
//		Array2D<double> state = mQuad->getDesiredState();
//		state[7][0] += 0.200;
//		mQuad->setDesiredState(state);
	}

	void FlightInterface::onMoveBackward()
	{
//		Array2D<double> state = mQuad->getDesiredState();
//		state[7][0] -=0.2000;
//		mQuad->setDesiredState(state);
	}

	void FlightInterface::onToggleIbvs()
	{
//		mQuad->toggleIbvs();
	}
//	void FlightInterface::onQuadModelChangeStarted(QuadrotorInterface *quad)
//	{
//		disconnect(&mTelemVicon,SIGNAL(newRecordAvailable(const TelemetryViconDataRecord&)),
//			quad->getDynamicModel(), SLOT(onNewMeasurementAvailable(const TelemetryViconDataRecord&)));
//	}

//	void FlightInterface::onQuadModelChangeDone(QuadrotorInterface *quad)
//	{
//		connect(&mTelemVicon,SIGNAL(newRecordAvailable(const TelemetryViconDataRecord&)),
//			quad->getDynamicModel(), SLOT(onNewMeasurementAvailable(const TelemetryViconDataRecord&)));
//	}
}
}
