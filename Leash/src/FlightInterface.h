#ifndef QUADROTOR_FLIGHTINTERFACE
#define QUADROTOR_FLIGHTINTERFACE
#pragma warning(disable : 4996)
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <exception>
#include <string>
#include <vector>

#include <QObject>
#include <QMainWindow>
#include <QtGui>
#include <QShortcut>
#include <QMetaType>
#include <QButtonGroup>

#include "TNT/tnt.h"

//#include "C:/Advantech/DAQNavi/Inc/bdaqctrl.h"

#include "ICSL/constants.h"
#include "ICSL/ControlTimer/src/ControlTimer.h"
#include "ICSL/SystemModel/ISystemModel.h"
#include "ICSL/SystemModel/ISystemModelAffine.h"
#include "ICSL/Timer/src/Timer.h"
#include "ICSL/TNT_Utils/TNT_Utils.h"

#include "Quadrotor/quadrotorConstants.h"
#include "Quadrotor/quadrotor_config.h"
#include "Quadrotor/TelemetryVicon/src/TelemetryVicon.h"

#include "QuadrotorInterface.h"
#include "ui_FlightInterface.h"

namespace ICSL{
namespace Quadrotor{
	using namespace std;
//	class FlightInterface : public QMainWindow, public Automation::BDaq::CntrEventListener, protected Ui::FlightInterface
	class FlightInterface : public QMainWindow, public ControlTimerListener, protected Ui::FlightInterface
	{
		Q_OBJECT

		public:
			explicit FlightInterface(QWidget *parent=0);
			virtual ~FlightInterface();

			void setDeltaT(double dt);

			void doEmergencyShutdown();
			
			void initialize();
			void run();
//			virtual void BDAQCALL CntrEvent(void * sender, Automation::BDaq::CntrEventArgs * args){doControl();}
			void onTimerEvent(){doControl();}
			void doControl();
			TNT::Array2D<double> runPathPlanner();

			void saveData(string dir, string filename);

		protected slots:
			void updateDisplay();
			void onBtnStartMotors_clicked();
			void onBtnStopMotors_clicked();
			void onBtnBeginTracking_clicked();
			void onBtnQuit_clicked();
			void onBtnClearBuffers_clicked();
			void onIncreaseHeight();
			void onDecreaseHeight();
			void onMoveLeft();
			void onMoveRight();
			void onMoveForward();
			void onMoveBackward();
			void onQuadModelChangeStarted(QuadrotorInterface* );
			void onQuadModelChangeDone(QuadrotorInterface* quad);
			void onToggleIbvs();

		protected:
			double mDeltaT;
			unsigned long mStartTimeUniverseMS, mProfileStartTime, mLastCntlTime;
			QTimer *mTmrGui;
			TelemetryVicon mTelemVicon;

			FlightMode mFlightMode;
			QuadrotorInterface *mQuad;
			QuadrotorInterface *quadDisplayA;

			ICSL::Timer mTmr;

			QShortcut *mScStartMotors, *mScStopMotors, *mScBeginTracking, *mScQuit;
			QShortcut *mScIncreaseHeight, *mScDecreaseHeight, *mScMoveLeft, *mScMoveRight, *mScMoveForward, *mScMoveBackward;
			QShortcut *mScToggleIbvs;

			TNT::Array2D<double> runPathPlannerCircle();
	};
}
}

#endif
