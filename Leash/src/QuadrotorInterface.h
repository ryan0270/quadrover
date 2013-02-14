#ifndef QUADROTOR_QUADROTORINTERFACE
#define QUADROTOR_QUADROTORINTERFACE
#pragma warning(disable : 4996)

#include <toadlet/egg.h>
//#include <toadlet/flick.h>

#include <fstream>
#include <iostream>
#include <list>
#include <vector>

#include <QObject>
#include <QWidget>
#include <QtGui>
//#include <QMutex>

#include "TNT/tnt.h"

#include "ICSL/icsl_config.h"
#include "ICSL/constants.h"
#include "ICSL/TNT_Utils/TNT_Utils.h"
#include "ICSL/Timer/src/Timer.h"
#include "ICSL/xml_utils/xml_utils.h"
#include "ICSL/SystemModel/ISystemModel.h"

#include "Quadrotor/quadrotor_config.h"
#include "Quadrotor/quadrotorConstants.h"
#include "Quadrotor/TelemetryVicon/src/TelemetryVicon.h"
#include "Quadrotor/SystemModelQuadrotor/src/SystemModelQuadrotor.h"

#include "SystemControllerHierarchical.h"
#include "SystemControllerFeedbackLin.h"
#include "PhoneInterface.h"
#include "ui_QuadrotorInterface.h"
#include "../../Rover/cpp/Common.h"


namespace ICSL{
namespace Quadrotor {
using namespace std;
class QuadrotorInterface : public QWidget, public PhoneInterfaceListener, protected Ui::QuadrotorInterface
{
	Q_OBJECT

	public:
		enum CommStatus
		{FREE, BUSY};

		enum ControlType
		{CNTL_DIRECTION_THRUST, CNTL_FEEDBACK_LIN};

		explicit QuadrotorInterface(QWidget *parent=0);
		virtual ~QuadrotorInterface();

		void setDesiredState(TNT::Array2D<double> const &state);
		void setDesiredAccel(TNT::Array2D<double> const &accel);
		void setName(string name){mDynamicModel->setName(name);}
		void setFlightMode(FlightMode mode){mFlightMode = mode;} /// this is only used for data logging purposes
		void setDeltaT(double dt)
		{mDeltaT = dt; mControllerHierarchical.setDeltaT(dt); mControllerFeedbackLin.setDeltaT(dt); mDynamicModel->setDeltaT(dt); mPhoneInterface->setDeltaT(dt);}
		void setSimulated(bool b){mDynamicModel->setSimulated(true);}

		TNT::Array2D<double> const getCurState(){mMutex_dataAccess.lock(); Array2D<double> temp = mDynamicModel->getCurState(); mMutex_dataAccess.unlock(); return temp;}
		TNT::Array2D<double> const getCurControl(){mMutex_dataAccess.lock(); Array2D<double> temp = mDynamicModel->getCurActuator(); mMutex_dataAccess.unlock(); return temp;}
		string getName(){return mName;};
		SystemModelQuadrotor* const getDynamicModel(){return mDynamicModel;};
		TNT::Array2D<double> getDesiredState(){mMutex_dataAccess.lock(); Array2D<double> temp = mControllerHierarchical.getDesiredState(); mMutex_dataAccess.unlock(); return temp;}
		PhoneInterface* getPhoneInterface(){return mPhoneInterface;};

		void initialize();
		void syncStartTime(unsigned long start){mStartTimeUniverseMS = start; mPhoneInterface->syncStartTime(start);};
		void updateDisplay();
		void calcControl();
		void sendControl();	
		void resetErrorMemory();
		void sendStopAndShutdown();
		void sendMotorStart();
		void saveData(string dir, string filename);
		bool loadConfigFromFile(string filename);
		void saveConfigToFile(string filename);
		void clearAllBuffers();
		void resetAll();
		void toggleIbvs(){mPhoneInterface->toggleIbvs();}

		void sendPosControllerGains();

		// from PhoneInterfaceListener
		void onPhoneConnected(){sendPosControllerGains();};

		protected slots:
		void onBtnApply_clicked();
		void onBtnReset_clicked();
		void onBtnLoadFromFile_clicked();
		void onBtnSaveToFile_clicked();
		void onBtnSetXyTarget_clicked();
		void onBtnResetIntegrators_clicked();
		void onRdCntl_clicked();
		void onChkUseFilteredStates_clicked();

	protected:
		FlightMode mFlightMode;
		CommStatus mCommStatus;
		ControlType mControlType;
		double mDeltaT;
		unsigned long mStartTimeUniverseMS, mProfileStartTimeMS;
		bool mInitialized;
		string mName;
		double mChannelMin[4], mChannelMax[4], mChannelZero[4]; // x, y, z
		short mChannelPolarity[4];

		SystemModelQuadrotor *mDynamicModel;
		SystemControllerHierarchical mControllerHierarchical;
		SystemControllerFeedbackLin mControllerFeedbackLin;

		ICSL::Timer mTmr;

		list<TNT::Array2D<double> > mStateRefBuffer, mStateVelRefBuffer;
		list<TNT::Array2D<double> > mErrorMemorySlidingModeBuffer;
		list<TNT::Array2D<double> > mControlBuffer;
		list<unsigned long> mTimeBuffer;
		list<int> mFlightModeBuffer;

		TNT::Array2D<double> mProfileStartPoint;

		void sendControlCommands(TNT::Array2D<double> cntl);
		TNT::Array2D<double> convertTelemViconToState(ICSL::Quadrotor::TelemetryViconDataRecord const &rec);
//		TNT::Array2D<double> estimateState(TelemetryViconDataRecord const &rec, list<TNT::Array2D<double> > const & stateBuffer, double deltaT);
//		TNT::Array2D<double> estimateState(TNT::Array2D<double> &state, list<TNT::Array2D<double> > const & stateBuffer, double deltaT);


		double constrain(double val, double minVal, double maxVal);
		double roundToPrecision(double val, int precision);
		void populateConfigTree();
		void formatTree(QTreeWidgetItem *root);
		void applyControlConfig(QTreeWidgetItem *root);
		void applyLimitsConfig(QTreeWidgetItem *root);
		void sendParamsToPhone();
		static bool receive(Socket::ptr socket, tbyte* data, int size);

		System mSys;

		PhoneInterface *mPhoneInterface;

		toadlet::egg::Mutex mMutex_buffers, mMutex_dataAccess;
};
}
}

#endif
