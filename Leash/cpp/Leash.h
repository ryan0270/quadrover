#ifndef CLASS_PHONEINTERFACE
#define CLASS_PHONEINTERFACE

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "toadlet/egg.h"

#include <fstream>
#include <iostream>
#include <list>
#include <vector>
#include <random>

#include <QObject>
#include <QWidget>
#include <QtGui>

#include "TNT/tnt.h"

#include "ICSL/icsl_config.h"
#include "ICSL/constants.h"

#include "ICSL/xml_utils/xml_utils.h"
#include "ICSL/TNT_Utils/TNT_Utils.h"
#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"

#include "ui_Leash.h"
#include "../../Rover/cpp/Common.h"
#include "TelemetryVicon/src/TelemetryVicon.h"

namespace ICSL{
namespace Quadrotor {
using namespace std;

enum LogType
{
	LOG_TYPE_UNKNOWN=0,
	LOG_TYPE_VICON_STATE,
};

class LogItem
{
	public:
		LogItem(){toadlet::egg::System sys; time = sys.mtime(); line = ""; type = LOG_TYPE_UNKNOWN;}
		LogItem(uint64_t time, toadlet::egg::String s, LogType t){this->time = time; line = s; type = t;}

		uint64_t time;
		String line;
		LogType type;
};

class Leash : public QMainWindow, public TelemetryViconListener
{
	Q_OBJECT

	public:
		explicit Leash(QWidget *parent=0);
		virtual ~Leash();

		void initialize();
		void run(); // this is for Qt's startup, not a toadlet thread
		void shutdown();
		void pollUDP();
		void pollTCP();
		
		bool sendUDP(tbyte* data, int size);
		bool sendTCP(tbyte* data, int size);
		bool sendMotorStart();
		bool sendMotorStop(bool warnIfDisconnected);
		bool sendParams();
		bool isConnected(){return mIsConnected;};

		void toggleIbvs();

		void syncStartTime(unsigned long start){mStartTimeUniverseMS = start; if(mSocketTCP!=NULL) onBtnSyncTime_clicked();};
		bool loadConfigFromFile(string filename);
		void saveConfigToFile(string filename);

		void clearLogBuffer(){mLogData.clear();}
		void saveLogData(string dir, string filename);

		void sendDesiredState();
		
		// for TelemetryViconListener
		void onTelemetryUpdated(TelemetryViconDataRecord const &rec);

	protected slots:
		void onBtnApply_clicked();
		void onBtnResetConfig_clicked();
		void onBtnLoadConfig_clicked();
		void onBtnSaveConfig_clicked();
		void onBtnConnect_clicked();
		void onBtnSendParams_clicked();
		void onBtnResetObserver_clicked();
		void onBtnSyncTime_clicked();
		void onBtnGetPhoneLog_clicked();
		void onBtnClearPhoneLog_clicked();
		void onBtnClearLocalLog_clicked(){clearLogBuffer();}
		void onChkViewBinarizedImage_clicked();
		void onChkUseIbvsController_clicked();
		void onBtnSetDesiredPos_clicked();

		void updateDisplay();
		void onBtnStartMotors_clicked();
		void onBtnStopMotors_clicked();
		void onBtnQuit_clicked();
		void onIncreaseHeight();
		void onDecreaseHeight();
		void onMoveLeft();
		void onMoveRight();
		void onMoveForward();
		void onMoveBackward();
		void onToggleIbvs();

	protected:
		Ui::Leash *ui;
		bool mIsConnected;
		bool mNewViconDataReady;
		bool mFirstDraw;
		string mIP;
		int mPort;
		Socket::ptr mSocketUDP, mSocketTCP;
		float mAttObsvGainP, mAttObsvGainI;
		float mCntlGainTransP[3], mCntlGainTransD[3], mCntlGainTransI[3], mCntlGainTransILimit[3];
		float mCntlGainIbvsP[3], mCntlGainIbvsD[3];
		float mCntlGainAttP[3], mCntlGainAttD[3];
		float mMotorForceGain, mMotorTorqueGain, mMotorArmLength, mTotalMass;
		Collection<float> mAttObsvDirWeights;
		Collection<float> mAttObsvNominalMag;
		int mMotorValues[4];
		uint64_t mTimeMS;
		int mMotorTrim[4];

		System mSys;
		unsigned long mStartTimeUniverseMS;

		toadlet::egg::Mutex mMutex_socketUDP, mMutex_socketTCP, mMutex_logBuffer;
		toadlet::egg::Mutex mMutex_data;

		void populateUI();

		int receiveUDP(Socket::ptr socket, tbyte* data, int size);
		int receiveTCP(Socket::ptr socket, tbyte* data, int size);
		bool receivePacket(Socket::ptr socket, Packet &pck, int size);
		void receiveLogFile(Socket::ptr socket, string filename);

		bool mUseIbvs;

		Collection<float> mAccelBias, mAccelBiasDynVar;
		Collection<float> mKalmanMeasVar, mKalmanDynVar;

		uint32_t mLogMask;

		QImage cvMat2QImage(const cv::Mat &mat);

		TNT::Array2D<float> mState, mDesState, mViconState;
		QList<QStandardItem*> mStateData;
		QList<QStandardItem*> mDesStateData;
		QList<QStandardItem*> mViconStateData;
		QList<QStandardItem*> mGyroData, mGyroBiasData;
		QList<QStandardItem*> mAccelData;
		QList<QStandardItem*> mMagData;
		QList<QStandardItem*> mPosIntData, mTorqueIntData;
		QList<QStandardItem*> mMotorData;

		void populateControlUI();
		void loadControllerConfig(mxml_node_t *cntlRoot);
		void saveControllerConfig(mxml_node_t *cntlRoot);
		void applyControllerConfig();

		void populateObserverUI();
		void loadObserverConfig(mxml_node_t *obsvRoot);
		void saveObserverConfig(mxml_node_t *obsvRoot);
		void applyObserverConfig();

		void populateHardwareUI();
		void loadHardwareConfig(mxml_node_t *hdwRoot);
		void saveHardwareConfig(mxml_node_t *hdwRoot);
		void applyHardwareConfig();

		void populateDataLoggingUI();
		void applyDataLoggingConfig();

		void populateVisionUI();
		void loadVisionConfig(mxml_node_t *hdwRoot);
		void saveVisionConfig(mxml_node_t *hdwRoot);
		void applyVisionConfig();

		int mImgBufferSize;

		static void resizeTableWidget(QTableWidget *tbl); // make the table widget match the cell size (not the same as having cells match their contents)
		static void resizeTable(QTableView *tbl); // make the table widget match the cell size (not the same as having cells match their contents)
		void setVerticalTabOrder(QTableWidget *tbl);

		list<LogItem> mLogData;

		QTimer *mTmrGui;
		TelemetryVicon mTelemVicon;

		QShortcut *mScStartMotors, *mScStopMotors, *mScQuit;
		QShortcut *mScIncreaseHeight, *mScDecreaseHeight, *mScMoveLeft, *mScMoveRight, *mScMoveForward, *mScMoveBackward;
		QShortcut *mScToggleIbvs;

		// Vision params
		float mFeatureFindQualityLevel, mVelEstVisionMeasCov, mVelEstProbNoCorr, mFeatureFindFASTAdaptRate;
		int mFeatureFindSeparationDistance, mFeatureFindFASTThreshold, mFeatureFindPointCountTarget;

		list<float> mImgProcTimeBuffer;

		TNT::Array2D<float> mRotViconToQuad, mRotQuadToVicon;
		TNT::Array2D<float> mRotQuadToPhone, mRotPhoneToQuad;
		TNT::Array2D<float> mRotViconToPhone, mRotPhoneToVicon;

		uint64_t mLastTelemSendTime;

		default_random_engine mRandGenerator;
		normal_distribution<float> mStdGaussDist;

		void pingRover();

		string mCntlSysFile;
		void sendTransSystemController();
};
}
}

#endif
