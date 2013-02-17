#ifndef CLASS_PHONEINTERFACE
#define CLASS_PHONEINTERFACE

#include "toadlet/egg.h"
//using toadlet::int64;
//using toadlet::uint64;

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <algorithm>

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

#include "ICSL/Timer/src/Timer.h"
#include "ICSL/xml_utils/xml_utils.h"
//#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"

#include "ui_Leash.h"
#include "../../Rover/cpp/Common.h"

namespace ICSL{
namespace Quadrotor {
using namespace std;

class LeashListener
{
	public:
	virtual void onPhoneConnected(){};
};

class Leash : public QWidget
{
	Q_OBJECT

	public:
		explicit Leash(QWidget *parent=0);
		virtual ~Leash();

		void initialize();
		void pollUDP();
		void pollTCP();
		void updateDisplay();
		
		bool sendUDP(tbyte* data, int size);
		bool sendTCP(tbyte* data, int size);
		bool sendMotorStart();
		bool sendMotorStop();
		bool sendParams();
		bool isConnected(){return mIsConnected;};

		void toggleIbvs();

		void syncStartTime(unsigned long start){mStartTimeUniverseMS = start; if(mSocketTCP!=NULL && mSocketTCP->connected()) onBtnSyncTime_clicked();};
		bool loadConfigFromFile(string filename);
		void saveConfigToFile(string filename);

		void addListener(LeashListener *l){mListeners.push_back(l);};

	protected slots:
		void onBtnApply_clicked();
		void onBtnResetConfig_clicked();
		void onBtnLoadConfig_clicked();
		void onBtnSaveConfig_clicked();
		void onBtnConnect_clicked();
		void onBtnSendParams_clicked();
		void onBtnResetObserver_clicked();
		void onBtnSyncTime_clicked();
		void onBtnRequestLogFile_clicked();
//		void onBtnSendMuCntl_clicked();
		void onBtnClearLog_clicked();
//		void onChkUseMuCntl_clicked();
		void onChkViewBinarizedImage_clicked();
		void onChkUseIbvsController_clicked();
		void onBtnResetDesImgMoment_clicked();
		void onBtnConfirmDesImgMoment_clicked();
		void onBtnSetYawZero_clicked();

	protected:
		Ui::Leash *ui;
		bool mIsConnected;
		string mIP;
		int mPort;
		Socket::ptr mSocketUDP, mSocketTCP;
		double mAttObsvGainP, mAttObsvGainI;
		double mCntlGainTransP[3], mCntlGainTransD[3], mCntlGainTransI[3], mCntlGainTransILimit[3];
		double mCntlGainAttP[3], mCntlGainAttD[3];
		double mMotorForceGain, mMotorTorqueGain, mMotorArmLength, mTotalMass;
		Collection<double> mAttObsvDirWeights;
		Collection<double> mAttObsvNominalMag;
		toadlet::egg::Collection<double> mIbvsGainAngularRate;
		double mIbvsGainAngle;
		TNT::Array2D<double> mIntMemory;
		int mMotorValues[4], mMotorValuesIbvs[4];
		uint64 mTimeMS;
		int mMotorTrim[4];

		System mSys;
		unsigned long mStartTimeUniverseMS;

		toadlet::egg::Mutex mMutex_socketUDP, mMutex_socketTCP;
		toadlet::egg::Mutex mMutex_data, mMutex_image;

		uint32 mCntlCalcTimeUS, mImgProcTimeUS;

		void applyCommConfig(QTreeWidgetItem *root);
		void applyMotorConfig(QTreeWidgetItem *root);
		void applyControlConfig(QTreeWidgetItem *root);
//		void applyObserverConfig(QTreeWidgetItem *root);
		void applyIbvsConfig(QTreeWidgetItem *root);
		void applyKalmanFilterConfig(QTreeWidgetItem *root);
		void applyLogConfig(QTreeWidgetItem *root);
		void populateUI();
		void formatTree(QTreeWidgetItem *root);
		int receiveUDP(Socket::ptr socket, tbyte* data, int size);
		int receiveTCP(Socket::ptr socket, tbyte* data, int size);
		bool receivePacket(Socket::ptr socket, Packet &pck, int size);
		void receiveLogFile(Socket::ptr socket, string filename);

		bool mUseIbvs;

		TNT::Array2D<double> mAttBias;
		double mAttBiasGain;

		float mKalmanForceGainAdaptGain;
		Collection<float> mKalmanAttBias, mKalmanAttBiasAdaptGain;
		Collection<float> mKalmanMeasVar, mKalmanDynVar;

		uint32 mLogMask;

		QImage cvMat2QImage(const cv::Mat &mat);

		toadlet::egg::Collection<LeashListener*> mListeners;

		QList<QStandardItem*> mAttData, mPosData, mVelData;
		QList<QStandardItem*> mDesAttData, mDesPosData, mDesVelData;
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

		static void resizeTableWidget(QTableWidget *tbl); // make the table widget match the cell size (not the same as having cells match their contents)
		void setVerticalTabOrder(QTableWidget *tbl);
};
}
}

#endif
