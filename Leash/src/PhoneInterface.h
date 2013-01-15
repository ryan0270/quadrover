#ifndef CLASS_PHONEINTERFACE
#define CLASS_PHONEINTERFACE

#include "toadlet/egg.h"
using toadlet::int64;
using toadlet::uint64;

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
#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"

#include "Quadrotor/quadrotor_config.h"
#include "Quadrotor/quadrotorConstants.h"

#include "ui_PhoneInterface.h"
#include "../../QuadPhone/cpp/Common.h"

namespace ICSL{
namespace Quadrotor {
using namespace std;

class PhoneInterfaceListener
{
	public:
	virtual void onPhoneConnected(){};
};

class PhoneInterface : public QWidget, protected Ui::PhoneInterface
{
	Q_OBJECT

	public:
		explicit PhoneInterface(QWidget *parent=0);
		virtual ~PhoneInterface();

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

		void setDeltaT(double dt){mDeltaT = dt;}
		void setForceScaling(double k){mForceScaling = k;}
		void setTorqueScaling(double k){mTorqueScaling = k;}
		void setMass(double m){mMass = m;}

		bool isUsingIbvs(){return chkUseIbvsController->isChecked();}

		toadlet::egg::Collection<double> getDesiredImageMoment(double z);
		double getDesiredImageArea(){
			mMutex_data.lock();
			double area = mDesiredStateImage[2]; 
			mMutex_data.unlock(); 
			return area;}

		void toggleIbvs();

		void syncStartTime(unsigned long start){mStartTimeUniverseMS = start; if(mSocketTCP!=NULL && mSocketTCP->connected()) onBtnSyncTime_clicked();};
		bool loadConfigFromFile(string filename);
		void saveConfigToFile(string filename);

		void addListener(PhoneInterfaceListener *l){mListeners.push_back(l);};

	protected slots:
		void onBtnApply_clicked();
		void onBtnResetConfig_clicked();
		void onBtnLoadFromFile_clicked();
		void onBtnSaveToFile_clicked();
		void onBtnConnect_clicked();
		void onBtnSendParams_clicked();
		void onBtnResetObserver_clicked();
		void onBtnSyncTime_clicked();
		void onBtnRequestLogFile_clicked();
		void onBtnSendMuCntl_clicked();
		void onBtnClearLog_clicked();
		void onChkUseMuCntl_clicked();
		void onChkViewBinarizedImage_clicked();
		void onChkUseIbvsController_clicked();
		void onBtnResetDesImgMoment_clicked();
		void onBtnConfirmDesImgMoment_clicked();
		void onBtnSetYawZero_clicked();

	protected:
		bool mIsConnected;
		string mIP;
		int mPort;
		Socket::ptr mSocketUDP, mSocketTCP;
		double mGainP[3], mGainI[3], mGainD[3], mObserverGainP, mObserverGainI;
		double mGravBandwidth;
		double mDeltaT, mForceScaling, mTorqueScaling, mMass;
		Collection<double> mObserverWeights;
		int mArduinoStatus;
		toadlet::egg::Collection<double> mIbvsGainImg, mIbvsGainFlowInt, mIbvsGainFlow, mIbvsGainAngularRate, mAttCmdOffset, mIbvsGainFF;
		double mIbvsGainAngle, mIbvsGainDynamic;
		TNT::Array2D<double> mState, mDesiredState, mGyro, mAccel, mBias, mComp;
		toadlet::egg::Collection<float> mStateImage, mDesiredStateImage;
		TNT::Array2D<double> mIntMemory;
		int mMotorValues[4], mMotorValuesIbvs[4];
		bool mUseMotors;
		uint64 mTimeMS;
		int mMotorTrim[4];

		System mSys;
		unsigned long mStartTimeUniverseMS;

		toadlet::egg::Mutex mMutex_socketUDP, mMutex_socketTCP;
		toadlet::egg::Mutex mMutex_data, mMutex_image;

		int mCurCntlType, mImgViewType;
		string mCntlSysFile;
		uint32 mCntlCalcTimeUS, mImgProcTimeUS;

		void applyCommConfig(QTreeWidgetItem *root);
		void applyMotorConfig(QTreeWidgetItem *root);
		void applyControlConfig(QTreeWidgetItem *root);
		void applyControllerSysConfig(QTreeWidgetItem *root);
		void applyObserverConfig(QTreeWidgetItem *root);
		void applyIbvsConfig(QTreeWidgetItem *root);
		void applyKalmanFilterConfig(QTreeWidgetItem *root);
		void applyLogConfig(QTreeWidgetItem *root);
		void populateConfigTree();
		void formatTree(QTreeWidgetItem *root);
		int receiveUDP(Socket::ptr socket, tbyte* data, int size);
		int receiveTCP(Socket::ptr socket, tbyte* data, int size);
		bool receivePacket(Socket::ptr socket, Packet &pck, int size);
		void receiveLogFile(Socket::ptr socket, string filename);

		bool mUseIbvs;
		toadlet::egg::Collection<int> mFiltBoxColorMin, mFiltBoxColorMax;
		int mFiltSatMin, mFiltSatMax;
		int mFiltValMin, mFiltValMax;
		int mFiltCircMin, mFiltCircMax;
		int mFiltConvMin, mFiltConvMax;
		int mFiltAreaMin, mFiltAreaMax;
		double mDesiredHeight;

		TNT::Array2D<double> mAttBias;
		double mAttBiasGain, mForceScalingGain;

		float mKfPosMeasStdDev, mKfVelMeasStdDev;

		Collection<float> mGainCntlSys;

		uint32 mLogMask;

		cv::Mat mLastImage;
//		cv::Mat processImage(const cv::Mat &img);
		QImage cvMat2QImage(const cv::Mat &mat);
		static bool compareCirclesByRadius(cv::Vec3f c1, cv::Vec3f c2){return (c1[2] < c2[2]);}

		void populateImgProcParams();

		toadlet::egg::Collection<PhoneInterfaceListener*> mListeners;
};
}
}

#endif
