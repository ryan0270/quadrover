#ifndef ICSL_CLASS_QUADTELEMETRY
#define ICSL_CLASS_QUADTELEMETRY

#include <vector>
#include <queue>
#include <string> 
#include <exception> 

#include <QtGui>

#include "toadlet/egg.h"

#include "TNT/tnt.h"

#include "ICSL/Timer/src/Timer.h"
#include "ICSL/MeasurementSystems/src/MeasurementSystem_Vicon.h"

namespace ICSL{
namespace Quadrotor{
using namespace std; 

class TelemetryViconException : public exception 
{ 
	public: 
		TelemetryViconException(){mMessage = "TelemetryVicon exception occurred";}; 
		TelemetryViconException(string msg){mMessage = msg;}; 
		~TelemetryViconException() throw(){};
		const char* what() const throw() {return mMessage.c_str();}; 
	
	protected: 
		string mMessage; 
}; 

struct TelemetryViconDataRecord
{
	float x, y, z;
	float roll, pitch, yaw;

	unsigned long time;
	bool isEmpty, invalidRead;
	static const int ITEM_COUNT = 6;
	string objectID;
};

class TelemetryViconListener
{
	public:
		virtual ~TelemetryViconListener(){};
		virtual void onTelemetryUpdated(TelemetryViconDataRecord const &rec)=0;
};

class TelemetryVicon// : public QObject
{ 
//	Q_OBJECT
	public: 
		TelemetryVicon(); 
		virtual ~TelemetryVicon(); 

		void initializeMonitor();
	
		/*! 
		  Attempts to connect to \param portName 
		  \return success of connection attempt 
		  */ 
		bool connect(string source);
		void disconnect();

		/*! 
		  \return updated sensor data after reading the input buffer 
			*/ 
		vector<TelemetryViconDataRecord> getLatestSensorData(); 
		
		/*! 
		  Signals the sensor monitor thread to either awaken or suspend. Note 
		  that this thread is created in a suspended state when the object 
		  is created. 
		  */ 
		void startMonitor();
		void stopMonitor();

		/*! 
		  Syncs the start time of the sensor monitor used 
		  to calculated the timestamps of sensor records 
		  */ 
		void syncStartTimeMS(unsigned long); 

		void setOriginPosition(TNT::Array2D<double> const &origin);

		void setRefreshDelay(unsigned long d){mMonitor.mRefreshDelay = max(d,mMonitor.mPollingDelay);};
		void setPollingDelay(unsigned long d){mMonitor.mPollingDelay = d;};

		void addTrackedQuadrotor(string name);

		int addListener(TelemetryViconListener *chad){mListeners.push_back(chad); return (int)mListeners.size();};

//	signals:
//		void newRecordAvailable(TelemetryViconDataRecord const &rec);

	protected: 
		bool mConnected; 
		int mNumQuads;
		TNT::Array2D<double> mOrigin;

		vector<TelemetryViconListener*> mListeners;
		queue< vector<TelemetryViconDataRecord> > mDataQueue;
		ICSL::MeasurementSystem_Vicon mVicon;

		int mQueueMaxSize; 

		ICSL::Timer mTmr; 

		struct ThreadSignals 
		{ 
			bool beginSuspend; 
			bool close; 
			bool queueInUse;
			bool setHomePos;
			unsigned long startTime; 
		}; 
		ThreadSignals mThreadSignals; 

		class Monitor : public toadlet::egg::Thread 
		{ 
			public: 
				TelemetryVicon *parent; 
				bool mSleeping, mInitialized;
				TNT::Array2D<double> mPosHome;
				unsigned long mRefreshDelay;
				unsigned long mPollingDelay;
			protected: 
				void run(); 
		}; 
		Monitor mMonitor; 
		toadlet::egg::Mutex mMutexDataAccess;
		toadlet::egg::Mutex mMutexSuspendReady, mMutexSuspendAwaken, mMutexRecordReady, mMutexHomePosSet; 
		toadlet::egg::WaitCondition mWCSuspendReady, mWCSuspendAwaken, mWCRecordReady, mWCHomePosSet;
}; 

}
}

// Q_DECLARE_METATPYE(Quadrotor::TelemetryViconDataRecord)
// qRegisterMetaType<Quadrotor::TelemetryViconDataRecord>("TelemetryViconDataRecrod");

#endif
