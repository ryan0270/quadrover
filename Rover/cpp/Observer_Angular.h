// Separate this out to avoid some circular include problems
#ifndef ICSL_OBSERVER_ANGULAR_LISTENER
#include <memory>
#include "Data.h"
#include "TNT/tnt.h"

namespace ICSL{
namespace Quadrotor{
//using namespace std;
class Observer_AngularListener
{
	public:
	virtual ~Observer_AngularListener(){};

	virtual void onObserver_AngularUpdated(shared_ptr<DataVector<double> > attData, shared_ptr<DataVector<double> > angularVelData)=0;
};
}}
#define ICSL_OBSERVER_ANGULAR_LISTENER
#endif

#ifndef ICSL_OBSERVER_ANGULAR_LISTENER_ONLY
#ifndef ICSL_OBSERVER_ANGULAR 
#define ICSL_OBSERVER_ANGULAR 
#include <memory>
#include <sched.h>
#include <thread>
#include <mutex>

#include "Data.h"
#include "constants.h"
#include "TNT/tnt.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Time.h"
#include "CommManager.h"
#include "SensorManager.h"
#include "Rotation.h"

#include <toadlet/egg.h>

namespace ICSL{
namespace Quadrotor{

//class Observer_Angular : public InputDeviceListener
class Observer_Angular : public CommManagerListener,
						 public SensorManagerListener
{
	public:
		Observer_Angular();
		virtual ~Observer_Angular();

		void doInnovationUpdate(double dt, shared_ptr<DataVector<double> > const &accelData, shared_ptr<DataVector<double> > const &magData);
		void doGyroUpdate(double dt, shared_ptr<DataVector<double> > const &gyroData);

		/*
		 * @return current attitude in [roll pitch yaw]^T column vector format
		 */
		TNT::Array2D<double> getCurAttitude();
		TNT::Array2D<double> getCurVel();
		TNT::Array2D<double> getBias();
		TNT::Array2D<double> getLastGyro();
		TNT::Array2D<double> getLastAccel();
		TNT::Array2D<double> getLastMagnometer();
		double getGainP(){return mGainP;};
		double getGainI(){return mGainI;};

		void setGainP(double p){mMutex_data.lock(); mGainP = p; mMutex_data.unlock();};
		void setGainI(double i){mMutex_data.lock(); mGainI = i; mMutex_data.unlock();};
		void setWeights(double accelWeight, double magWeight);
		void setStartTime(Time time){mStartTime = time;}
		void setYawZero();

		void reset();
		bool doingBurnIn(){return mDoingBurnIn;}
		
		void addListener(Observer_AngularListener* chad){mListeners.push_back(chad);};

		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

		void addDirectionMeasurement(TNT::Array2D<double> const &dirMeas, TNT::Array2D<double> const &dirInertial, double weight);

		void initialize();
		void start(){ thread th(&Observer_Angular::run, this); th.detach(); }
		void run();
		void shutdown();

		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

		SO3 estimateAttAtTime(Time const &t);

		// CommManagerListener functions
		void onNewCommObserverReset();
		void onNewCommAttObserverGain(double gainP, double gainI, double accelWeight, double magWeight);
		void onNewCommNominalMag(toadlet::egg::Collection<float> const &nomMag);
		void onNewCommStateVicon(toadlet::egg::Collection<float> const &data);
		
		// for SensorManagerListener
		void onNewSensorUpdate(shared_ptr<IData> const &data);

	protected:
		bool mRunning, mDone;
		bool mNewAccelReady, mNewGyroReady, mNewMagReady;
		bool mDoingBurnIn;
		double mGainP, mGainI, mGainB, mIntSat;
		double mAccelWeight, mMagWeight;
		TNT::Array2D<double> mGyroBias, mInnovation;
		TNT::Array2D<double> mCurAttitude, mCurRotMat, mCurVel;
//		TNT::Array2D<double> mAccel, mGyro, mMagnometer;
		shared_ptr<DataVector<double> >  mAccelData, mGyroData, mMagData; // use this for copying data from SensorManager updates
		TNT::Array2D<double> mAccelDirNom, mMagDirNom;
		toadlet::egg::Collection<TNT::Array2D<double> > mExtraDirsMeasured, mExtraDirsInertial;
		toadlet::egg::Collection<double> mExtraDirsWeight;

		TNT::Array2D<double> convert_so3toCoord(TNT::Array2D<double> const &so3);
		TNT::Array2D<double> convert_coordToso3(TNT::Array2D<double> const &SO3);

		TNT::Array2D<double> extractEulerAngles(TNT::Array2D<double> const &rotMat);

		Time mStartTime;
		int mBurnCount;
		std::mutex mMutex_data, mMutex_cache;

//		static int inputDetected(int fd, int events, void *data);

		toadlet::egg::Collection<Observer_AngularListener*> mListeners;

		QuadLogger *mQuadLogger;

		double mYawVicon;

		int mThreadPriority, mScheduler;

		list<shared_ptr<SO3Data<double>>> mSO3Buffer;
		std::mutex mMutex_SO3Buffer;
};

}
}

#endif
#endif
