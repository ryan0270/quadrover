#ifndef ICSL_OBSERVER_ANGULAR 
#define ICSL_OBSERVER_ANGULAR 
#include <algorithm>
#include <android/sensor.h>

#include <toadlet/egg.h>

#include "ICSL/constants.h"
#include "TNT/tnt.h"
#include "QuadLogger.h"
#include "Common.h"
#include "Time.h"
#include "CommManager.h"

using toadlet::uint64;

namespace ICSL{
namespace Quadrotor{

class Observer_AngularListener
{
	public:
	virtual ~Observer_AngularListener(){};

	virtual void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel)=0;
};

//class Observer_Angular : public InputDeviceListener
class Observer_Angular : public toadlet::egg::Thread, public CommManagerListener
{
	public:
		Observer_Angular();
		virtual ~Observer_Angular();

		void doInnovationUpdate(double dt);
		void doGyroUpdate(double dt);

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

		void setGainP(double p){mMutex_all.lock(); mGainP = p; mMutex_all.unlock();};
		void setGainI(double i){mMutex_all.lock(); mGainI = i; mMutex_all.unlock();};
		void setWeights(double accelWeight, double magWeight);
		void setStartTime(Time time){mStartTime = time;}
		void setYawZero();

		void reset();
		void doBurnIn(uint64 periodMS){mBurnInPeriodMS = periodMS; mBurnInStartTime.setTime();}
		bool doingBurnIn(){return mBurnInStartTime.getMS()+mBurnInPeriodMS > Time::nowMS();}
		
		void addListener(Observer_AngularListener* chad){mListeners.push_back(chad);};

		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}

		void addDirectionMeasurement(TNT::Array2D<double> const &dirMeas, TNT::Array2D<double> const &dirInertial, double weight);

		void enableViconAttitude(bool val){mUseViconAttitude = val;}

		void initialize();
		void run();
		void shutdown();

		// CommManagerListener functions
		void onNewCommObserverReset();
		void onNewCommObserverGain(double gainP, double gainI, double gravWeight, double compWeight, double gravBandwidth);
		void onNewCommSetYawZero();
		void onNewCommStateVicon(toadlet::egg::Collection<float> const &data);

	protected:
		bool mRunning, mDone;
		bool mUseViconAttitude;
		double mGainP, mGainI, mGainB, mIntSat;
		double mAccelWeight, mMagWeight;
		TNT::Array2D<double> mGyroBias, mGyroBiasFirst, mInnovation;
		TNT::Array2D<double> mCurAttitude, mCurRotMat, mCurVel;
		TNT::Array2D<double> mLastAccel, mLastGyro, mLastMagnometer;
		TNT::Array2D<double> mAccelDirNom, mMagDirNom;
//		Collection<TNT::Array2D<double> > mDirsInertial;
		Collection<TNT::Array2D<double> > mExtraDirsMeasured, mExtraDirsInertial;
		Collection<double> mExtraDirsWeight;

		TNT::Array2D<double> convert_so3toSO3(TNT::Array2D<double> const &so3);
		TNT::Array2D<double> convert_SO3toso3(TNT::Array2D<double> const &SO3);

		TNT::Array2D<double> extractEulerAngles(TNT::Array2D<double> const &rotMat);

		Time mLastUpdateTime, mStartTime, mBurnInStartTime;
		Time mLastBiasUpdateTime; // bias update happen with the accel/mag which have different update rates than the gyro
		uint64 mBurnInPeriodMS;
		int mBurnCount;
		toadlet::egg::Mutex mMutex_all;

//		void inputDetected(const InputData &data);
		static int inputDetected(int fd, int events, void *data);

		Collection<Observer_AngularListener*> mListeners;

		QuadLogger *mQuadLogger;

		ASensorManager* mSensorManager;
		ASensorEventQueue* mSensorEventQueue;
		const ASensor *mAccelSensor, *mGyroSensor, *mMagSensor, *mRotationSensor;

		TNT::Array2D<double> mAttitudeVicon;
};

}
}

#endif
