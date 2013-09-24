#ifndef ICSL_OBSERVER_TRANSLATIONAL
#define ICSL_OBSERVER_TRANSLATIONAL
#include <memory>
#include <fstream>
#include <sched.h>
#include <thread>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "TNT/tnt.h"

#include "constants.h"

#include "Data.h"
#include "Time.h"
#include "CommManager.h"
#include "QuadLogger.h"
#define ICSL_OBSERVER_ANGULAR_LISTENER_ONLY
#include "Observer_Angular.h"
#undef ICSL_OBSERVER_ANGULAR_LISTENER_ONLY
#include "SensorManager.h"
#define ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
#include "VelocityEstimator.h"
#undef ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY

#define ICSL_TARGETFINDER_LISTENER_ONLY
#include "TargetFinder.h"
#undef ICSL_TARGETFINDER_LISTENER_ONLY

#include "toadlet/egg.h"

namespace ICSL{
namespace Quadrotor{
class Observer_TranslationalListener
{
	public:
	Observer_TranslationalListener(){};
	virtual ~Observer_TranslationalListener(){};

	virtual void onObserver_TranslationalUpdated(const TNT::Array2D<double> &pos, const TNT::Array2D<double> &vel)=0;
};

class Observer_Translational : public Observer_AngularListener,
								public CommManagerListener,
								public SensorManagerListener,
								public VelocityEstimatorListener,
								public TargetFinderListener
{
	public:
	Observer_Translational();
	virtual ~Observer_Translational();

	void initialize();
	void start(){ thread th(&Observer_Translational ::run, this); th.detach(); }
	void run();
	void shutdown();

	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void setStartTime(Time t);
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setRotViconToPhone(const TNT::Array2D<double> &rot){mRotViconToPhone.inject(rot);}

	void addListener(Observer_TranslationalListener *listener){mListeners.push_back(listener);}

	TNT::Array2D<double> estimateStateAtTime(const Time &t);
	TNT::Array2D<double> estimateErrCovAtTime(const Time &t);

	bool isTargetFound(){return mHaveFirstCameraPos;}

	// from Observer_AngularListener
	void onObserver_AngularUpdated(const shared_ptr<SO3Data<double>> &attData, const shared_ptr<DataVector<double>> &angularVelData);

	// from CommManagerListener
	void onNewCommStateVicon(const toadlet::egg::Collection<float> &data);
	void onNewCommKalmanMeasVar(const toadlet::egg::Collection<float> &var);
	void onNewCommKalmanDynVar(const toadlet::egg::Collection<float> &var);
	void onNewCommUseIbvs(bool useIbvs);
	void onNewCommAccelBias(float xBias, float yBias, float zBias);
	void onNewCommViconCameraOffset(float x, float y, float z);
	void onNewCommTargetNominalLength(float length);
	void onNewCommMAPHeightMeasCov(float cov);

	// for SensorManagerListener
	void onNewSensorUpdate(const shared_ptr<IData> &data);

	// for VelocityEstimatorListener
	void onVelocityEstimator_newEstimate(const shared_ptr<DataVector<double>> &velData,
										 const shared_ptr<Data<double>> &heightData);

	// for TargetFinderListener
	void onTargetFound(const shared_ptr<ImageTargetFindData> &data);

	protected:
	bool mRunning, mDone;
	bool mDoMeasUpdate;
	bool mNewViconPosAvailable, mNewCameraPosAvailable;
	bool mUseViconPos, mUseCameraPos;
	bool mHaveFirstVicon;
	Time mStartTime;

	double mMAPHeightMeasCov;
	double mTargetNominalLength;
	TNT::Array2D<double> mViconCameraOffset;

	TNT::Array2D<double> mRotViconToPhone;
	QuadLogger *mQuadLogger;

	toadlet::egg::Collection<Observer_TranslationalListener*> mListeners;

	// for the translational Kalman Filter
	TNT::Array2D<double> mMeasCov, mPosMeasCov, mVelMeasCov; 
	TNT::Array2D<double> mDynCov, mErrCovKF;
	// State vector
	// 0. x
	// 1. y
	// 2. z
	// 3. x vel
	// 4. y vel
	// 5. z vel
	// 6. x accel bias
	// 7. y accel bias
	// 8. z accel bias
	TNT::Array2D<double> mStateKF;
	TNT::Array2D<double> mAccelBiasReset;

	std::mutex mMutex_events;
	std::mutex mMutex_kfData;
	std::mutex mMutex_accel, mMutex_gravDir;

	static void doTimeUpdateKF(const TNT::Array2D<double> &accel, 
							   double dt,
							   TNT::Array2D<double> &state,
							   TNT::Array2D<double> &errCov,
							   const TNT::Array2D<double> &dynCov);
	static void doMeasUpdateKF_velOnly(const TNT::Array2D<double> &meas,
									   const TNT::Array2D<double> &measCov,
									   TNT::Array2D<double> &state,
									   TNT::Array2D<double> &errCov);
	static void doMeasUpdateKF_posOnly(const TNT::Array2D<double> &meas,
									   const TNT::Array2D<double> &measCov,
									   TNT::Array2D<double> &state,
									   TNT::Array2D<double> &errCov);
	static void doMeasUpdateKF_xyOnly(const TNT::Array2D<double> &meas,
									   const TNT::Array2D<double> &measCov,
									   TNT::Array2D<double> &state,
									   TNT::Array2D<double> &errCov);
	static void doMeasUpdateKF_heightOnly(double meas, 
											    double measCov, 
											    TNT::Array2D<double> &state, 
											    TNT::Array2D<double> &errCov);

	SO3 mRotCamToPhone, mRotPhoneToCam;
	SO3 mCurAtt;
	std::mutex mMutex_att;

	int mThreadPriority, mScheduler;

	vector<list<shared_ptr<IData>> *> mDataBuffers;
	list<shared_ptr<DataVector<double>>> mStateBuffer, mErrCovKFBuffer, mViconPosBuffer, mCameraPosBuffer;
	list<shared_ptr<DataVector<double>>> mViconVelBuffer, mCameraVelBuffer, /*mOpticFlowVelBuffer,*/ mMapVelBuffer;
	list<shared_ptr<Data<double>>> /*mHeightDataBuffer,*/ mMapHeightBuffer;
	list<shared_ptr<HeightData<double>>> mHeightDataBuffer;
	list<shared_ptr<DataVector<double>>> mRawAccelDataBuffer, mGravityDirDataBuffer;
	list<shared_ptr<IData>> mNewEventsBuffer;

	bool mHaveFirstCameraPos;
	Time mLastCameraPosTime, mLastViconPosTime;
	std::mutex mMutex_posTime;

	bool mUseIbvs;

	Time applyData(list<shared_ptr<IData>> &events);
};

} // namespace Quadrotor
} // namespace ICSL

#endif
