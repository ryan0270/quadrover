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
#include "Observer_Angular.h"
#include "SensorManager.h"
#define ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
#include "VelocityEstimator.h"
#undef ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY

#include "TargetFinder.h"

#include "toadlet/egg.h"

namespace ICSL{
namespace Quadrotor{
class Observer_TranslationalListener
{
	public:
	Observer_TranslationalListener(){};
	virtual ~Observer_TranslationalListener(){};

	virtual void onObserver_TranslationalUpdated(TNT::Array2D<double> const &pos, TNT::Array2D<double> const &vel)=0;
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
	void setRotViconToPhone(TNT::Array2D<double> const &rot){mRotViconToPhone.inject(rot);}

	void addListener(Observer_TranslationalListener *listener){mListeners.push_back(listener);}

	TNT::Array2D<double> estimateStateAtTime(Time const &t);
	TNT::Array2D<double> estimateErrCovAtTime(Time const &t);

	bool isTargetFound(){return mHaveFirstCameraPos;}

	// from Observer_AngularListener
	void onObserver_AngularUpdated(shared_ptr<DataVector<double> > attData, shared_ptr<DataVector<double> > angularVelData);

	// from CommManagerListener
	void onNewCommStateVicon(toadlet::egg::Collection<float> const &data);
	void onNewCommKalmanMeasVar(toadlet::egg::Collection<float> const &var);
	void onNewCommKalmanDynVar(toadlet::egg::Collection<float> const &var);
	void onNewCommUseIbvs(bool useIbvs);
	void onNewCommAccelBias(float xBias, float yBias, float zBias);

	// for SensorManagerListener
	void onNewSensorUpdate(shared_ptr<IData> const &data);

	// for VelocityEstimatorListener
	void onVelocityEstimator_newEstimate(shared_ptr<DataVector<double> > const &velData, shared_ptr<Data<double> > const &heightData);

	// for TargetFinderListener
	void onTargetFound(shared_ptr<ImageTargetFindData> const &data);

	protected:
	bool mRunning, mDone;
	bool mDoMeasUpdate;
	bool mNewViconPosAvailable, mNewCameraPosAvailable;
	bool mUseViconPos, mUseCameraPos;
	bool mHaveFirstVicon;
	Time mStartTime;

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

	static void doTimeUpdateKF(TNT::Array2D<double> const &accel, 
							   double const &dt,
							   TNT::Array2D<double> &state,
							   TNT::Array2D<double> &errCov,
							   TNT::Array2D<double> const &dynCov);
	static void doMeasUpdateKF_velOnly(TNT::Array2D<double> const &meas,
									   TNT::Array2D<double> const &measCov,
									   TNT::Array2D<double> &state,
									   TNT::Array2D<double> &errCov);
	static void doMeasUpdateKF_posOnly(TNT::Array2D<double> const &meas,
									   TNT::Array2D<double> const &measCov,
									   TNT::Array2D<double> &state,
									   TNT::Array2D<double> &errCov);
	static void doMeasUpdateKF_heightOnly(double const &meas, 
										  double const &measCov, 
										  TNT::Array2D<double> &state, 
										  TNT::Array2D<double> &errCov);

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	int mThreadPriority, mScheduler;

	vector<list<shared_ptr<Data<double>>> *> mDataBuffers;
	list<shared_ptr<DataVector<double>>> mStateBuffer, mErrCovKFBuffer, mViconPosBuffer, mCameraPosBuffer;
	list<shared_ptr<DataVector<double>>> mViconVelBuffer, mCameraVelBuffer, mOpticFlowVelBuffer, mMapVelBuffer;
	list<shared_ptr<Data<double>>> /*mHeightDataBuffer,*/ mMapHeightBuffer;
	list<shared_ptr<DataVector<double>>> mRawAccelDataBuffer, mGravityDirDataBuffer;
	list<shared_ptr<IData>> mNewEventsBuffer;

	bool mHaveFirstCameraPos;
	Time mLastCameraPosTime, mLastViconPosTime;
	std::mutex mMutex_posTime;

	bool mUseIbvs;

	TNT::Array2D<double> mViconCameraOffset;

	Time applyData(shared_ptr<IData> const &data);
};

} // namespace Quadrotor
} // namespace ICSL

#endif
