#ifndef ICSL_SENSORMANAGERLISTENER
#define ICSL_SENSORMANAGERLISTENER

//#if defined ICSL_OBSERVER_SIMULATION || defined ICSL_SENSORMANAGERLISTENER_ONLY
#include <memory>
#include <Data.h>
namespace ICSL{
namespace Quadrotor{
//using namespace std;
class SensorManagerListener
{
	public:
	virtual void onNewSensorUpdate(const shared_ptr<IData> &data)=0;
};

}}

#endif

#if !defined ICSL_OBSERVER_SIMULATION && !defined ICSL_SENSORMANAGERLISTENER_ONLY
#ifndef ICSL_SENSOR_MANAGER
#define ICSL_SENSOR_MANAGER

#include <sched.h>
#include <thread>
#include <mutex>

#include <memory>
#include <string>
#include <fstream>
#include <sstream>

#include <android/sensor.h>

#include <toadlet/egg.h>

#include "TNT/tnt.h"

#include "constants.h"

#include "Common.h"
#include "QuadLogger.h"
#include "Time.h"
//#define ICSL_OBSERVER_ANGULAR_LISTENER_ONLY
#include "Observer_Angular.h"
//#undef ICSL_OBSERVER_ANGULAR_LISTENER_ONLY

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

namespace ICSL{
namespace Quadrotor{
//using namespace std;
static const int ASENSOR_TYPE_PRESSURE=6; // not yet defined for NDK

class SensorManager : public Observer_AngularListener
{
	public:
	SensorManager();
	virtual ~SensorManager();

	void start(){ thread th(&SensorManager::run, this); th.detach(); }
	void run();
	void initialize();
	void shutdown();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setStartTime(Time time){mMutex_startTime.lock(); mStartTime.setTime(time); /*mTimestampOffsetNS = 0;*/ mMutex_startTime.unlock();}

	void addListener(SensorManagerListener *l){mMutex_listeners.lock(); mListeners.push_back(l); mMutex_listeners.unlock();}

	// used to pass images in from Java
	void passNewImage(const cv::Mat *image, int64 timestampNS);

	void setObserverAngular(Observer_Angular *obsv){mObsvAngular = obsv;}

	// for Observer_AngularListener
	void onObserver_AngularUpdated(shared_ptr<DataVector<double> > attData, shared_ptr<DataVector<double> > angularVelData);

	protected:
	bool mRunning, mDone;
	ASensorManager* mSensorManager;
	ASensorEventQueue* mSensorEventQueue;
	const ASensor *mAccelSensor, *mGyroSensor, *mMagSensor, *mPressureSensor;

	static int getBatteryTemp();
	static int getSecTemp();
	static int getFuelgaugeTemp();
	void runTemperatureMonitor();

	QuadLogger *mQuadLogger;

	TNT::Array2D<double> mLastAccel, mLastGyro, mLastMag;
	double mLastPressure;
	mutex mMutex_data, mMutex_listeners, mMutex_startTime, mMutex_attData;
	std::mutex mMutex_logger;

	Time mStartTime;

	Collection<SensorManagerListener *> mListeners;

	TNT::Array2D<double> mCurAtt, mCurAngularVel;

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	int64_t mTimestampOffsetNS;
	int mThreadPriority, mScheduler;

	Time mLastImageTime;
	double mImageDT;
	int mImageCnt;

	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;

	Observer_Angular *mObsvAngular;
};


} // namespace Quadrotor
} // namespace ICSL
#endif // ICSL_OBSERVER_SIMULATION
#endif
