#ifndef ICSL_SENSOR_MANAGER
#define ICSL_SENSOR_MANAGER

#ifdef ICSL_OBSERVER_SIMULATION
#include <memory>
#include <Data.h>
namespace ICSL{
namespace Quadrotor{
//using namespace std;
class SensorManagerListener
{
	public:
	virtual void onNewSensorUpdate(shared_ptr<IData> const &data)=0;
};

}}

#else // ICSL_OBSERVER_SIMULATION

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
#include "Observer_Angular.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

namespace ICSL{
namespace Quadrotor{
//using namespace std;
static const int ASENSOR_TYPE_PRESSURE=6; // not yet defined for NDK

class SensorManagerListener
{
	public:
	virtual void onNewSensorUpdate(shared_ptr<IData> const &data)=0;
};

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
	void passNewImage(cv::Mat const *image, int64 const &timestampNS);

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
//	static int getTmuTemp();
	void runTemperatureMonitor();

	QuadLogger *mQuadLogger;

	TNT::Array2D<double> mLastAccel, mLastGyro, mLastMag;
	double mLastPressure;
	mutex mMutex_data, mMutex_listeners, mMutex_startTime, mMutex_attData;
	std::mutex mMutex_logger;

	Time mStartTime;

	Collection<SensorManagerListener *> mListeners;

	shared_ptr<cv::VideoCapture> initCamera();
//	void runImageAcq(shared_ptr<cv::VideoCapture> cap);
	TNT::Array2D<double> mCurAtt, mCurAngularVel;
	TNT::Array2D<double> mAttAccum, mAngularVelAccum; // accumulators so I can get the average during image acq interval
	int mAttAccumCnt, mAngularVelAccumCnt;
	std::mutex mMutex_accum;

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	int64_t mTimestampOffsetNS;
	int mThreadPriority, mScheduler;

	Time mLastImageTime;
	double mImageDT;
	int mImageCnt;

	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;
};


} // namespace Quadrotor
} // namespace ICSL
#endif // ICSL_OBSERVER_SIMULATION
#endif
