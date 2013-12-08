#ifndef ICSL_SENSOR_MANAGER
#define ICSL_SENSOR_MANAGER
#include <sched.h>
#include <sys/time.h>
#include <sys/resource.h>
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
#include "DataLogger.h"
#include "Time.h"
#include "Observer_Angular.h"
#include "Listeners.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

namespace ICSL{
namespace Quadrotor{
using namespace std;
static const int ASENSOR_TYPE_PRESSURE=6; // not yet defined for NDK

class SensorManager : public CommManagerListener,
					  public SonarListener
{
	public:
	SensorManager();
	virtual ~SensorManager();

	void start(){ thread th(&SensorManager::run, this); th.detach(); }
	void run();
	void initialize();
	void shutdown();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
	void setThreadNice(int nice){mThreadNiceValue = nice;};

	void setDataLogger(DataLogger *log){mDataLogger = log;}
	void setStartTime(Time time){mMutex_startTime.lock(); mStartTime.setTime(time); /*mTimestampOffsetNS = 0;*/ mMutex_startTime.unlock();}

	void addListener(SensorManagerListener *l){mMutex_listeners.lock(); mListeners.push_back(l); mMutex_listeners.unlock();}

	void setObserverAngular(Observer_Angular *obsv){mObsvAngular = obsv;}

	// this comes from jni
	void onNewSonarReading(int heightMM, uint64_t timestampNS);

	// used to pass images in from Java
	void passNewImage(const cv::Mat *image, uint64_t timestampNS);

	// for CommManagerListener
	void onNewCommStateVicon(const toadlet::egg::Collection<float> &data);
	void onNewCommAccelBias(float xBias, float yBias, float zBias);

	// for SonarListener
	void onNewSonar(const shared_ptr<HeightData<double>> &data);

	protected:
	bool mRunning, mDone;
	ASensorManager* mSensorManager;
	ASensorEventQueue* mSensorEventQueue;
	const ASensor *mAccelSensor, *mGyroSensor, *mMagSensor, *mPressureSensor;

	static int getBatteryTemp();
	static int getSecTemp();
	static int getFuelgaugeTemp();
	void runTemperatureMonitor();
	void runHeightMonitor();

	DataLogger *mDataLogger;

	TNT::Array2D<double> mLastAccel, mLastGyro, mLastMag;
	double mLastPressure;
	mutex mMutex_data, mMutex_listeners, mMutex_startTime, mMutex_attData;
	std::mutex mMutex_logger;

	Time mStartTime;

	Collection<SensorManagerListener *> mListeners;

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	int64_t mTimestampOffsetNS, mTimestampOffsetNS_mag; // because on the S4 right now the mag uses epoch time while everything else uses uptime
	int mThreadPriority, mScheduler, mThreadNiceValue;

	Time mLastImageTime;
	double mImageDT;
	int mImageCnt;

	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;

	Observer_Angular *mObsvAngular;

	TNT::Array2D<double> mAccelBias;
	bool mHaveNewParams;

	double mLastHeight;
	bool mNewHeightAvailable;
	std::mutex mMutex_vicon;
};


} // namespace Quadrotor
} // namespace ICSL
#endif // ICSL_OBSERVER_SIMULATION
