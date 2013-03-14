#ifndef ICSL_SENSOR_DATA
#define ICSL_SENSOR_DATA
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "TNT/tnt.h"
#include "Common.h"
#include "Time.h"
#include "TNT_Utils.h"
namespace ICSL{ namespace Quadrotor{
enum ImageFormat
{
	IMG_FORMAT_BGR=1,
	IMG_FORMAT_RGB,
	IMG_FORMAT_HSV,
	IMG_FORMAT_GRAY,
};

class SensorData
{
	public:
	SensorData(){type = SENSOR_DATA_TYPE_UNDEFINED;}
	SensorData(double d, SensorDataType t){data = d; type = t;}

	virtual void lock(){mMutex.lock();}
	virtual void unlock(){mMutex.unlock();}

	virtual void copyTo(SensorData &d) {
		if(&d == this)
			return;
		lock();
		d.lock();
		d.timestamp.setTime(timestamp); 
		d.data = data; d.type = type;
		d.unlock();
		unlock();
	}
	double data;
	Time timestamp;
	SensorDataType type;

	protected:
	toadlet::egg::Mutex mMutex;
};

class SensorDataVector : public SensorData
{
	public:
	SensorDataVector(){type = SENSOR_DATA_TYPE_UNDEFINED;};
	SensorDataVector(TNT::Array2D<double> const &d, SensorDataType t){data = d.copy(); type = t;}

	void copyTo(SensorDataVector &d) {
		if(&d == this)
			return;
		lock();
		d.lock();
		d.timestamp.setTime(timestamp); 
		d.data = data.copy(); 
		d.type = type;
		d.unlock();
		unlock();
	}
	TNT::Array2D<double> data;
};

class SensorDataImage : public SensorData
{
	public:
	SensorDataImage() : att(3,1,0.0), startAngularVel(3,1,0.0), endAngularVel(3,1,0.)  {
		type = SENSOR_DATA_TYPE_IMAGE; 
		imgFormat = IMG_FORMAT_BGR; 
		img = shared_ptr<cv::Mat>(new cv::Mat());
		cap = NULL;
	}
	SensorDataImage(cv::Mat img1, TNT::Array2D<double> att1, TNT::Array2D<double> angularVel1, ImageFormat fmt){
		img1.copyTo(*img);
		att = att1.copy();
		startAngularVel = startAngularVel.copy();
		endAngularVel = endAngularVel.copy();
		imgFormat = fmt;
	}

	void copyTo(SensorDataImage &d) {
		if(&d == this)
			return;
		lock();
		d.lock();
		d.timestamp.setTime(timestamp); 
		img->copyTo(*(d.img)); 
		d.att.inject(att);
		d.startAngularVel.inject(startAngularVel);
		d.endAngularVel.inject(endAngularVel);
		d.focalLength = focalLength;
		d.unlock();
		unlock();
	}
	shared_ptr<cv::Mat> img;
	TNT::Array2D<double> att;
	TNT::Array2D<double> startAngularVel, endAngularVel;
	ImageFormat imgFormat;
	double focalLength;
	shared_ptr<cv::VideoCapture> cap;
};

class SensorDataPhoneTemp : public SensorData
{
	public:
	SensorDataPhoneTemp(){type = SENSOR_DATA_TYPE_PHONE_TEMP; battTemp = secTemp = fgTemp = tmuTemp = -1;}

	void copyTo(SensorDataPhoneTemp &d) const {
		d.timestamp.setTime(timestamp); 
		d.battTemp = battTemp;
		d.secTemp = secTemp;
		d.fgTemp = fgTemp;
		d.tmuTemp = tmuTemp;
	}
	float battTemp, secTemp, fgTemp, tmuTemp;
};
}}
#endif

#ifndef ICSL_SENSOR_DATA_ONLY
#ifndef ICSL_SENSOR_MANAGER
#define ICSL_SENSOR_MANAGER
#include <sched.h>

#include <memory>
#include <string>
#include <fstream>
#include <sstream>

#include <android/sensor.h>
#include <opencv2/core/core.hpp>
#include <toadlet/egg.h>

#include "TNT/tnt.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "ICSL/constants.h"

#include "Common.h"
#include "QuadLogger.h"
#include "Time.h"
#define ICSL_OBSERVER_ANGULAR_LISTENER_ONLY
#include "Observer_Angular.h"
#undef ICSL_OBSERVER_ANGULAR_LISTENER_ONLY

namespace ICSL{
namespace Quadrotor{
static const int ASENSOR_TYPE_PRESSURE=6; // not yet defined for NDK
using namespace std;

class SensorManagerListener
{
	public:
	virtual void onNewSensorUpdate(shared_ptr<SensorData> const &data)=0;
};

class SensorManager : public toadlet::egg::Thread, public Observer_AngularListener
{
	public:
	SensorManager();
	virtual ~SensorManager();

	void run();
	void initialize();
	void shutdown();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setStartTime(Time time){mMutex_startTime.lock(); mStartTime = time; mTimestampOffsetNS = 0; mMutex_startTime.unlock();}

	void addListener(SensorManagerListener *l){mMutex_listeners.lock(); mListeners.push_back(l); mMutex_listeners.unlock();}

	// for Observer_AngularListener
	void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel);

	protected:
	bool mRunning, mDone;
	ASensorManager* mSensorManager;
	ASensorEventQueue* mSensorEventQueue;
	const ASensor *mAccelSensor, *mGyroSensor, *mMagSensor, *mPressureSensor;

	static int getBatteryTemp();
	static int getSecTemp();
	static int getFuelgaugeTemp();
	static int getTmuTemp();
	void runTemperatureMonitor();

	QuadLogger *mQuadLogger;

	TNT::Array2D<double> mLastAccel, mLastGyro, mLastMag;
	double mLastPressure;
	toadlet::egg::Mutex mMutex_data, mMutex_listeners, mMutex_startTime, mMutex_attData;

	Time mStartTime;

	Collection<SensorManagerListener *> mListeners;

	shared_ptr<cv::VideoCapture> initCamera();
	void runImgAcq(shared_ptr<cv::VideoCapture> cap);
	TNT::Array2D<double> mCurAtt, mCurAngularVel;

	TNT::Array2D<double> mRotCamToPhone, mRotPhoneToCam;

	int64_t mTimestampOffsetNS;
	int mThreadPriority, mScheduler;
};


} // namespace Quadrotor
} // namespace ICSL
#endif
#endif
