#ifndef ICSL_SENSOR_DATA
#define ICSL_SENSOR_DATA
#include <opencv2/core/core.hpp>
#include "TNT/tnt.h"
#include "Common.h"
#include "Time.h"
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

	virtual void copyTo(SensorData &d){d.timestamp.setTime(timestamp); d.data = data; d.type = type;}
	double data;
	Time timestamp;
	SensorDataType type;
};

class SensorDataVector : public SensorData
{
	public:
	SensorDataVector(){type = SENSOR_DATA_TYPE_UNDEFINED;};
	SensorDataVector(TNT::Array2D<double> const &d, SensorDataType t){data = d.copy(); type = t;}

	void copyTo(SensorDataVector &d) const {d.timestamp.setTime(timestamp); d.data = data.copy(); d.type = type;}
	TNT::Array2D<double> data;
};

class SensorDataImage : public SensorData
{
	public:
	SensorDataImage() : att(3,1,0.0), angularVel(3,1,0.0) {type = SENSOR_DATA_TYPE_IMAGE; imgFormat = IMG_FORMAT_BGR;}
	SensorDataImage(cv::Mat img1, TNT::Array2D<double> att1, TNT::Array2D<double> angularVel1, ImageFormat fmt){
		img1.copyTo(img);
		att = att1.copy();
		angularVel = angularVel.copy();
		imgFormat = fmt;
	}

	void copyTo(SensorDataImage &d){d.timestamp.setTime(timestamp); img.copyTo(d.img); d.att = att.copy(); d.angularVel = angularVel.copy();}
	cv::Mat img;
	TNT::Array2D<double> att;
	TNT::Array2D<double> angularVel;
	ImageFormat imgFormat;
};

class SensorDataPhoneTemp : public SensorData
{
	public:
	SensorDataPhoneTemp(){type = SENSOR_DATA_TYPE_PHONE_TEMP; battTemp = secTemp = fgTemp = tmuTemp = -1;}

	void copyTo(SensorDataPhoneTemp &d){
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

#include <string>
#include <fstream>
#include <sstream>
#include <android/sensor.h>
#include <opencv2/core/core.hpp>
#include "toadlet/egg.h"
#include "TNT/tnt.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>

#include "Common.h"
#include "QuadLogger.h"
#include "Time.h"
#define ICSL_OBSERVER_ANGULAR_LISTENER_ONLY
#include "Observer_Angular.h"
#undef ICSL_OBSERVER_ANGULAR_LISTENER_ONLY

namespace ICSL{
namespace Quadrotor{
static const int ASENSOR_TYPE_PRESSURE=6; // not yet defined for NDK

class SensorManagerListener
{
	public:
	virtual void onNewSensorUpdate(SensorData const *data)=0;
};

class SensorManager : public toadlet::egg::Thread, public Observer_AngularListener
{
	public:
	SensorManager();
	virtual ~SensorManager();

	void run();
	void initialize();
	void shutdown();

	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setStartTime(Time time){mMutex_startTime.lock(); mStartTime = time; mMutex_startTime.unlock();}

	void addListener(SensorManagerListener *l){mMutex_listeners.lock(); mListeners.push_back(l); mMutex_listeners.unlock();}

	// for Observer_AngularListener
	void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel);

	protected:
	bool mRunning, mDone;
	ASensorManager* mSensorManager;
	ASensorEventQueue* mSensorEventQueue;
	const ASensor *mAccelSensor, *mGyroSensor, *mMagSensor, *mPressureSensor;

	int getBatteryTemp();
	int getSecTemp();
	int getFuelgaugeTemp();
	int getTmuTemp();

	QuadLogger *mQuadLogger;

	TNT::Array2D<double> mLastAccel, mLastGyro, mLastMag;
	double mLastPressure;
	toadlet::egg::Mutex mMutex_data, mMutex_listeners, mMutex_startTime, mMutex_attData;

	Time mStartTime;

	Collection<SensorManagerListener *> mListeners;

	cv::VideoCapture* initCamera();
	void runImgAcq(cv::VideoCapture *cap);
	TNT::Array2D<double> mCurAtt, mCurAngularVel;
};


} // namespace Quadrotor
} // namespace ICSL
#endif
#endif
