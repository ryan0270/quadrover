#ifndef ICSL_SENSOR_MANAGER
#define ICSL_SENSOR_MANAGER

#include <string>
#include <fstream>
#include <sstream>
#include <android/sensor.h>
#include <opencv2/core/core.hpp>
#include "toadlet/egg.h"
#include "TNT/tnt.h"

#include "Common.h"
#include "QuadLogger.h"
#include "Time.h"

namespace ICSL{
namespace Quadrotor{
static const int ASENSOR_TYPE_PRESSURE=6; // not yet defined for NDK

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
	SensorDataImage(){type = SENSOR_DATA_TYPE_UNDEFINED;}
	SensorDataImage(cv::Mat img1, TNT::Array2D<double> att1, TNT::Array2D<double> angularVel1){
		img1.copyTo(img);
		att = att1.copy();
		angularVel = angularVel.copy();
	}

	void copyTo(SensorDataImage &d){d.timestamp.setTime(timestamp); img.copyTo(d.img); d.att = att.copy(); d.angularVel = angularVel.copy();}
	cv::Mat img;
	TNT::Array2D<double> att;
	TNT::Array2D<double> angularVel;
};

class SensorManagerListener
{
	public:
	virtual void onNewSensorUpdate(SensorData const &data)=0;
};

class SensorManager : public toadlet::egg::Thread
{
	public:
	SensorManager();
	virtual ~SensorManager();

	void run();
	void initialize();
	void shutdown();

	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setStartTime(Time time){mStartTime = time;}

	void addListener(SensorManagerListener *l){mMutex_listeners.lock(); mListeners.push_back(l); mMutex_listeners.unlock();}

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
	toadlet::egg::Mutex mMutex_data, mMutex_listeners;

	Time mStartTime;

	Collection<SensorManagerListener *> mListeners;
};


} // namespace Quadrotor
} // namespace ICSL
#endif
