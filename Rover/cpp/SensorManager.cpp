#include "SensorManager.h"

using namespace std;
using namespace toadlet::egg;
using namespace TNT;

namespace ICSL{
namespace Quadrotor{
	SensorManager::SensorManager() :
		mLastAccel(3,1,0.0),
		mLastGyro(3,1,0.0),
		mLastMag(3,1,0.0),
		mLastPressure(0),
		mCurAtt(3,1,0.0),
		mCurAngularVel(3,1,0.0)
	{
		mRunning = false;
		mDone = true;
	}

	SensorManager::~SensorManager()
	{
	}

	void SensorManager::initialize()
	{
		mSensorManager = ASensorManager_getInstance();
		ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
		mSensorEventQueue = ASensorManager_createEventQueue(mSensorManager, looper, 0, NULL, NULL);

		mAccelSensor = ASensorManager_getDefaultSensor(mSensorManager, ASENSOR_TYPE_ACCELEROMETER);
		if(mAccelSensor != NULL)
		{
			const char* name = ASensor_getName(mAccelSensor);
			const char* vendor = ASensor_getVendor(mAccelSensor);
			float res = ASensor_getResolution(mAccelSensor);
			Log::alert(String()+"Accel sensor:\n\t"+name+"\n\t"+vendor+"\n\tresolution:"+res);
			ASensorEventQueue_enableSensor(mSensorEventQueue, mAccelSensor);
			ASensorEventQueue_setEventRate(mSensorEventQueue, mAccelSensor, 10*1000); // this is the best it actually achieves
		}

		mGyroSensor = ASensorManager_getDefaultSensor(mSensorManager, ASENSOR_TYPE_GYROSCOPE);
		if(mGyroSensor != NULL)
		{
			const char* name = ASensor_getName(mGyroSensor);
			const char* vendor = ASensor_getVendor(mGyroSensor);
			float res = ASensor_getResolution(mGyroSensor);
			Log::alert(String()+"Gyro sensor:\n\t"+name+"\n\t"+vendor+"\n\tresolution: "+res);
			ASensorEventQueue_enableSensor(mSensorEventQueue, mGyroSensor);
			ASensorEventQueue_setEventRate(mSensorEventQueue, mGyroSensor, 5*1000);
		}

		mMagSensor = ASensorManager_getDefaultSensor(mSensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);
		if(mMagSensor != NULL)
		{
			const char* name = ASensor_getName(mMagSensor);
			const char* vendor = ASensor_getVendor(mMagSensor);
			float res = ASensor_getResolution(mMagSensor);
			Log::alert(String()+"Mag sensor:\n\t"+name+"\n\t"+vendor+"\n\tresolution: "+res);
			ASensorEventQueue_enableSensor(mSensorEventQueue, mMagSensor);
			ASensorEventQueue_setEventRate(mSensorEventQueue, mMagSensor, 10*1000); // this is the best it actually achieves
		}

		mPressureSensor = ASensorManager_getDefaultSensor(mSensorManager, ASENSOR_TYPE_PRESSURE);
		if(mPressureSensor != NULL)
		{
			const char* name = ASensor_getName(mPressureSensor);
			const char* vendor = ASensor_getVendor(mPressureSensor);
			float res = ASensor_getResolution(mPressureSensor);
			Log::alert(String()+"Pressure sensor:\n\t"+name+"\n\t"+vendor+"\n\tresolution: "+res);
			ASensorEventQueue_enableSensor(mSensorEventQueue, mPressureSensor);
			ASensorEventQueue_setEventRate(mSensorEventQueue, mPressureSensor, 10*1000); // this is the best it actually achieves
		}

		// list out all available sensors
//		const ASensor* const* sensorList;
//		int numSensors = ASensorManager_getSensorList(mSensorManager, &sensorList);
//		for(int i=1; i<numSensors; i++)
//		{
//			const char* name = ASensor_getName(sensorList[i]);
//			const char* vendor=  ASensor_getVendor(sensorList[i]);
//			float res = ASensor_getResolution(sensorList[i]);
//			int type = ASensor_getType(sensorList[i]);
//			String str = "Sensor \n";
//			str = str+"\t"+name+"\n";
//			str = str+"\t"+vendor+"\n";
//			str = str+"\t"+type+"\n";
//			str = str+"\tresolution: "+res;
//			Log::alert(str);
//		}
	}

	void SensorManager::shutdown()
	{
		Log::alert("////////////////////// SensorManager shutdown started //////////////////");
		mRunning = false;
		toadlet::egg::System sys;
		while(!mDone) 
		{
			Log::alert("SensorManager waiting");
			sys.msleep(10);
		}

		if(mMagSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mMagSensor);
		if(mAccelSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mAccelSensor);
		if(mGyroSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mGyroSensor);
		if(mPressureSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mPressureSensor);

		if(mSensorManager != NULL && mSensorEventQueue != NULL)
			ASensorManager_destroyEventQueue(mSensorManager, mSensorEventQueue);
		Log::alert("////////////////////// SensorManager shutdown done //////////////////");
	}

	void SensorManager::run()
	{
		mRunning = true;
		shared_ptr<cv::VideoCapture> cap = initCamera();
		if(cap == NULL)
			mRunning = false;

		class : public Thread{
			public:
			void run(){parent->runImgAcq(cap);}
			shared_ptr<cv::VideoCapture> cap;
			SensorManager *parent;
		} imgAcqThread;
		imgAcqThread.parent = this;
		imgAcqThread.cap = cap;
		imgAcqThread.start();

		System sys;
		mDone = false;
		Time lastTempMeasureTime;
		while(mRunning)
		{
			ASensorEvent event;
			while(ASensorEventQueue_getEvents(mSensorEventQueue, &event, 1) > 0)
			{
				int logFlag = -1;
				int logID = -1;
				uint64 curTimeMS;
				shared_ptr<SensorData> data = NULL;
				switch(event.type)
				{
					case ASENSOR_TYPE_PRESSURE:
						{
							logFlag= LOG_FLAG_PRESSURE;
							logID = LOG_ID_PRESSURE;
							data = shared_ptr<SensorData>(new SensorData(event.pressure, SENSOR_DATA_TYPE_PRESSURE));
							mLastPressure = event.pressure;
						}
						break;
					case ASENSOR_TYPE_ACCELEROMETER:
						{
							logFlag= LOG_FLAG_ACCEL;
							logID = LOG_ID_ACCEL;

							mMutex_data.lock();
							mLastAccel[0][0] = event.data[0];
							mLastAccel[1][0] = event.data[1];
							mLastAccel[2][0] = event.data[2];
							data = shared_ptr<SensorData>(new SensorDataVector(mLastAccel, SENSOR_DATA_TYPE_ACCEL));
							mMutex_data.unlock();
						}
						break;
					case ASENSOR_TYPE_GYROSCOPE:
						{
							logFlag= LOG_FLAG_GYRO;
							logID = LOG_ID_GYRO;

							mMutex_data.lock();
							mLastGyro[0][0] = event.data[0];
							mLastGyro[1][0] = event.data[1];
							mLastGyro[2][0] = event.data[2];
							data = shared_ptr<SensorData>(new SensorDataVector(mLastGyro, SENSOR_DATA_TYPE_GYRO));
							mMutex_data.unlock();
						}
						break;
					case ASENSOR_TYPE_MAGNETIC_FIELD:
						{
							logFlag= LOG_FLAG_MAGNOMETER;
							logID = LOG_ID_MAGNOMETER;
							mMutex_data.lock();
							mLastMag[0][0] = event.data[0];
							mLastMag[1][0] = event.data[1];
							mLastMag[2][0] = event.data[2];
							data = shared_ptr<SensorData>(new SensorDataVector(mLastMag, SENSOR_DATA_TYPE_MAG));
							mMutex_data.unlock();
						}
						break;
					default:
						Log::alert(String()+"Unknown sensor event: "+event.type);
				}

				if(mQuadLogger != NULL && logFlag != -1)
				{
					String s=String()+ mStartTime.getElapsedTimeMS() + "\t" + logID +"\t"+event.data[0]+"\t"+event.data[1]+"\t"+event.data[2]+"\t"+event.data[3];
					mQuadLogger->addLine(s,logFlag);
				}

				if(data != NULL)
				{
					mMutex_listeners.lock();
					for(int i=0; i<mListeners.size(); i++)
						mListeners[i]->onNewSensorUpdate(data);
					mMutex_listeners.unlock();
//					delete data;
				}
			}

			if(lastTempMeasureTime.getElapsedTimeMS() > 1e3)
			{
				lastTempMeasureTime.setTime();
				float battTemp = getBatteryTemp()/10.0;
				float secTemp = getSecTemp()/10.0;
				float fgTemp= getFuelgaugeTemp()/10.0;
				float tmuTemp = getTmuTemp()/10.0;

				shared_ptr<SensorData> data = shared_ptr<SensorData>(new SensorDataPhoneTemp());
				static_pointer_cast<SensorDataPhoneTemp>(data)->battTemp = battTemp;
				static_pointer_cast<SensorDataPhoneTemp>(data)->secTemp = secTemp;
				static_pointer_cast<SensorDataPhoneTemp>(data)->fgTemp = fgTemp;
				static_pointer_cast<SensorDataPhoneTemp>(data)->tmuTemp = tmuTemp;
				mMutex_listeners.lock();
				for(int i=0; i<mListeners.size(); i++)
					mListeners[i]->onNewSensorUpdate(data);
				mMutex_listeners.unlock();
//				delete data;

				String s = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_PHONE_TEMP + "\t";
				s = s+battTemp+"\t";
				s = s+secTemp+"\t";
				s = s+fgTemp+"\t";
				s = s+tmuTemp+"\t";
				mQuadLogger->addLine(s,LOG_FLAG_PHONE_TEMP);
			}

			sys.msleep(1);
		}

		imgAcqThread.join();

		mDone = true;
	}

	shared_ptr<cv::VideoCapture> SensorManager::initCamera()
	{
		shared_ptr<cv::VideoCapture> cap = shared_ptr<cv::VideoCapture>(new cv::VideoCapture());

		cap->open(CV_CAP_ANDROID+0); // back camera
		if(cap->isOpened())
		{
			Log::alert("Successfully opened the camera");
			mRunning = true;

			int width = 640;
			int height = 480;
			cap->set(CV_CAP_PROP_FRAME_WIDTH, width);
			cap->set(CV_CAP_PROP_FRAME_HEIGHT, height);
			//		cap.set(CV_CAP_PROP_ANDROID_FLASH_MODE,CV_CAP_ANDROID_FLASH_MODE_TORCH); // for now just leave this on the whole time
			cap->set(CV_CAP_PROP_ANDROID_FOCUS_MODE,CV_CAP_ANDROID_FOCUS_MODE_CONTINUOUS_VIDEO);
			//		cap.set(CV_CAP_PROP_ANDROID_FOCUS_MODE,CV_CAP_ANDROID_FOCUS_MODE_INFINITY);
			cap->set(CV_CAP_PROP_EXPOSURE, -4);
			//		cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 5);
			cap->set(CV_CAP_PROP_ANDROID_ANTIBANDING, CV_CAP_ANDROID_ANTIBANDING_OFF);
			cap->set(CV_CAP_PROP_AUTOGRAB, 1); // any nonzero is "on"
		}
		else
		{
			Log::alert("Failed to open the camera");
			cap = NULL;
		}

		return cap;
	}

	void SensorManager::runImgAcq(shared_ptr<cv::VideoCapture> cap)
	{
		if(cap == NULL)
			return;

		cv::Mat img;
		while(mRunning)
		{
			shared_ptr<SensorData> data = shared_ptr<SensorData>(new SensorDataImage());
			data->type = SENSOR_DATA_TYPE_IMAGE;
			data->timestamp.setTime();
			cap->grab();
			shared_ptr<cv::Mat> img(new cv::Mat);
			cap->retrieve(*img);
//			static_pointer_cast<SensorDataImage>(data)->img = img;
			img->copyTo(*static_pointer_cast<SensorDataImage>(data)->img); // for some strange reason, when I just directly assign the ptr (the above line) it cause problems for imwrite when QuadLogger goes to save the images
			static_pointer_cast<SensorDataImage>(data)->cap = cap;
			static_pointer_cast<SensorDataImage>(data)->focalLength = 3.7*img->cols/5.76; // (focal length mm)*(img width px)/(ccd width mm)

			mMutex_attData.lock();
			static_pointer_cast<SensorDataImage>(data)->att.inject(mCurAtt);
			static_pointer_cast<SensorDataImage>(data)->angularVel.inject(mCurAngularVel);
			mMutex_attData.unlock();
			static_pointer_cast<SensorDataImage>(data)->imgFormat = IMG_FORMAT_BGR;

			mMutex_listeners.lock();
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onNewSensorUpdate(data);
			mMutex_listeners.unlock();
		}
//		delete data;

		if(cap->isOpened())
		{
			cap->set(CV_CAP_PROP_ANDROID_FLASH_MODE,CV_CAP_ANDROID_FLASH_MODE_OFF);
			cap->release();
		}

//		delete cap;
	}

	int SensorManager::getBatteryTemp()
	{
		int temp = 0;
		string filename = "/sys/class/power_supply/battery/temp";
		ifstream file(filename.c_str());
		if(file.is_open())
		{
			string line;
			getline(file,line);
			file.close();

			stringstream ss(line);
			ss >> temp;
		}
		else
			Log::alert("Failed to open "+String(filename.c_str()));

		return temp;
	}

	int SensorManager::getSecTemp()
	{
		int temp = 0;
		// this path is for the SIII
		string filename = "/sys/devices/platform/sec-thermistor/temperature";
		ifstream file(filename.c_str());
		if(file.is_open())
		{
			string line;
			getline(file,line);
			file.close();

			stringstream ss(line);
			ss >> temp;
		}
		else
			Log::alert("Failed to open "+String(filename.c_str()));

		return temp;
	}

	int SensorManager::getFuelgaugeTemp()
	{
		int temp = 0;
		string filename = "/sys/class/power_supply/max17047-fuelgauge/temp";
		ifstream file(filename.c_str());
		if(file.is_open())
		{
			string line;
			getline(file,line);
			file.close();

			stringstream ss(line);
			ss >> temp;
		}
		else
			(("Failed to open "+filename).c_str());

		return temp;
	}

	int SensorManager::getTmuTemp()
	{
		int temp = 0;
		// this path is for the SIII
		string filename = "/sys/devices/platform/s5p-tmu/curr_temp";
		ifstream file(filename.c_str());
		if(file.is_open())
		{
			string line;
			getline(file,line);
			file.close();

			stringstream ss(line);
			ss >> temp;
		}
		else
			Log::alert("Failed to open "+String(filename.c_str()));

		return temp;
	}

	void SensorManager::onObserver_AngularUpdated(Array2D<double> const &att, Array2D<double> const &angularVel)
	{
		mMutex_attData.lock();
		mCurAtt.inject(att);
		mCurAngularVel.inject(angularVel);
		mMutex_attData.unlock();
	}

} // namespace Quadrotor
} // namespace ICSL
