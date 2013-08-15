#include "SensorManager.h"

using namespace std;
using namespace toadlet::egg;
using namespace TNT;
using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{
	SensorManager::SensorManager() :
		mLastAccel(3,1,0.0),
		mLastGyro(3,1,0.0),
		mLastMag(3,1,0.0),
		mLastPressure(0),
		mCurAtt(3,1,0.0),
		mCurAngularVel(3,1,0.0),
		mAttAccum(3,1,0.0),
		mAngularVelAccum(3,1,0.0)
	{
		mRunning = false;
		mDone = true;

		mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
								 createRotMat(0,(double)PI));
		mRotPhoneToCam = transpose(mRotCamToPhone);

		mTimestampOffsetNS = 0;

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mImageDT = 0;
		mImageCnt = 0;
		mLastImageTime.setTimeMS(0);

		mAttAccumCnt = 0;
		mAngularVelAccum = 0;
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

		// Now try load camera calibration params
		cv::FileStorage fs;
//		String filename = "sdcard/RoverService/s4Calib_640x480.yml";
		String filename = "sdcard/RoverService/s3Calib_640x480.yml";
		fs.open(filename.c_str(), cv::FileStorage::READ);
		if( fs.isOpened() )
		{
			String str = "Camera calib loaded from " + filename;
			fs["camera_matrix"] >> mCameraMatrix_640x480;
			str = str+"\n\t"+"Focal length: " + mCameraMatrix_640x480.at<double>(0,0);
			str = str+"\n\t"+"centerX: " + mCameraMatrix_640x480.at<double>(0,2);
			str = str+"\n\t"+"centerY: " + mCameraMatrix_640x480.at<double>(1,2);
			Log::alert(str);
		}
		else
		{
			Log::alert("Failed to open " + filename);
		}
		fs.release();
	}

	void SensorManager::shutdown()
	{
		Log::alert("////////////////////// SensorManager shutdown started //////////////////");
		mRunning = false;
		while(!mDone) 
			System::msleep(10);

		if(mSensorEventQueue != NULL && mMagSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mMagSensor);
		if(mSensorEventQueue != NULL && mAccelSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mAccelSensor);
		if(mSensorEventQueue != NULL && mGyroSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mGyroSensor);
		if(mSensorEventQueue != NULL && mPressureSensor != NULL)
			ASensorEventQueue_disableSensor(mSensorEventQueue, mPressureSensor);

		if(mSensorManager != NULL && mSensorEventQueue != NULL)
			ASensorManager_destroyEventQueue(mSensorManager, mSensorEventQueue);
		Log::alert("////////////////////// SensorManager shutdown done //////////////////");
	}

	void SensorManager::run()
	{
		mRunning = true;
		thread tempMonitorTh(&SensorManager::runTemperatureMonitor, this);

		double accelCal1X = -0.0002;
		double accelCal1Y = 0.1116;
		double accelCal1Z = 9.4939;
		double accelCal2X = -0.0568;
		double accelCal2Y = -0.0564;
		double accelCal2Z = -10.0235;

		double accelScaleZ = 0.5*(accelCal1Z-accelCal2Z)/GRAVITY;
		// double have information on these so assume they are the same
		double accelScaleX = accelScaleZ;
		double accelScaleY = accelScaleZ;

		double accelOffX = 0.5*(accelCal1X+accelCal2X);
		double accelOffY = 0.5*(accelCal1Y+accelCal2Y);
		double accelOffZ = 0.5*(accelCal1Z+accelCal2Z);

		Array2D<double> accelCalibrated(3,1), gyroCalibrated(3,1), magCalibrated(3,1);

		mDone = false;
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			ASensorEvent event;
			while(ASensorEventQueue_getEvents(mSensorEventQueue, &event, 1) > 0)
			{
				if(mTimestampOffsetNS == 0)
					mTimestampOffsetNS = mStartTime.getNS()-event.timestamp;
// TODO: Set the event time to match the timestamp in the ASensor struct
				LogFlags logFlag = LOG_FLAG_OTHER;
				int logID = -1;
				shared_ptr<IData> data = NULL;
				switch(event.type)
				{
					case ASENSOR_TYPE_PRESSURE:
						{
							logFlag= LOG_FLAG_PRESSURE;
							logID = LOG_ID_PRESSURE;
//							data = shared_ptr<Data<double> >(new Data<double>(event.pressure, DATA_TYPE_PRESSURE));
							data = shared_ptr<IData>(new Data<double>());
							data->type = DATA_TYPE_PRESSURE;
							static_pointer_cast<Data<double> >(data)->data = event.pressure;
							static_pointer_cast<Data<double> >(data)->dataCalibrated = event.pressure;
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
							accelCalibrated[0][0] = (event.data[0]-accelOffX)/accelScaleX;
							accelCalibrated[1][0] = (event.data[1]-accelOffY)/accelScaleY;
							accelCalibrated[2][0] = (event.data[2]-accelOffZ)/accelScaleZ;
							data = shared_ptr<IData>(new DataVector<double>());
							data->type = DATA_TYPE_ACCEL;
							static_pointer_cast<DataVector<double> >(data)->data = mLastAccel.copy();
							static_pointer_cast<DataVector<double> >(data)->dataCalibrated = accelCalibrated.copy();
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
							data = shared_ptr<IData>(new DataVector<double>());
							data->type = DATA_TYPE_GYRO;
							static_pointer_cast<DataVector<double> >(data)->data = mLastGyro.copy();
// TODO: Apply gyro bias estimate for the calibrated data
							static_pointer_cast<DataVector<double> >(data)->dataCalibrated = mLastGyro.copy();
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
							data = shared_ptr<IData>(new DataVector<double>());
							data->type = DATA_TYPE_MAG;
							static_pointer_cast<DataVector<double> >(data)->data = mLastMag.copy();
							static_pointer_cast<DataVector<double> >(data)->dataCalibrated = mLastMag.copy();
							mMutex_data.unlock();
						}
						break;
					default:
						Log::alert(String()+"Unknown sensor event: "+event.type);
				}
				data->timestamp.setTimeNS(event.timestamp+mTimestampOffsetNS);

				if(mQuadLogger != NULL && logFlag != -1 && data != NULL)
				{
					String s=String()+(data->timestamp.getMS()-mStartTime.getMS())+"\t"+logID+"\t";
					s = s+event.data[0]+"\t"+event.data[1]+"\t"+event.data[2]+"\t"+event.data[3];
					mMutex_logger.lock();
					mQuadLogger->addLine(s,logFlag);
					mMutex_logger.unlock();
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


			System::usleep(100);
		}

		tempMonitorTh.join();

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

			int width = 640; int height = 480;
//			int width = 960; int height = 720;
//			int width = 1280; int height = 720;
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

//	void SensorManager::runImageAcq(shared_ptr<cv::VideoCapture> cap)
//	{
//		if(cap == NULL)
//			return;
//
//		cv::Mat image;
//		sched_param sp;
//		sp.sched_priority = mThreadPriority-1;
//		sched_setscheduler(0, mScheduler, &sp);
//		while(mRunning)
//		{
//			shared_ptr<Data> data = shared_ptr<Data>(new DataImage());
//			data->type = DATA_TYPE_IMAGE;
//
//			mMutex_attData.lock();
//			static_pointer_cast<DataImage>(data)->startAngularVel = matmult(mRotPhoneToCam,mCurAngularVel.copy());
//			static_pointer_cast<DataImage>(data)->att.inject(matmult(mRotPhoneToCam,mCurAtt));
//			mMutex_attData.unlock();
//			cap->grab();
//			data->timestamp.setTime();
//			mMutex_attData.lock();
//			static_pointer_cast<DataImage>(data)->endAngularVel = matmult(mRotPhoneToCam,mCurAngularVel.copy());
//			mMutex_attData.unlock();
//
//			shared_ptr<cv::Mat> image(new cv::Mat);
//			cap->retrieve(*image);
//			static_pointer_cast<DataImage>(data)->image = image;
////			image->copyTo(*static_pointer_cast<DataImage>(data)->image); // for some strange reason, when I just directly assign the ptr (the above line) it cause problems for imwrite when QuadLogger goes to save the images
//			static_pointer_cast<DataImage>(data)->cap = cap;
//			static_pointer_cast<DataImage>(data)->focalLength = 3.7*image->cols/5.76; // (focal length mm)*(image width px)/(ccd width mm)
//
//			static_pointer_cast<DataImage>(data)->imageFormat = IMG_FORMAT_BGR;
//
//			mMutex_listeners.lock();
//			for(int i=0; i<mListeners.size(); i++)
//				mListeners[i]->onNewSensorUpdate(data);
//			mMutex_listeners.unlock();
//
//			data = NULL;
//		}
//
//		if(cap->isOpened())
//		{
//			cap->set(CV_CAP_PROP_ANDROID_FLASH_MODE,CV_CAP_ANDROID_FLASH_MODE_OFF);
//			cap->release();
//		}
//		cap == NULL;
//
////		delete cap;
//	}

	void SensorManager::runTemperatureMonitor()
	{
		sched_param sp;
		sp.sched_priority = mThreadPriority-1;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			float battTemp = getBatteryTemp()/10.0;
			float secTemp = getSecTemp()/10.0;
			float fgTemp= getFuelgaugeTemp()/10.0;
//			float tmuTemp = getTmuTemp()/10.0;

			shared_ptr<DataPhoneTemp<double> > data = shared_ptr<DataPhoneTemp<double> >(new DataPhoneTemp<double>());
			data->battTemp = battTemp;
			data->secTemp = secTemp;
			data->fgTemp = fgTemp;
//			data->tmuTemp = tmuTemp;
			mMutex_listeners.lock();
			for(int i=0; i<mListeners.size(); i++)
			{
				mListeners[i]->onNewSensorUpdate(data);
			}
			mMutex_listeners.unlock();

			if(mQuadLogger != NULL)
			{
				String s = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_PHONE_TEMP + "\t";
				s = s+battTemp+"\t";
				s = s+secTemp+"\t";
				s = s+fgTemp+"\t";
//				s = s+tmuTemp+"\t";
				mMutex_logger.lock();
				mQuadLogger->addLine(s,LOG_FLAG_PHONE_TEMP);
				mMutex_logger.unlock();
			}

			System::msleep(500);
		}
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

//	int SensorManager::getTmuTemp()
//	{
//		int temp = 0;
//		// this path is for the SIII
//		string filename = "/sys/devices/platform/s5p-tmu/curr_temp";
//		ifstream file(filename.c_str());
//		if(file.is_open())
//		{
//			string line;
//			getline(file,line);
//			file.close();
//
//			stringstream ss(line);
//			ss >> temp;
//		}
//		else
//			Log::alert("Failed to open "+String(filename.c_str()));
//
//		return temp;
//	}

	void SensorManager::onObserver_AngularUpdated(shared_ptr<DataVector<double> > attData, shared_ptr<DataVector<double> > angularVelData)
	{
		mMutex_attData.lock();
		mCurAtt.inject(attData->data);
		mCurAngularVel.inject(angularVelData->data);
		mMutex_attData.unlock();

		mMutex_accum.lock();
		mAttAccum += attData->data;
		mAttAccumCnt++;
		mAngularVelAccum += angularVelData->data;
		mAngularVelAccumCnt++;
		mMutex_accum.unlock();
	}

	void SensorManager::passNewImage(cv::Mat const *imageYUV, int64 const &timestampNS)
	{
//		if(mLastImageTime.getMS() != 0)
//		{
//			double dt = mLastImageTime.getElapsedTimeNS()/1.0e9;
//			mImageDT = (mImageDT*mImageCnt + dt)/(mImageCnt+1);
//			mImageCnt++;
//		}
//		mLastImageTime.setTime();
//		Log::alert(String()+"dt: "+mImageDT);

		shared_ptr<cv::Mat> imageBGR(new cv::Mat);
		cv::cvtColor(*imageYUV, *imageBGR, CV_YUV420sp2BGR);

		shared_ptr<DataImage> data(new DataImage());
		data->type = DATA_TYPE_IMAGE;
		data->timestamp.setTimeNS(timestampNS);

		Array2D<double> att, angVel;
		mMutex_accum.lock();
		if(mAttAccumCnt == 0)
			att = Array2D<double>(3,1,0.0);
		else
			att = 1.0/mAttAccumCnt*mAttAccum;
		if(mAngularVelAccumCnt == 0)
			angVel = Array2D<double>(3,1,0.0);
		else
			angVel = 1.0/mAngularVelAccumCnt*mAngularVelAccum;

		for(int i=0; i<3; i++)
		{
			mAttAccum[i][0] = 0;
			mAngularVelAccum[i][0] = 0;
		}
		mAttAccumCnt = 0;
		mAngularVelAccumCnt = 0;
		mMutex_accum.unlock();

		Array2D<double> attCam(3,1);
		attCam[0][0] = matmultS(submat(mRotPhoneToCam,0,0,0,2), att);
		attCam[1][0] = matmultS(submat(mRotPhoneToCam,1,1,0,2), att);
		attCam[2][0] = matmultS(submat(mRotPhoneToCam,2,2,0,2), att);
		data->att.inject(attCam);
		data->angularVel.inject(matmult(mRotPhoneToCam, angVel));

		data->image = imageBGR;
		data->imageFormat = IMG_FORMAT_BGR;
		data->cap = NULL;
		if(mCameraMatrix_640x480.rows > 0)
		{
			data->focalLength_640x480 = mCameraMatrix_640x480.at<double>(0,0);
			data->centerX_640x480 = mCameraMatrix_640x480.at<double>(0,2);
			data->centerY_640x480 = mCameraMatrix_640x480.at<double>(1,2);
		}
		else
		{
			data->focalLength_640x480 = 580;
			data->centerX_640x480 = 320;
			data->centerY_640x480 = 240;
		}

		mMutex_listeners.lock();
		for(int i=0; i<mListeners.size(); i++)
			mListeners[i]->onNewSensorUpdate(static_pointer_cast<IData>(data));
		mMutex_listeners.unlock();

		if(mQuadLogger != NULL)
		{
			String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMAGE + "\t" + data->id;
			mMutex_logger.lock();
			mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
			mMutex_logger.unlock();
		}
	}

} // namespace Quadrotor
} // namespace ICSL
