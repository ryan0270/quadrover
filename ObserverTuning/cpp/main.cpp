#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>

#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d/features2d.hpp>

#include "toadlet/egg.h"

#include "TNT_Utils.h"

#include "constants.h"
#include "Common.h"
#include "Data.h"
#include "Observer_Angular.h"
#include "Observer_Translational.h"
#include "QuadLogger.h"
#include "CommManager.h"
#include "Time.h"
#include "TranslationController.h"
#include "AttitudeThrustController.h"
#include "VideoMaker.h"
#include "SensorManager.h"
#include "MotorInterface.h"
#include "FeatureFinder.h"
#include "TargetFinder.h"
#include "VelocityEstimator.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// yeah, I'm too lazy to avoid making this global at the moment
Collection<ICSL::Quadrotor::CommManagerListener *> commManagerListeners;
Collection<ICSL::Quadrotor::SensorManagerListener *> sensorManagerListeners;
void addSensorManagerListener(ICSL::Quadrotor::SensorManagerListener *l){sensorManagerListeners.push_back(l);}
void addCommManagerListener(ICSL::Quadrotor::CommManagerListener *l){commManagerListeners.push_back(l);}


int main(int argv, char* argc[])
{
	using namespace ICSL::Quadrotor;
	using namespace ICSL::Constants;
	using namespace TNT;
	using namespace std;
	cout << "start chadding" << endl;

	string dataDir = "../dataSets/Sep8";

	string imgDir;
	imgDir = dataDir + "/video";

	vector<pair<int, Time>> imgIdList;
	// preload all images
//	list<pair<int, cv::Mat>> imgList;
//	int imgId = 0;
//	imgId = 6895;
//	int numImages;
//	numImages = 2413;
//	for(int i=0; i<numImages; i++)
//	{
////		cout << "Loading image " << i << endl;
//		cv::Mat img;
//		while(img.data == NULL)
//		{
//			stringstream ss;
//			ss << "image_" << ++imgId << ".bmp";
//			img = cv::imread(imgDir+"/"+ss.str());
//		}
//
//		imgList.push_back(pair<int, cv::Mat>(imgId, img));
//	}
//
////cout << "final image: " << imgId << endl;

	//
	Array2D<double> mRotViconToQuad = createRotMat(0, (double)PI);
	Array2D<double> mRotQuadToPhone = matmult(createRotMat(2,-0.25*PI),
											  createRotMat(0,(double)PI));
	Array2D<double> mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
					 						 createRotMat(0,(double)PI));
	Array2D<double> mRotPhoneToCam = transpose(mRotCamToPhone);
	Array2D<double> mRotViconToPhone = matmult(mRotQuadToPhone, mRotViconToQuad);

	// make the same workers as I use in Rover
	TranslationController mTranslationController;
	AttitudeThrustController mAttitudeThrustController;
	Observer_Angular mObsvAngular;
	Observer_Translational mObsvTranslational;
	QuadLogger mQuadLogger;
	VelocityEstimator mVelocityEstimator;
	FeatureFinder mFeatureFinder;
	TargetFinder mTargetFinder;
//	SensorManager mSensorManager;
	MotorInterface mMotorInterface;
//	CommManager mCommManager;
//	VideoMaker mVideoMaker;

	Time startTime;

	// Make the appropriate connections
	mTranslationController.setRotViconToPhone(mRotViconToPhone);
	mTranslationController.setStartTime(startTime);
	mTranslationController.setQuadLogger(&mQuadLogger);
	mTranslationController.initialize();
	addCommManagerListener(&mTranslationController);

	mAttitudeThrustController.setStartTime(startTime);
	mAttitudeThrustController.setQuadLogger(&mQuadLogger);
	mAttitudeThrustController.setMotorInterface(&mMotorInterface);
	mAttitudeThrustController.initialize();
	mTranslationController.addListener(&mAttitudeThrustController);
	addCommManagerListener(&mAttitudeThrustController);

	mObsvAngular.initialize();
	mObsvAngular.setStartTime(startTime);
	mObsvAngular.setQuadLogger(&mQuadLogger);
	mObsvAngular.addListener(&mAttitudeThrustController);
	mObsvAngular.start();
	addCommManagerListener(&mObsvAngular);

	mObsvTranslational.setQuadLogger(&mQuadLogger);
	mObsvTranslational.setStartTime(startTime);
	mObsvTranslational.setRotViconToPhone(mRotViconToPhone);
	mObsvTranslational.initialize();
	mObsvTranslational.addListener(&mTranslationController);
	mObsvAngular.addListener(&mObsvTranslational);
//	mObsvTranslational.start();
	addCommManagerListener(&mObsvTranslational);

	mFeatureFinder.initialize();
	mFeatureFinder.setStartTime(startTime);
	mFeatureFinder.setQuadLogger(&mQuadLogger);
	addSensorManagerListener(&mFeatureFinder);
	addCommManagerListener(&mFeatureFinder);

	mTargetFinder.initialize();
	mTargetFinder.setStartTime(startTime);
	mTargetFinder.setQuadLogger(&mQuadLogger);
	addSensorManagerListener(&mTargetFinder);
	mTargetFinder.addListener(&mObsvTranslational);
	addCommManagerListener(&mTargetFinder);

	mVelocityEstimator.initialize();
	mVelocityEstimator.setStartTime(startTime);
	mVelocityEstimator.setQuadLogger(&mQuadLogger);
	mVelocityEstimator.setObserverTranslational(&mObsvTranslational);
	mVelocityEstimator.setRotPhoneToCam(mRotPhoneToCam);
	mVelocityEstimator.addListener(&mObsvTranslational);
	mFeatureFinder.addListener(&mVelocityEstimator);
	addCommManagerListener(&mVelocityEstimator);

	addSensorManagerListener(&mObsvAngular);
	addSensorManagerListener(&mObsvTranslational);

	uint32 logMask = 0;
	logMask = LOG_FLAG_PC_UPDATES ;
	logMask |= LOG_FLAG_STATE;
//	logMask |= LOG_FLAG_STATE_DES;
//	logMask |= LOG_FLAG_MOTORS;
	logMask |= LOG_FLAG_OBSV_UPDATE;
	logMask |= LOG_FLAG_OBSV_BIAS;
//	logMask |= LOG_FLAG_MAGNOMETER;
//	logMask |= LOG_FLAG_ACCEL;
//	logMask |= LOG_FLAG_GYRO;
//	logMask |= LOG_FLAG_PRESSURE;
	logMask |= LOG_FLAG_CAM_RESULTS;
//	logMask |= LOG_FLAG_CAM_IMAGES;
//	logMask |= LOG_FLAG_PHONE_TEMP;
	mQuadLogger.setStartTime(startTime);
	
	mMotorInterface.addListener(&mTranslationController);

	////////////////////////////////////////////////////////////////////////////////////
	// Now to set parameters like they would have been online
	double gainP = 4;
	double gainI = 0.004;
	double accelWeight = 1;
	double magWeight = 0;
	Collection<float> nomMag;
	nomMag.push_back(-16.2);
	nomMag.push_back(3.7);
	nomMag.push_back(15.9);
	mObsvAngular.onNewCommAttObserverGain(gainP, gainI, accelWeight, magWeight);
	mObsvAngular.onNewCommNominalMag(nomMag);

	Array2D<double> gyroBias(3,1);
	gyroBias[0][0] = -0.006;
	gyroBias[1][0] = -0.008;
	gyroBias[2][0] = -0.008;
	shared_ptr<IData> gyroBiasData(new DataVector<double>());
	gyroBiasData->type = DATA_TYPE_GYRO;
	static_pointer_cast<DataVector<double> >(gyroBiasData)->data = gyroBias.copy();
	static_pointer_cast<DataVector<double> >(gyroBiasData)->dataCalibrated = gyroBias.copy();
	for(int i=0; i<2000; i++)
	{
		mObsvAngular.onNewSensorUpdate(gyroBiasData);
		System::usleep(700);
	}

	Collection<float> measVar;
	measVar.push_back(0.0001);
	measVar.push_back(0.0001);
	measVar.push_back(0.0001);
	measVar.push_back(0.1);
	measVar.push_back(0.1);
	measVar.push_back(0.1);
	mObsvTranslational.onNewCommKalmanMeasVar(measVar);

	Collection<float> dynVar;
	dynVar.push_back(0.0001);
	dynVar.push_back(0.0001);
	dynVar.push_back(0.0001);
	dynVar.push_back(0.5);
	dynVar.push_back(0.5);
	dynVar.push_back(0.5);
	dynVar.push_back(0.001); // accel bias
	dynVar.push_back(0.001);
	dynVar.push_back(0.001);
	mObsvTranslational.onNewCommKalmanDynVar(dynVar);

	float xBias = -0.1;
	float yBias = -0.1;
	float zBias = -0.3;
	mObsvTranslational.onNewCommAccelBias(xBias, yBias, zBias);

	mQuadLogger.setMask(logMask);
	mQuadLogger.setDir(dataDir.c_str());
	mQuadLogger.setFilename("obsvLog.txt");

	////////////////////////////////////////////////////////////////////////////////////
	// load data and manually feed them to the appropriate listener ... hopefully
	list<shared_ptr<IData>> dataList;
	dataList.clear();

	double accelCal1X = 0.05;
	double accelCal1Y = 0.06;
	double accelCal1Z = 9.38;
	double accelCal2X = 0.06;
	double accelCal2Y = -0.16;
	double accelCal2Z = -10.10;

	double accelScaleZ = 1.009;//0.5*(accelCal1Z-accelCal2Z)/GRAVITY;
	double accelScaleX = 1.0;
	double accelScaleY = 1.0;

	double accelOffX = 0.5*(accelCal1X+accelCal2X);
	double accelOffY = 0.5*(accelCal1Y+accelCal2Y);
	double accelOffZ = 0.5*(accelCal1Z+accelCal2Z);

	int firstTime = -1;

	sched_param sp;
	sp.sched_priority = sched_get_priority_max(SCHED_NORMAL);
	sched_setscheduler(0, SCHED_NORMAL, &sp);

	Array2D<double> curViconState(12,1);;
	bool haveFirstVicon = false;

	////////////////////////////////////////////////////////////////////////////////////
	// Run settings
	int endTimeDelta = 100e3;
	float viconUpdateRate = 5; // Hz
	int viconUpdatePeriodMS = 1.0f/viconUpdateRate*1000+0.5;
	Time lastViconUpdateTime;

//	srand(1);
	default_random_engine randGenerator;
	normal_distribution<double> stdGaussDist(0,1);
	Array2D<double> noiseStd(12,1,0.0);
	noiseStd[6][0] = 0.010;
	noiseStd[7][0] = 0.010;
	noiseStd[8][0] = 0.010;

	string line;
	string dataFilename = dataDir+"/phoneLog.txt";
	ifstream file(dataFilename.c_str());
	if(file.is_open())
	{
		getline(file, line); // first line is a throw-away
		getline(file, line); // second line is also a throw-away

		toadlet::uint64 lastDispTimeMS;
		while(file.good() && startTime.getElapsedTimeMS()< firstTime+endTimeDelta)
		{
			getline(file, line);
			stringstream ss(line);
			double time;
			int type;
			ss >> time >> type;

			if(firstTime == -1)
			{
				firstTime = time;
				lastDispTimeMS = time;
				Time now;
				startTime.setTimeMS(now.getMS()-firstTime);

				mTranslationController.setStartTime(startTime);
				mAttitudeThrustController.setStartTime(startTime);
				mObsvAngular.setStartTime(startTime);
				mObsvTranslational.setStartTime(startTime);
				mFeatureFinder.setStartTime(startTime);
				mTargetFinder.setStartTime(startTime);
				mVelocityEstimator.setStartTime(startTime);
				mQuadLogger.setStartTime(startTime);

				mObsvTranslational.start();
				mQuadLogger.start();
				cout << "Time: " << time << endl;
			}
			
			while(startTime.getElapsedTimeMS() < time)
				System::usleep(10);

			if(time - lastDispTimeMS > 5e3)
			{
				cout << "Time: " << time << endl;
				lastDispTimeMS = time;
			}

			// Vicon updates
			if(lastViconUpdateTime.getElapsedTimeMS() > viconUpdatePeriodMS && haveFirstVicon)
			{
				toadlet::egg::Collection<float> state;
				for(int i=0; i<curViconState.dim1(); i++)
					state.push_back(curViconState[i][0] + noiseStd[i][0]*stdGaussDist(randGenerator) );
				for(int i=0; i<sensorManagerListeners.size(); i++)
					commManagerListeners[i]->onNewCommStateVicon(state);
				lastViconUpdateTime.setTime();
			}

			// Sensor updates
			shared_ptr<IData> data = NULL;
			switch(type)
			{
				case LOG_ID_ACCEL:
					{
						Array2D<double> accel(3,1), accelCal(3,1);
						ss >> accel[0][0] >> accel[1][0] >> accel[2][0];
						accelCal[0][0] = (accel[0][0]-accelOffX)/accelScaleX;
						accelCal[1][0] = (accel[1][0]-accelOffY)/accelScaleY;
						accelCal[2][0] = (accel[2][0]-accelOffZ)/accelScaleZ;

						data = shared_ptr<IData>(new DataVector<double>());
						data->type = DATA_TYPE_ACCEL;
						static_pointer_cast<DataVector<double> >(data)->data = accel;
						static_pointer_cast<DataVector<double> >(data)->dataCalibrated = accelCal;

						data->timestamp.setTimeMS(startTime.getMS()+time);
						for(int i=0; i<sensorManagerListeners.size(); i++)
							sensorManagerListeners[i]->onNewSensorUpdate(data);
					}
					break;
				case LOG_ID_GYRO:
					{
						Array2D<double> gyro(3,1);
						ss >> gyro[0][0] >> gyro[1][0] >> gyro[2][0];

						data = shared_ptr<IData>(new DataVector<double>());
						data->type = DATA_TYPE_GYRO;
						static_pointer_cast<DataVector<double> >(data)->data = gyro;
						static_pointer_cast<DataVector<double> >(data)->dataCalibrated = gyro;

						data->timestamp.setTimeMS(startTime.getMS()+time);
						for(int i=0; i<sensorManagerListeners.size(); i++)
							sensorManagerListeners[i]->onNewSensorUpdate(data);
					}
					break;
				case LOG_ID_MAGNOMETER:
					{
						Array2D<double> mag(3,1);
						ss >> mag[0][0] >> mag[1][0] >> mag[2][0];

						data = shared_ptr<IData>(new DataVector<double>());
						data->type = DATA_TYPE_MAG;
						static_pointer_cast<DataVector<double> >(data)->data = mag;
						static_pointer_cast<DataVector<double> >(data)->dataCalibrated = mag;

						data->timestamp.setTimeMS(startTime.getMS()+time);
						for(int i=0; i<sensorManagerListeners.size(); i++)
							sensorManagerListeners[i]->onNewSensorUpdate(data);
					}
					break;
				case LOG_ID_PRESSURE:
					break;
				case LOG_ID_IMAGE:
					break;
				case LOG_ID_MOTOR_CMDS:
					break;
				case LOG_ID_PHONE_TEMP:
					break;
				case LOG_ID_RECEIVE_VICON:
					// Ideally I'd pull this directly from the Leash log file, but that's work :)
					{
						for(int i=0; i<12; i++)
							ss >> curViconState[i][0];
						haveFirstVicon = true;
					}
					break;
			}
		}

		file.close();
	}

	mQuadLogger.shutdown();
	mTranslationController.shutdown();
	mAttitudeThrustController.shutdown();
//	mCommManager.shutdown();
	mVelocityEstimator.shutdown();
	mFeatureFinder.shutdown();
	mTargetFinder.shutdown();
//	mVideoMaker.shutdown();
	mObsvAngular.shutdown(); 
	mObsvTranslational.shutdown(); 
//	mSensorManager.shutdown();
	mMotorInterface.shutdown();

    return 0;
}
