#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>

//#include <opencv2/core/core.hpp>
////#include <opencv2/highgui/highgui.hpp>
////#include <opencv2/imgproc/imgproc.hpp>
////#include <opencv2/features2d/features2d.hpp>
//
//#include "toadlet/egg.h"
//
//#include "TNT_Utils.h"
//
//#include "constants.h"
//#include "Common.h"
//#include "Data.h"
//#include "Observer_Angular.h"
//#include "Observer_Translational.h"
//#include "QuadLogger.h"
//#include "CommManager.h"
//#include "Time.h"
//#include "TranslationController.h"
//#include "AttitudeThrustController.h"
//#include "VideoMaker.h"
//#include "SensorManager.h"
//#include "MotorInterface.h"
//#include "FeatureFinder.h"
//#include "TargetFinder.h"
//#include "VelocityEstimator.h"
//
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/features2d/features2d.hpp>
//
//// yeah, I'm too lazy to avoid making this global at the moment
//Collection<ICSL::Quadrotor::SensorManagerListener *> mListeners;
//void addListener(ICSL::Quadrotor::SensorManagerListener *l){mListeners.push_back(l);}

int main(int argv, char* argc[])
{
//	using namespace ICSL::Quadrotor;
//	using namespace ICSL::Constants;
//	using namespace TNT;
//	using namespace std;
//	cout << "start chadding" << endl;
//
//	string dataDir = "../dataSets/Sep8";
//
//	string imgDir;
//	imgDir = dataDir + "/video";
//
//	vector<pair<int, Time>> imgIdList;
//	// preload all images
////	list<pair<int, cv::Mat>> imgList;
////	int imgId = 0;
////	imgId = 6895;
////	int numImages;
////	numImages = 2413;
////	for(int i=0; i<numImages; i++)
////	{
//////		cout << "Loading image " << i << endl;
////		cv::Mat img;
////		while(img.data == NULL)
////		{
////			stringstream ss;
////			ss << "image_" << ++imgId << ".bmp";
////			img = cv::imread(imgDir+"/"+ss.str());
////		}
////
////		imgList.push_back(pair<int, cv::Mat>(imgId, img));
////	}
////
//////cout << "final image: " << imgId << endl;
//
//	//
//	Array2D<double> mRotViconToQuad = createRotMat(0, (double)PI);
//	Array2D<double> mRotQuadToPhone = matmult(createRotMat(2,-0.25*PI),
//											  createRotMat(0,(double)PI));
//	Array2D<double> mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
//					 						 createRotMat(0,(double)PI));
//	Array2D<double> mRotPhoneToCam = transpose(mRotCamToPhone);
//	Array2D<double> mRotViconToPhone = matmult(mRotQuadToPhone, mRotViconToQuad);
//
//	// make the same workers as I use in Rover
//	TranslationController mTranslationController;
//	AttitudeThrustController mAttitudeThrustController;
//	Observer_Angular mObsvAngular;
//	Observer_Translational mObsvTranslational;
//	QuadLogger mQuadLogger;
//	VelocityEstimator mVelocityEstimator;
//	FeatureFinder mFeatureFinder;
//	TargetFinder mTargetFinder;
////	SensorManager mSensorManager;
//	MotorInterface mMotorInterface;
////	CommManager mCommManager;
////	VideoMaker mVideoMaker;
//
//	Time startTime;
//
//	// Make the appropriate connections
//	mTranslationController.setRotViconToPhone(mRotViconToPhone);
//	mTranslationController.setStartTime(startTime);
//	mTranslationController.setQuadLogger(&mQuadLogger);
//	mTranslationController.initialize();
//
//	mAttitudeThrustController.setStartTime(startTime);
//	mAttitudeThrustController.setQuadLogger(&mQuadLogger);
//	mAttitudeThrustController.setMotorInterface(&mMotorInterface);
//	mAttitudeThrustController.initialize();
////	mAttitudeThrustController.start();
//	mTranslationController.addListener(&mAttitudeThrustController);
//
//	mObsvAngular.initialize();
//	mObsvAngular.setStartTime(startTime);
//	mObsvAngular.setQuadLogger(&mQuadLogger);
////	mObsvAngular.start();
//	mObsvAngular.addListener(&mAttitudeThrustController);
//
//	mObsvTranslational.setQuadLogger(&mQuadLogger);
//	mObsvTranslational.setStartTime(startTime);
//	mObsvTranslational.setRotViconToPhone(mRotViconToPhone);
//	mObsvTranslational.initialize();
////	mObsvTranslational.start();
//	mObsvTranslational.addListener(&mTranslationController);
//	mObsvAngular.addListener(&mObsvTranslational);
//
//	mFeatureFinder.initialize();
//	mFeatureFinder.setStartTime(startTime);
//	mFeatureFinder.setQuadLogger(&mQuadLogger);
////	mFeatureFinder.start();
////	mSensorManager.addListener(&mFeatureFinder);
//
//	mTargetFinder.initialize();
//	mTargetFinder.setStartTime(startTime);
//	mTargetFinder.setQuadLogger(&mQuadLogger);
////	mTargetFinder.start();
////	mSensorManager.addListener(&mTargetFinder);
//	mTargetFinder.addListener(&mObsvTranslational);
//
//	mVelocityEstimator.initialize();
//	mVelocityEstimator.setStartTime(startTime);
//	mVelocityEstimator.setQuadLogger(&mQuadLogger);
//	mVelocityEstimator.setObserverTranslational(&mObsvTranslational);
//	mVelocityEstimator.setRotPhoneToCam(mRotPhoneToCam);
////	mVelocityEstimator.start();
//	mVelocityEstimator.addListener(&mObsvTranslational);
//	mFeatureFinder.addListener(&mVelocityEstimator);
//
////	mSensorManager.setStartTime(startTime);
////	mSensorManager.setQuadLogger(&mQuadLogger);
////	mSensorManager.initialize();
////	mSensorManager.start();
////	mObsvAngular.addListener(&mSensorManager);
////	mSensorManager.addListener(&mObsvAngular);
////	mSensorManager.addListener(&mObsvTranslational);
//
//	mQuadLogger.setStartTime(startTime);
//
////	mMotorInterface.addListener(&mObsvTranslational);
//	mMotorInterface.addListener(&mTranslationController);
//
//	// load data and manually feed them to the appropriate listener ... hopefully
//	list<shared_ptr<IData>> dataList;
//	dataList.clear();
//
//	int firstTime = -1;
//	int curTime = -1;
//	int endTimeDelta = 1e3;
//
//	string line;
//	string dataFilename = dataDir+"/phoneLog.txt";
//	ifstream file(dataFilename.c_str());
//	if(file.is_open())
//	{
//		getline(file, line); // first line is a throw-away
//		getline(file, line); // second line is also a throw-away
//
//		while(file.good() && curTime < firstTime+endTimeDelta)
//		{
//			getline(file, line);
//			stringstream ss(line);
//			double time;
//			int type;
//			ss >> time >> type;
//
//			if(firstTime == -1)
//				firstTime = time;
//			curTime = time;
//
//			switch(type)
//			{
//				case LOG_ID_ACCEL:
//					break;
//				case LOG_ID_GYRO:
//					break;
//				case LOG_ID_MAGNOMETER:
//					break;
//				case LOG_ID_PRESSURE:
//					break;
//				case LOG_ID_IMAGE:
//					break;
//				case LOG_ID_MOTOR_CMDS:
//					break;
//				case LOG_ID_PHONE_TEMP:
//					break;
//			}
//
//		}
//
//		file.close();
//	}
//
//    return 0;
}
