#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "TNT/tnt.h"
#include "TNT/jama_qr.h"
#include "TNT/jama_lu.h"
#include "TNT/jama_svd.h"
#include "TNT_Utils.h"

#include "Time.h"
#include "QuadLogger.h"
#include "Data.h"
#include "Rotation.h"

void loadData(const std::string &dataDir, const std::string &imgDir,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataImage>> &imageDataBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataVector<double>>> &gyroBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataVector<double>>> &accelBuffer);

void stateToComponents(const TNT::Array2D<double> &state,
					   ICSL::Quaternion &globalFrameQuat,
					   TNT::Array2D<double> &gyroBias,
					   TNT::Array2D<double> &velIMU,
					   TNT::Array2D<double> &accelBias,
					   TNT::Array2D<double> &posIMU,
					   ICSL::Quaternion &cameraFrameQuat,
					   TNT::Array2D<double> &posCamera);

void componentsToState(TNT::Array2D<double> &state,
					   const ICSL::Quaternion &globalFrameQuat,
					   const TNT::Array2D<double> &gyroBias,
					   const TNT::Array2D<double> &velIMU,
					   const TNT::Array2D<double> &accelBias,
					   const TNT::Array2D<double> &posIMU,
					   const ICSL::Quaternion &cameraFrameQuat,
					   const TNT::Array2D<double> &posCamera);

void  componentsToState(TNT::Array2D<double> &state,
						const TNT::Array2D<double> &globalFrameQuat,
						const TNT::Array2D<double> &gyroBias,
						const TNT::Array2D<double> &velIMU,
						const TNT::Array2D<double> &accelBias,
						const TNT::Array2D<double> &posIMU,
						const TNT::Array2D<double> &cameraFrameQuat,
						const TNT::Array2D<double> &posCamera);

void doTimeUpdate(double dt,
				  TNT::Array2D<double> &state,
				  TNT::Array2D<double> &cov,
				  TNT::Array2D<double> &dynNoiseCov,
				  const TNT::Array2D<double> &gyro,
				  const TNT::Array2D<double> &accel);

void doMeasurementUpdateStatic(TNT::Array2D<double> &state,
							   TNT::Array2D<double> &cov,
							   const TNT::Array2D<double> &C,
							   const TNT::Array2D<double> &meas,
							   const TNT::Array2D<double> &measCov);

void doMeasurementUpdateVision(TNT::Array2D<double> &state,
							   TNT::Array2D<double> &cov,
							   const std::vector<cv::Point2f> &points,
							   double focalLength, const cv::Point2f &center,
							   const TNT::Array2D<double> &measCov);

TNT::Array2D<double> getInitialIMUPos(const ICSL::Quaternion &cameraFrameQuat,
								 const std::list<std::shared_ptr<ICSL::Quadrotor::DataImage>>::const_iterator imgIter);

int main(int argv, char* argc[])
{
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
	using namespace std;
	using namespace TNT;
	using toadlet::egg::Log;
	using toadlet::egg::String;
	cout << "start chadding" << endl;

	string dataDir = "../data";
	string imgDir = dataDir + "/video";

	// Load the log file
	list<shared_ptr<DataVector<double>>> gyroBuffer, accelBuffer;
	list<shared_ptr<DataImage>> imageDataBuffer;
	loadData(dataDir, dataDir+"/video", imageDataBuffer, gyroBuffer, accelBuffer);

	// Initial orientation guesses (relative to IMU frame)
	SO3 cameraFrame(matmult(createRotMat(2,-0.5*(double)PI),
							      createRotMat(0,(double)PI)));
	SO3 globalFrame;
	Quaternion cameraFrameQuat = cameraFrame.getQuaternion();
	Quaternion globalFrameQuat = globalFrame.getQuaternion();

	Array2D<double> posIMU(3,1); // in the global frame
	posIMU[0][0] = 0;
	posIMU[1][0] = 0;
	posIMU[2][0] = 0.5;
	Array2D<double> velIMU(3,1); // in the global frame
	velIMU[0][0] = 0;
	velIMU[1][0] = 0;
	velIMU[2][0] = 0;
	Array2D<double> posCamera(3,1); // in IMU frame
	posCamera[0][0] = 0;
	posCamera[1][0] = 0;
	posCamera[2][0] = 0;

	Array2D<double> gyroBias(3,1);
	gyroBias[0][0] = -0.006;
	gyroBias[1][0] = -0.008;
	gyroBias[2][0] = -0.008;

	Array2D<double> accelBias(3,1);
	accelBias[0][0] = -0.07;
	accelBias[1][0] = -0.12;
	accelBias[2][0] = -0.3;

	// Kalman filter variables
	//  0. quat for orientation of global frame in imu frame
	//  1. 
	//  2. 
	//  3. 
	//  4. gyroscope bias
	//  5. 
	//  6. 
	//  7. velocity of the imu frame in the global frame
	//  8. 
	//  9. 
	// 10. accelerometer bias
	// 11. 
	// 12. 
	// 13. position of the imu in the global frame
	// 14. 
	// 15. 
	// 16. quat for orientation of the camera in the imu frame
	// 17. 
	// 18. 
	// 19. 
	// 20. position of the camera in the imu frame
	// 21. 
	// 22. 
	Array2D<double> kfState(23,1);
	componentsToState(kfState, globalFrameQuat, gyroBias, velIMU, accelBias, posIMU, cameraFrameQuat, posCamera);
	Quaternion globalFrameInit(globalFrameQuat);
	Quaternion cameraFrameInit(cameraFrameQuat);

	Array2D<double> kfCov = 1e-6*createIdentity(21.0);

	Array2D<double> dynNoiseCov(12,12,0.0);
	assignSubmat(dynNoiseCov,0,0,pow(1,2)*createIdentity(3.0));
	assignSubmat(dynNoiseCov,3,3,pow(1,2)*createIdentity(3.0));
	assignSubmat(dynNoiseCov,6,6,pow(1,2)*createIdentity(3.0));
	assignSubmat(dynNoiseCov,9,9,pow(1,2)*createIdentity(3.0));

	Array2D<double> idleMeasC(15,21,0.0);
	idleMeasC[0][0] = 1;
	idleMeasC[1][1] = 1;
	idleMeasC[2][2] = 1;
	idleMeasC[3][6] = 1;
	idleMeasC[4][7] = 1;
	idleMeasC[5][8] = 1;
	idleMeasC[6][12] = 1;
	idleMeasC[7][13] = 1;
	idleMeasC[8][14] = 1;
	idleMeasC[9][15] = 1;
	idleMeasC[10][16] = 1;
	idleMeasC[11][17] = 1;
	idleMeasC[12][18] = 1;
	idleMeasC[13][19] = 1;
	idleMeasC[14][20] = 1;
	Array2D<double> idleMeasCov(15,15,0.0);
	idleMeasCov[0][0] = idleMeasCov[1][1] = idleMeasCov[2][2] = 1e-4;
	idleMeasCov[3][3] = idleMeasCov[4][4] = idleMeasCov[5][5] = 1e-4;
	idleMeasCov[6][6] = idleMeasCov[7][7] = idleMeasCov[8][8] = 1e-4;
	idleMeasCov[9][9] = idleMeasCov[10][10] = idleMeasCov[11][11] = 1e-4;
	idleMeasCov[12][12] = idleMeasCov[13][13] = idleMeasCov[14][14] = 1e-4;

	Array2D<double> visionMeasCov(2,2);
	visionMeasCov[0][0] = 2*2;	visionMeasCov[0][1] = 0;
	visionMeasCov[1][0] = 0;	visionMeasCov[1][1] = 2*2;
	visionMeasCov = 1e6*visionMeasCov;

	///////////////////////////////////////////////////////////////////////////
	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);

	list<shared_ptr<DataImage>>::const_iterator imgIter = imageDataBuffer.begin();
	list<shared_ptr<DataVector<double>>>::const_iterator accelIter, gyroIter;
	accelIter = accelBuffer.begin();
	gyroIter = gyroBuffer.begin();

	list<shared_ptr<DataVector<double>>> imuEvents;
	imuEvents.insert(imuEvents.end(), gyroBuffer.begin(), gyroBuffer.end());
	imuEvents.insert(imuEvents.end(), accelBuffer.begin(), accelBuffer.end());
	imuEvents.sort(IData::timeSortPredicate);
	list<shared_ptr<DataVector<double>>>::const_iterator imuEventIter;
	imuEventIter = imuEvents.begin();

	Array2D<double> posIMU_init = getInitialIMUPos(cameraFrameQuat, imgIter);
	Array2D<double> posCamera_init = posCamera.copy();

	Time idleStopTime;
	idleStopTime.setTimeMS(40e3);

	int keypress = 0;
	cv::Mat img(240,320, CV_8UC3, cv::Scalar(0)), oldImg(240,320,CV_8UC3,cv::Scalar(0)), imgGray;
	int imgCnt = 0;
	Time curTime;
	shared_ptr<DataImage> curImgData, prevImgData;
	curImgData = *imgIter;
	imgIter++;
	Array2D<double> curAccel(3,1), curGyro(3,1);
	curAccel[0][0] = accelBias[0][0];
	curAccel[1][0] = accelBias[1][0];
	curAccel[2][0] = GRAVITY+accelBias[2][0];
	curGyro[0][0] = gyroBias[0][0];
	curGyro[1][0] = gyroBias[1][0];
	curGyro[2][0] = gyroBias[2][0];
	curAccel[2][0] = GRAVITY+accelBias[2][0];
	Time lastUpdateTime;
	lastUpdateTime.setTimeMS(0);
	while(keypress != (int)'q' && imgIter != imageDataBuffer.end())
	{
		prevImgData = curImgData;
		curImgData = *imgIter;
		curTime.setTime(curImgData->timestamp);
		curImgData->image->copyTo(img);
		curImgData->imageGray->copyTo(imgGray);
		double f = curImgData->focalLength;
		cv::Point2f center = curImgData->center;

		double dt;
		while(imuEventIter != imuEvents.end() && (*imuEventIter)->timestamp < curTime)
		{
			if(lastUpdateTime.getMS() == 0)
				dt = 0;
			else
				dt = Time::calcDiffNS(lastUpdateTime, (*imuEventIter)->timestamp)/1.0e9;
			doTimeUpdate(dt, kfState, kfCov, dynNoiseCov, curGyro, curAccel);
			lastUpdateTime.setTime((*imuEventIter)->timestamp);

			switch((*imuEventIter)->type)
			{
				case DATA_TYPE_ACCEL:
					curAccel.inject((*imuEventIter)->data);
					break;
				case DATA_TYPE_GYRO:
					curGyro.inject((*imuEventIter)->data);
					break;
			}

			imuEventIter++;
		}
		if(lastUpdateTime.getMS() == 0)
			dt = 0;
		else
			dt = Time::calcDiffNS(lastUpdateTime, (*imuEventIter)->timestamp)/1.0e9;
		doTimeUpdate(dt, kfState, kfCov, dynNoiseCov, curGyro, curAccel);
		lastUpdateTime.setTime((*imuEventIter)->timestamp);

		// measure update
		if(curTime < idleStopTime)
		{
			Array2D<double> meas(15,1);
			assignSubmat(meas,0,0,globalFrameInit.getVectorPart());
			meas[3][0] = 0;
			meas[4][0] = 0;
			meas[5][0] = 0;
			assignSubmat(meas,6,0,posIMU_init);
			assignSubmat(meas,9,0,cameraFrameInit.getVectorPart());
			assignSubmat(meas,12,0,posCamera_init);

			doMeasurementUpdateStatic(kfState, kfCov, idleMeasC, meas, idleMeasCov);
//			printArray("kfState:\t",kfState);
		}
		else
		{ 
			cv::Size sz(4,11);
			vector<cv::Point2f> centers;
			bool found = findCirclesGrid(imgGray, sz, centers, cv::CALIB_CB_ASYMMETRIC_GRID);
			cv::drawChessboardCorners(img, sz, cv::Mat(centers), found);


			if(found)
			{
				Array2D<double> measCov(2*centers.size(),2*centers.size(), 0.0);
				for(int i=0; i<measCov.dim1(); i+=2)
					assignSubmat(measCov,i,i,visionMeasCov);
				doMeasurementUpdateVision(kfState, kfCov, centers, f, center, measCov);

				printArray("kfState:\t",kfState);
			}
			else
				cout << "Pattern not found" << endl;

			imshow("chad", img);

			keypress = cv::waitKey(1) % 256;
		}

		if(imgCnt > 1300)
			return 0;

		imgIter++;
		imgCnt++;
	}

    return 0;
}

void loadData(const std::string &dataDir, const std::string &imgDir,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataImage>> &imageDataBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataVector<double>>> &gyroBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataVector<double>>> &accelBuffer)
{
	using namespace std;
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
	using namespace TNT;

	imageDataBuffer.clear();
	gyroBuffer.clear();
	accelBuffer.clear();

	// Camera calibration
	cv::Point2f center;
	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;
	cv::FileStorage fs;
	string calibFilename = dataDir + "/calib_640x480.yml";
	fs.open(calibFilename .c_str(), cv::FileStorage::READ);
	if( fs.isOpened() )
	{
		mCameraMatrix_640x480 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraDistortionCoeffs = shared_ptr<cv::Mat>(new cv::Mat());

		fs["camera_matrix"] >> *mCameraMatrix_640x480;
		fs["distortion_coefficients"] >> *mCameraDistortionCoeffs;
		cout << "Camera calib loaded from " << calibFilename.c_str() << endl;

		mCameraMatrix_320x240 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraMatrix_640x480->copyTo( *mCameraMatrix_320x240 );
		(*mCameraMatrix_320x240) = (*mCameraMatrix_320x240)*0.5;

		cout << "Loaded camera matrix" << endl;
	}
	else
		cout << "Failed to open " <<  calibFilename.c_str();
	fs.release();

	string line;
	string dataFilename = dataDir+"/log.txt";
	ifstream file(dataFilename.c_str());
	if(file.is_open())
	{
		getline(file, line); // first line is a throw-away
		getline(file, line); // second line is also a throw-away
		
		while(file.good())
		{
			getline(file, line);
			stringstream ss(line);
			double time;
			int type;
			ss >> time >> type;

			switch(type)
			{
				case LOG_ID_IMAGE:
					{
						int id;
						ss >> time >> id;

						shared_ptr<cv::Mat> img(new cv::Mat());
						shared_ptr<cv::Mat> imgGray(new cv::Mat());

						stringstream ss2;
						ss2 << "image_" << id << ".bmp";
						string imgFilename = imgDir+"/"+ss2.str();
						*img = cv::imread(imgFilename);
						if(img->data != NULL)
						{
							shared_ptr<DataImage> data(new DataImage());
							data->timestamp.setTimeMS(time);
							data->imageId = id;
							data->image = img;
							cv::cvtColor(*img, *imgGray, CV_BGR2GRAY); 
							data->imageGray = imgGray;
							data->imageFormat = IMG_FORMAT_BGR;
							data->cap = NULL;
							if(img->rows = 240)
								data->cameraMatrix = mCameraMatrix_320x240;
							else
								data->cameraMatrix = mCameraMatrix_640x480;
							data->focalLength = data->cameraMatrix->at<double>(0,0);
							data->center.x = data->cameraMatrix->at<double>(0,2);
							data->center.y = data->cameraMatrix->at<double>(1,2);
							data->distCoeffs = mCameraDistortionCoeffs;

							imageDataBuffer.push_back(data);
						}
					}
					break;
				case LOG_ID_ACCEL:
					{
						ss >> time;
						Array2D<double> accel(3,1);
						for(int i=0; i<3; i++)
							ss >> accel[i][0];
						shared_ptr<DataVector<double>> data(new DataVector<double>);
						data->timestamp.setTimeMS(time);
						data->data = accel.copy();
						data->type = DATA_TYPE_ACCEL;
						accelBuffer.push_back(data);
					}
					break;
				case LOG_ID_GYRO:
					{
						ss >> time;
						Array2D<double> gyro(3,1);
						for(int i=0; i<3; i++)
							ss >> gyro[i][0];
						shared_ptr<DataVector<double>> data(new DataVector<double>);
						data->timestamp.setTimeMS(time);
						data->data = gyro.copy();
						data->type = DATA_TYPE_GYRO;
						gyroBuffer.push_back(data);
					}
					break;
			}
		}

		file.close();
	}
	else
		cout << "Failed to open file: " << dataFilename << endl;
}

void stateToComponents(const TNT::Array2D<double> &state,
					   ICSL::Quaternion &cameraFrameQuat,
					   TNT::Array2D<double> &gyroBias,
					   TNT::Array2D<double> &velIMU,
					   TNT::Array2D<double> &accelBias,
					   TNT::Array2D<double> &posIMU,
					   ICSL::Quaternion &globalFrameQuat,
					   TNT::Array2D<double> &posCamera)
{
	cameraFrameQuat.set(state[0][0], state[1][0], state[2][0], state[3][0]);
	globalFrameQuat.set(state[16][0], state[17][0], state[18][0], state[19][0]);
	for(int i=0; i<3; i++)
	{
		gyroBias[i][0] = state[i+4][0];
		velIMU[i][0] = state[i+7][0];
		accelBias[i][0] = state[i+10][0];
		posIMU[i][0] = state[i+13][0];
		posCamera[i][0] = state[i+20][0];
	}
}

void  componentsToState(TNT::Array2D<double> &state,
						const ICSL::Quaternion &globalFrameQuat,
						const TNT::Array2D<double> &gyroBias,
						const TNT::Array2D<double> &velIMU,
						const TNT::Array2D<double> &accelBias,
						const TNT::Array2D<double> &posIMU,
						const ICSL::Quaternion &cameraFrameQuat,
						const TNT::Array2D<double> &posCamera)
{
	using namespace TNT;
	using namespace ICSL;
	assignSubmat(state,0,0,globalFrameQuat.toVector());
	assignSubmat(state,4,0,gyroBias);
	assignSubmat(state,7,0,velIMU);
	assignSubmat(state,10,0,accelBias);
	assignSubmat(state,13,0,posIMU);
	assignSubmat(state,16,0,cameraFrameQuat.toVector());
	assignSubmat(state,20,0,posCamera);
}

void  componentsToState(TNT::Array2D<double> &state,
						const TNT::Array2D<double> &globalFrameQuat,
						const TNT::Array2D<double> &gyroBias,
						const TNT::Array2D<double> &velIMU,
						const TNT::Array2D<double> &accelBias,
						const TNT::Array2D<double> &posIMU,
						const TNT::Array2D<double> &cameraFrameQuat,
						const TNT::Array2D<double> &posCamera)
{
	using namespace TNT;
	using namespace ICSL;
	assignSubmat(state,0,0,globalFrameQuat);
	assignSubmat(state,4,0,gyroBias);
	assignSubmat(state,7,0,velIMU);
	assignSubmat(state,10,0,accelBias);
	assignSubmat(state,13,0,posIMU);
	assignSubmat(state,16,0,cameraFrameQuat);
	assignSubmat(state,20,0,posCamera);
}

void doTimeUpdate(double dt,
				  TNT::Array2D<double> &state,
				  TNT::Array2D<double> &cov,
				  TNT::Array2D<double> &dynNoiseCov,
				  const TNT::Array2D<double> &gyro,
				  const TNT::Array2D<double> &accel)
{
	using namespace std;
	using namespace TNT;
	using namespace ICSL;

	Quaternion cameraFrame, globalFrame;
	Array2D<double> gyroBias(3,1), velIMU(3,1), accelBias(3,1), posIMU(3,1), posCamera(3,1);
	stateToComponents(state, globalFrame, gyroBias, velIMU, accelBias, posIMU, cameraFrame, posCamera);
	
	Array2D<double> gravity(3,1);
	gravity[0][0] = 0;
	gravity[1][0] = 0;
	gravity[2][0] = -ICSL::Constants::GRAVITY;

	Array2D<double> gyroC = gyro-gyroBias;
	Array2D<double> wx(3,3);
	wx[0][0] = 0;				wx[0][1] = -gyroC[2][0];	wx[0][2] = gyroC[1][0];
	wx[1][0] = gyroC[2][0];		wx[1][1] = 0;				wx[1][2] = -gyroC[0][0];
	wx[2][0] = -gyroC[1][0];	wx[2][1] = gyroC[0][0];		wx[2][2] = 0;

	Array2D<double> accelC = accel-accelBias;
	Array2D<double> ax(3,3);
	ax[0][0] = 0;				ax[0][1] = -accelC[2][0];	ax[0][2] = accelC[1][0];
	ax[1][0] = accelC[2][0];	ax[1][1] = 0;				ax[1][2] = -accelC[0][0];
	ax[2][0] = -accelC[1][0];	ax[2][1] = accelC[0][0];	ax[2][2] = 0;

	Array2D<double> W(4,4);
	W[0][0] = 0;
	assignSubmat(W,1,0,gyroC);
	assignSubmat(W,0,1,-1.0*transpose(gyroC));
	assignSubmat(W,1,1,-1.0*wx);

	Array2D<double> cameraFrameDeriv, globalFrameDeriv, gyroBiasDeriv, velIMUDeriv, accelBiasDeriv, posIMUDeriv, posCameraDeriv;
	globalFrameDeriv = 0.5*matmult(W,globalFrame.toVector());
	gyroBiasDeriv = Array2D<double>(3,1,0.0);
	velIMUDeriv = SO3(globalFrame.inv())*accelC+gravity;
	accelBiasDeriv = Array2D<double>(3,1,0.0);
	posIMUDeriv = velIMU.copy();
	cameraFrameDeriv = Array2D<double>(3,1,0.0);
	posCameraDeriv = Array2D<double>(3,1,0.0);

	Array2D<double> stateDeriv(23,1);
	componentsToState(stateDeriv, globalFrameDeriv, gyroBiasDeriv, velIMUDeriv, accelBiasDeriv,
					  posIMUDeriv, cameraFrameDeriv, posCameraDeriv);

	state += dt*stateDeriv;

	// need to renormalize the quaternions
	Array2D<double> q1, q2;
	q1 = submat(state,0,3,0,0);
	q2 = submat(state,16,19,0,0);
	normalize(q1);
	normalize(q2);
	assignRows(state,0,3,q1);
	assignRows(state,16,19,q2);

	// Now the covariance update
	// First the system matrix
	// Note that this is for the error system, which has a slightly different
	// state vector due to the rotations
	Array2D<double> A(21,21,0.0);
	assignSubmat(A,0,0,-dt*wx);
	A[0][3] = A[1][4] = A[2][5] = -dt;
	assignSubmat(A,6,0,-dt*matmult(SO3(globalFrame.inv()).toRotMat(),ax) );
	assignSubmat(A,6,9,-dt*globalFrame.inv().toRotMat());
	A[12][6] = A[13][7] = A[14][8] = dt;
	for(int i=0; i<A.dim1(); i++)
		A[i][i] += 1;

	// Then the noise gain matrix
	Array2D<double> G(21,12,0.0);
	G[0][0] = G[1][1] = G[2][2] = -dt; // gyro noise to imu attitude
	G[3][3] = G[4][4] = G[5][5] = dt; // gyro bias noise to gyro bias
	assignSubmat(G,6,6, -dt*globalFrame.inv().toRotMat()); // accel noise to imu velocity
	G[9][9] = G[10][10] = G[11][11] = dt; // gyro bias noise to imu position ... why?
	G[12][3] = G[13][4] = G[14][5] = dt;

	Array2D<double> Q = matmult(G,matmult(dynNoiseCov,transpose(G)));
	cov = matmult(transpose(A),matmult(cov, A))+Q;

//printArray("cov:\n",submat(cov,0,14,0,14));
	int chad=0;
}

void doMeasurementUpdateStatic(TNT::Array2D<double> &state,
							   TNT::Array2D<double> &cov,
							   const TNT::Array2D<double> &C,
							   const TNT::Array2D<double> &meas,
							   const TNT::Array2D<double> &measCov)
{
	using namespace TNT;
	using namespace ICSL;
	using namespace std;

	Array2D<double> temp = matmult(C, matmult(cov, transpose(C)))+measCov;
	JAMA::SVD<double> svd_temp(temp);
	Array2D<double> U, V, Sinv;
	svd_temp.getU(U);
	svd_temp.getV(V);
	svd_temp.getS(Sinv);
	for(int i=0; i<Sinv.dim1(); i++)
		Sinv[i][i] = 1.0/Sinv[i][i];
	Array2D<double> tempInv = matmult(V, matmult(Sinv, transpose(U)));
	Array2D<double> K = matmult(cov, matmult(transpose(C), tempInv));

	// The error vector is a bit different than the normal one
	// because of the rotation states
	Quaternion cameraFrame, globalFrame;
	Array2D<double> gyroBias(3,1), velIMU(3,1), accelBias(3,1), posIMU(3,1), posCamera(3,1);
	stateToComponents(state, globalFrame, gyroBias, velIMU, accelBias, posIMU, cameraFrame, posCamera);

	Array2D<double> err(meas.dim1(),1);
	assignSubmat(err,0,0,submat(meas,0,2,0,0)-globalFrame.getVectorPart());
	assignSubmat(err,3,0,submat(meas,3,5,0,0)-submat(state,7,9,0,0));
	assignSubmat(err,6,0,submat(meas,6,8,0,0)-submat(state,13,15,0,0));
	assignSubmat(err,9,0,submat(meas,9,11,0,0)-cameraFrame.getVectorPart());
	assignSubmat(err,12,0,submat(meas,12,14,0,0)-submat(state,20,22,0,0));

	Array2D<double> delta = matmult(K,err);
	cov = matmult(createIdentity(1.0*K.dim1())-matmult(K,C), cov);

	Quaternion globalFrameDelta(1,0.5*delta[0][0], 0.5*delta[1][0], 0.5*delta[2][0]);
	globalFrameDelta.normalize();
	globalFrame = globalFrameDelta*globalFrame;
	Quaternion cameraFrameDelta(1,0.5*delta[15][0], 0.5*delta[16][0], 0.5*delta[17][0]);
	cameraFrameDelta.normalize();
	cameraFrame = cameraFrameDelta*cameraFrame;

	gyroBias += submat(delta,3,5,0,0);
	velIMU += submat(delta,6,8,0,0);
	accelBias += submat(delta,9,11,0,0);
	posIMU += submat(delta,12,14,0,0);
	posCamera += submat(delta,18,20,0,0);

	componentsToState(state, globalFrame, gyroBias, velIMU, accelBias, posIMU, cameraFrame, posCamera);

	int chad=0;
}

void doMeasurementUpdateVision(TNT::Array2D<double> &state,
							   TNT::Array2D<double> &cov,
							   const std::vector<cv::Point2f> &points,
							   double focalLength, const cv::Point2f &center,
							   const TNT::Array2D<double> &measCov)
{
	using namespace TNT;
	using namespace ICSL;
	using namespace std;

	int numPoints = points.size();

	Quaternion cameraFrameQuat, globalFrameQuat;
	Array2D<double> gyroBias(3,1), velIMU(3,1), accelBias(3,1), posIMU(3,1), posCamera(3,1);
	stateToComponents(state, globalFrameQuat, gyroBias, velIMU, accelBias, posIMU, cameraFrameQuat, posCamera);
	SO3 globalFrame(globalFrameQuat);
	SO3 cameraFrame(cameraFrameQuat);

	// Find the 3D pos of all the points
	vector<Array2D<double>> points3D(numPoints);
	double xPitch = 0.034;
	double yPitch = 0.034;
	int nCols = 4;
	int nRows= 11;
	Array2D<double> p(3,1);
	for(int i=0; i<numPoints; i++)
	{
		int gridX = i/nCols;
		float gridY = i%nCols+ 0.5*(gridX%2);

		p[0][0] = gridX*xPitch;
		p[1][0] = gridY*yPitch;
		p[2][0] = 0;

		points3D[i] = cameraFrame.inv()*globalFrame*(p-posIMU) - cameraFrame.inv()*posCamera;
	}

	// Build up the observation matrix
	Array2D<double> C(2*numPoints,21);
	Array2D<double> temp(3,21,0.0);
	Array2D<double> Jcam(2,3), Jtg, Jtc, Jpi, Jpc;
	for(int i=0; i<numPoints; i++)
	{
		p = points3D[i];
		double x = points3D[i][0][0];
		double y = points3D[i][1][0];
		double z = points3D[i][2][0];

		Jcam[0][0] = 1.0/z;	Jcam[0][1] = 0;		Jcam[0][2] = -x/z/z;
		Jcam[1][0] = 0;		Jcam[1][1] = 1.0/z;	Jcam[1][2] = -y/z/z;

		Jtg = matmult(cameraFrame.inv().toRotMat(),
					 SO3_LieAlgebra(globalFrame*(p-posIMU)).toMatrix());
		Jtc = matmult(-1.0*(cameraFrame.inv().toRotMat()),
					 SO3_LieAlgebra(globalFrame*(p-posIMU)-posCamera).toMatrix());
		Jpi = -1.0*(cameraFrame.inv()*globalFrame).toRotMat();
		Jpc = -1.0*cameraFrame.inv().toRotMat();

		assignSubmat(temp,0,0,Jtg);
		assignSubmat(temp,0,12,Jpi);
		assignSubmat(temp,0,15,Jtc);
		assignSubmat(temp,0,18,Jpc);

		assignSubmat(C,2*i,0, matmult(Jcam, temp));
	}

	// Now build the measurement vector
	Array2D<double> meas(2*numPoints,1);
	double f = focalLength;
	for(int i=0; i<numPoints; i++)
	{
		meas[2*i][0] = (points[i].x-center.x)/f;
		meas[2*i+1][0] = (points[i].y-center.y)/f;
	}

	// And the estimates
	Array2D<double> est(2*numPoints,1);
	for(int i=0; i<numPoints; i++)
	{
		est[2*i][0] = points3D[i][0][0]/points3D[i][2][0];
		est[2*i+1][0] = points3D[i][1][0]/points3D[i][2][0];
//printArray("points3D:\t",points3D[i]);
	}

//printArray("meas:\t",meas);
//printArray("est:\t",est);

	// Now do the Kalman filter updates
	Array2D<double> temp2 = matmult(C, matmult(cov, transpose(C)))+measCov;
	JAMA::SVD<double> svd_temp2(temp2);
	Array2D<double> U, V, Sinv;
	svd_temp2.getU(U);
	svd_temp2.getV(V);
	svd_temp2.getS(Sinv);
	for(int i=0; i<Sinv.dim1(); i++)
		Sinv[i][i] = 1.0/Sinv[i][i];
	Array2D<double> temp2Inv = matmult(V, matmult(Sinv, transpose(U)));
	Array2D<double> K = matmult(cov, matmult(transpose(C), temp2Inv));

	Array2D<double> err = meas-est;
	Array2D<double> delta = matmult(K,err);

	cov = matmult(createIdentity(1.0*K.dim1())-matmult(K,C), cov);

	// and, finally, apply delta to the actual state vector
	Quaternion globalFrameDelta(1,0.5*delta[0][0], 0.5*delta[1][0], 0.5*delta[2][0]);
	globalFrameDelta.normalize();
	globalFrameQuat = globalFrameDelta*globalFrameQuat;
	Quaternion cameraFrameDelta(1,0.5*delta[15][0], 0.5*delta[16][0], 0.5*delta[17][0]);
	cameraFrameDelta.normalize();
	cameraFrameQuat = cameraFrameDelta*cameraFrameQuat;

	gyroBias += submat(delta,3,5,0,0);
	velIMU += submat(delta,6,8,0,0);
	accelBias += submat(delta,9,11,0,0);
	posIMU += submat(delta,12,14,0,0);
	posCamera += submat(delta,18,20,0,0);

	componentsToState(state, globalFrameQuat, gyroBias, velIMU, accelBias, posIMU, cameraFrameQuat, posCamera);

	int chad=0;
}

TNT::Array2D<double> getInitialIMUPos(const ICSL::Quaternion &cameraFrameQuat,
								 const std::list<std::shared_ptr<ICSL::Quadrotor::DataImage>>::const_iterator imgIter)
{
	using namespace TNT;
	using namespace std;
	using namespace ICSL;

	double f = (*imgIter)->focalLength;
	cv::Point2f center = (*imgIter)->center;

	cv::Size sz(4,11);
	cv::Mat imgGray;
	(*imgIter)->imageGray->copyTo(imgGray);
	vector<cv::Point2f> points;
	bool found = findCirclesGrid(imgGray, sz, points, cv::CALIB_CB_ASYMMETRIC_GRID);
	int numPoints = points.size();

	vector<Array2D<double>> points3D(numPoints);
	double xPitch = 0.034;
	double yPitch = 0.034;
	int nCols = 4;
	int nRows= 11;
	cv::Point2f avgPos, expectedPos;
	for(int i=0; i<numPoints; i++)
		avgPos += 1.0/f*(points[i]-center);
	avgPos = 1.0/numPoints*avgPos;
	expectedPos = 1.0/numPoints*expectedPos;

	double d = cv::norm(points[numPoints-1]-points[0])/f;
	double dNom = sqrt(pow(xPitch*(nCols-1),2)+pow(yPitch*(nRows-1),2));
	
	Array2D<double> camPos(3,1);
	camPos[2][0] = -dNom/d;
	camPos[0][0] = -avgPos.x*camPos[2][0];
	camPos[1][0] = -avgPos.y*camPos[2][0];

	Array2D<double> imuPos = SO3(cameraFrameQuat.inv())*camPos;

printArray("initial cam pos:\t",camPos);
printArray("initial imu pos:\t",imuPos);

	return imuPos;
}
