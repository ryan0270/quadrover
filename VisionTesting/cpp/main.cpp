#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <memory>
#include <vector>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>

#include "toadlet/egg.h"

#include "../../Rover/cpp/Data.h"
#include "../../Rover/cpp/TNT/tnt.h"
#include "../../Rover/cpp/TNT/jama_cholesky.h"
#include "../../Rover/cpp/TNT/jama_eig.h"
#include "../../Rover/cpp/TNT_Utils.h"
#include "../../Rover/cpp/Time.h"
#include "../../Rover/cpp/constants.h"

#include "mapFuncs.h"

using namespace std;
using namespace ICSL::Rover;
using namespace ICSL::Quadrotor;
using namespace ICSL::Constants;
using namespace TNT;

void loadPhoneLog(String filename, 
				vector<pair<int, Time> > &imgIdList,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList,
				list<shared_ptr<DataVector> > &errCovList
				);
void loadPcLog(String filename,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList
		);
vector<string> tokenize(string str);

enum LogIDs
{
	LOG_ID_ACCEL = 1,
	LOG_ID_GYRO = 2,
	LOG_ID_MAGNOMETER = 3,
	LOG_ID_PRESSURE = 4,
	LOG_ID_IMAGE = 10,
	LOG_ID_GYRO_BIAS = -1003,
	LOG_ID_OBSV_ANG_INNOVATION = -1004,
	LOG_ID_OPTIC_FLOW = 12345,
	LOG_ID_OBSV_TRANS_ATT_BIAS = -710,
	LOG_ID_OBSV_TRANS_FORCE_GAIN = -711,
	LOG_ID_BAROMETER_HEIGHT = 1234,
	LOG_ID_MOTOR_CMDS = -1000,
	LOG_ID_CUR_ATT = -1002,
	LOG_ID_CUR_TRANS_STATE = -1012,
	LOG_ID_RECEIVE_VICON = 700,
	LOG_ID_CAMERA_POS = 800,
	LOG_ID_KALMAN_ERR_COV = -720,
};

int main(int argv, char* argc[])
{
	cout << "start chadding" << endl;

	string imgDir = "../video";

	vector<pair<int, Time> > imgIdList;
	list<shared_ptr<DataVector> > attDataList, transDataList, errCovList;
	loadPhoneLog("../log.txt", imgIdList, attDataList, transDataList, errCovList);
	if(imgIdList.size() == 0)
	{
		cout << "Failed loading data" << endl;
		return 0;
	}

	// preload all images
	list<pair<int, cv::Mat> > imgList;
	int imgId = 1700;
	for(int i=0; i<10; i++)
	{
		cv::Mat img;
		while(img.data == NULL)
		{
			stringstream ss;
			ss << "img_" << ++imgId << ".bmp";
			img = cv::imread(imgDir+"/"+ss.str());
		}

		imgList.push_back(pair<int, cv::Mat>(imgId, img));
	}

	list<shared_ptr<DataVector> > viconAttDataList, viconTransDataList;
	loadPcLog("../pcData_fullState.txt", viconAttDataList, viconTransDataList);

	Array2D<double> rotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI), createRotMat(0,(double)PI));
	Array2D<double> rotCamToPhone2 = blkdiag(rotCamToPhone, rotCamToPhone);
	Array2D<double> rotCamToPhone3 = blkdiag(rotCamToPhone, blkdiag(rotCamToPhone, rotCamToPhone));

	Array2D<double> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<double> rotPhoneToCam2 = transpose(rotCamToPhone2);
	Array2D<double> rotPhoneToCam3 = transpose(rotCamToPhone3);

	int keypress = 0;
	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);

	cv::Mat img, imgGray, imgGrayRaw;
	vector<cv::Point2f> prevPoints, curPoints;
	Time prevTime, curTime;
	Array2D<double> attPrev, attCur, attChange;


	Array2D<double> Sv = 0.1*0.1*createIdentity(3);
	double sz = 0.02;
//		double sz = errCov[2][0];
	double focalLength = 3.7*640/5.76;
	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = 2*5*5;
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

	tbb::task_scheduler_init init;

	cout << "//////////////////////////////////////////////////" << endl;

	//////////////////////////////////////////////////////////////////
	// using surf features
	//////////////////////////////////////////////////////////////////
//	{
//		curTime.setTimeMS(0);
//		attPrev = createIdentity(3);
//		attCur = createIdentity(3);
//		attChange = createIdentity(3);
//		int imgIdx = 0;
//		double maxSigma = 30;
//		list<pair<int, cv::Mat> >::iterator iter_imgList;
//		iter_imgList = imgList.begin();
//		long long numFeaturesAccum = 0;
//		vector<cv::KeyPoint> prevKp, curKp;
//		cv::Mat prevDescriptors, curDescriptors;
//
//		fstream fs("surfResults.txt", fstream::out);
//		Time begin;
//		while(keypress != (int)'q' && iter_imgList != imgList.end())
//		{
//			imgId = iter_imgList->first;
//			img = iter_imgList->second;
//			iter_imgList++;
//			while(imgIdList[imgIdx].first < imgId && imgIdx < imgIdList.size()) 
//				imgIdx++;
//			if(imgIdx == imgIdList.size() )
//			{
//				cout << "imgIdx exceeded vector" << endl;
//				return 0;
//			}
//			prevTime.setTime(curTime);
//			curTime = imgIdList[imgIdx].second;
//			Array2D<double> attState  = Data::interpolate(curTime, attDataList);
//			Array2D<double> transState = Data::interpolate(curTime, transDataList);
//			Array2D<double> errCov = Data::interpolate(curTime, errCovList);
//			Array2D<double> viconAttState = Data::interpolate(curTime, viconAttDataList);
//			Array2D<double> viconTransState = Data::interpolate(curTime, viconTransDataList);
//
//			// Rotate to camera coords
//			attState = matmult(rotPhoneToCam2, attState);
//			transState = matmult(rotPhoneToCam2, transState);
//			errCov = matmult(rotPhoneToCam3, errCov);
//			viconAttState = matmult(rotPhoneToCam2, viconAttState);
//			viconTransState = matmult(rotPhoneToCam2, viconTransState);
//
//			attPrev.inject(attCur);
//			attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
////				attChange.inject( matmult(attCur, transpose(attPrev)) );
//			attChange.inject( matmult(transpose(attCur), attPrev) );
//
//			/////////////////////////////////////////////////////
//			Time start;
//			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
//			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);
//
//			curKp.swap(prevKp);
//			curKp.clear();
//			int minHessian = 800;
//			cv::SurfFeatureDetector detector(minHessian);
//			detector.extended = 0; // 64-bit descriptors
//			detector.upright = 1; // don't compute orientation
//			detector.detect(imgGray, curKp);
//			numFeaturesAccum += curKp.size();
//
//			prevDescriptors = curDescriptors;
//			curDescriptors.release();
//			curDescriptors.data = NULL;
//			cv::SurfDescriptorExtractor extractor;
//			extractor.extended = 0; // 64-bit descriptors
//			extractor.upright = 1; // don't compute orientation
//			extractor.compute(imgGray, curKp, curDescriptors);
//
//			cv::FlannBasedMatcher matcher;
//			vector<cv::DMatch> matches;
//			matcher.match(prevDescriptors, curDescriptors, matches);
//
//			float minDist = 100;
//			for(int i=0; i<matches.size(); i++)
//				minDist = min(matches[i].distance, minDist);
//
//			vector<cv::DMatch> goodMatches;
//			for(int i=0; i<matches.size(); i++)
//				if(matches[i].distance < 2.0*minDist) goodMatches.push_back(matches[i]);
//
//			int N1, N2;
//			N1 = N2 = goodMatches.size();
//			Array2D<double> C(N1+1, N2+1, 0.0); // the extra row and column will stay at zero
//			vector<cv::Point2f> prevPoints(N1), curPoints(N2);
//			for(int i=0; i<N1; i++)
//			{
//				C[i][i] = 1;
//
//				int idx1 = goodMatches[i].queryIdx;
//				int idx2 = goodMatches[i].trainIdx;
//				prevPoints[i] = prevKp[idx1].pt;
//				curPoints[i] = curKp[idx2].pt;
//			}
//
//			// MAP velocity and height
//			cv::Point2f center(320,240);
//			for(int i=0; i<curPoints.size(); i++)
//				curPoints[i] -= center;
//			Array2D<double> mv(3,1,0.0);
//			mv[0][0] = viconTransState[3][0];
//			mv[1][0] = viconTransState[4][0];
//			mv[2][0] = viconTransState[5][0];
//
//			double mz = -viconTransState[2][0]; // in camera coords, z is flipped
//			mz -= 0.1; // camera to vicon markers offset
//
//			double dt;
//			if(prevTime.getMS() > 0)
//				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
//			else
//				dt = 0;
//			Array2D<double> omega = logSO3(attChange, dt);
//
//			if(curPoints.size() > 0)
//			{
//				Array2D<double> vel;
//				double z;
//				computeMAPEstimate(vel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
//
//				fs << curTime.getMS() << "\t" << 98 << "\t";
//				for(int i=0; i<vel.dim1(); i++)
//					fs << vel[i][0] << "\t";
//				fs << endl;
//				fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;
//			}
//
////			for(int i=0; i<curPoints.size(); i++)
////				cv::circle(img, curPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
////			imshow("chad",img);
////			keypress = cv::waitKey() % 256;
//
//			img.release();
//			img.data = NULL;
//		}
//
//		fs.close();
//
//		cout << "Avg num surf features: " << ((double)numFeaturesAccum)/imgList.size() << endl;
//		cout << "Surf time: " << begin.getElapsedTimeMS()/1.0e3 << endl;
//
//	}

	cout << "//////////////////////////////////////////////////" << endl;

	//////////////////////////////////////////////////////////////////
	// Once for new algo
	//////////////////////////////////////////////////////////////////
	{
		curTime.setTimeMS(0);
		attPrev = createIdentity(3);
		attCur = createIdentity(3);
		attChange = createIdentity(3);
		int imgIdx = 0;
		double maxSigma = 30;
		list<pair<int, cv::Mat> >::iterator iter_imgList;
		iter_imgList = imgList.begin();
		long long numFeaturesAccum = 0;

		fstream fs("mapResults.txt", fstream::out);
		Time begin;
		while(keypress != (int)'q' && iter_imgList != imgList.end())
		{
			imgId = iter_imgList->first;
			img = iter_imgList->second;
			iter_imgList++;
			while(imgIdList[imgIdx].first < imgId && imgIdx < imgIdList.size()) 
				imgIdx++;
			if(imgIdx == imgIdList.size() )
			{
				cout << "imgIdx exceeded vector" << endl;
				return 0;
			}
			prevTime.setTime(curTime);
			curTime = imgIdList[imgIdx].second;
			Array2D<double> attState  = Data::interpolate(curTime, attDataList);
			Array2D<double> transState = Data::interpolate(curTime, transDataList);
			Array2D<double> errCov = Data::interpolate(curTime, errCovList);
			Array2D<double> viconAttState = Data::interpolate(curTime, viconAttDataList);
			Array2D<double> viconTransState = Data::interpolate(curTime, viconTransDataList);

			// Rotate to camera coords
			attState = matmult(rotPhoneToCam2, attState);
			transState = matmult(rotPhoneToCam2, transState);
			errCov = matmult(rotPhoneToCam3, errCov);
			viconAttState = matmult(rotPhoneToCam2, viconAttState);
			viconTransState = matmult(rotPhoneToCam2, viconTransState);

			attPrev.inject(attCur);
			attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
//			attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );

			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);

			int maxPoints= 400;
			double qualityLevel = 0.05;
			double minDistance = maxSigma;
			int blockSize = 5;
			bool useHarrisDetector = false;
			curPoints.swap(prevPoints);
			cv::goodFeaturesToTrack(imgGray, curPoints, maxPoints, qualityLevel, minDistance, cv::noArray(), blockSize, useHarrisDetector);
			numFeaturesAccum += curPoints.size();

			/////////////////////////////////////////////////////
			//  Prior distributions
			cv::Point2f center(320,240);
			for(int i=0; i<curPoints.size(); i++)
				curPoints[i] -= center;
			Array2D<double> mv(3,1,0.0);
			mv[0][0] = viconTransState[3][0];
			mv[1][0] = viconTransState[4][0];
			mv[2][0] = viconTransState[5][0];
			double mz = -viconTransState[2][0]; // in camera coords, z is flipped
			mz -= 0.1; // camera to vicon markers offset
			double dt;
			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;
			Array2D<double> omega = logSO3(attChange, dt);
			vector<pair<Array2D<double>, Array2D<double> > > priorDistList;
			priorDistList = calcPriorDistributions(prevPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);

			// Correspondence
			Array2D<double> C;
			C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv);

			// MAP velocity and height
			if(prevPoints.size() > 0)
			{
				Array2D<double> vel;
				double z;
				computeMAPEstimate(vel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);

				fs << curTime.getMS() << "\t" << 98 << "\t";
				for(int i=0; i<vel.dim1(); i++)
					fs << vel[i][0] << "\t";
				fs << endl;
				fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;
			}

			//////////////////////////////////////////////////
			//Drawing
			// prior distributions
			if(priorDistList.size() > 0)
				maxSigma = 0;
//			cv::Mat overlay = img.clone();
			for(int i=0; i<priorDistList.size(); i++)
			{
				Array2D<double> Sd = priorDistList[i].second;
				JAMA::Eigenvalue<double> eig_Sd(Sd);
				Array2D<double> V, D;
				eig_Sd.getV(V);
				eig_Sd.getD(D);

				for(int j=0; j<D.dim1(); j++)
					maxSigma = max(maxSigma, sqrt(D[j][j]));

//				cv::Point2f pt(iter_priorDistList->first[0][0], iter_priorDistList->first[1][0]);
//				double width = 2*sqrt(D[0][0]);
//				double height = 2*sqrt(D[1][1]);
//				double theta = atan2(V[1][0], V[0][0]);
//				cv::RotatedRect rect(pt+center, cv::Size(width, height), theta);
//				cv::ellipse(overlay, rect, cv::Scalar(255,0,0), -1);
			}
//			double opacity = 0.3;
//			cv::addWeighted(overlay, opacity, img, 1-opacity, 0, img);

			// current points
//			for(int i=0; i<curPoints.size(); i++)
//				cv::circle(img, curPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
//			imshow("chad",img);
//	
//			keypress = cv::waitKey() % 256;

			img.release();
			img.data = NULL;
		}

		fs.close();

		cout << "Avg num features: " << ((double)numFeaturesAccum)/imgList.size() << endl;
		cout << "New total time: " << begin.getElapsedTimeMS()/1.0e3 << endl;

	}

    return 0;
}

vector<string> tokenize(string str)
{
	istringstream iss(str);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens) );
	return tokens;
}

void loadPhoneLog(String filename, 
				vector<pair<int, Time> > &imgIdList,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList,
				list<shared_ptr<DataVector> > &errCovList
				)
{
	imgIdList.clear();
	angleStateList.clear();
	transStateList.clear();
	errCovList.clear();

	string line;
	ifstream file(filename.c_str());
	if(file.is_open())
	{
		getline(file, line); // first line is a throw-away
		vector<string> tokens;

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
						int imgID;
						ss >> imgID;

						Time t;
						t.setTimeMS(time);
						pair<int, Time> data(imgID, t);
						imgIdList.push_back(data);
					}
					break;
				case LOG_ID_CUR_ATT:
					{
						Array2D<double> attState(6,1);
						for(int i=0; i<6; i++)
							ss >> attState[i][0];

						shared_ptr<DataVector> data(new DataVector());
						data->timestamp.setTimeMS(time);
						data->data = attState.copy();
						angleStateList.push_back(data);
					}
					break;
				case LOG_ID_CUR_TRANS_STATE:
					{
						Array2D<double> transState(6,1);
						for(int i=0; i<6; i++)
							ss >> transState[i][0];

						shared_ptr<DataVector> data(new DataVector());
						data->timestamp.setTimeMS(time);
						data->data = transState.copy();
						transStateList.push_back(data);
					}
					break;
				case LOG_ID_KALMAN_ERR_COV:
					{
						Array2D<double> errCov(9,1);
						for(int i=0; i<9; i++)
							ss >> errCov[i][0];

						shared_ptr<DataVector> data(new DataVector());
						data->timestamp.setTimeMS(time);
						data->data = errCov.copy();
						errCovList.push_back(data);

					}
					break;
			}
		}

		file.close();
	}
	else
		cout << "Couldn't find " << filename.c_str() << endl;
}

void loadPcLog(String filename, 
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList
				)
{
	angleStateList.clear();
	transStateList.clear();

	Array2D<double> rotViconToQuad = createRotMat(0, (double)PI);
	Array2D<double> rotQuadToPhone = matmult(createRotMat(2,-0.25*PI), createRotMat(0,(double)PI));
	Array2D<double> rotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI), createRotMat(0,(double)PI));
	Array2D<double> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<double> rotViconToPhone = matmult(rotQuadToPhone, rotViconToQuad);

	Array2D<double> rotQuadToPhone2 = blkdiag(rotQuadToPhone, rotQuadToPhone);
	Array2D<double> rotViconToPhone2 = blkdiag(rotViconToPhone, rotViconToPhone);

	string line;
	ifstream file(filename.c_str());
	if(file.is_open())
	{
		vector<string> tokens;

		while(file.good())
		{
			getline(file, line);
			stringstream ss(line);
			double time;
			int type;
			ss >> time >> type;

			// This file is assumed to be all state data
			Array2D<double> angleState(6,1), transState(6,1);
			for(int i=0; i<6; i++)
				ss >> angleState[i][0];
			for(int i=0; i<6; i++)
				ss >> transState[i][0];

			angleState = matmult(rotQuadToPhone2, angleState);
			transState = matmult(rotViconToPhone2, transState);

			shared_ptr<DataVector> dataAngle(new DataVector()), dataTrans(new DataVector());;
			dataAngle->timestamp.setTimeMS(time);
			dataAngle->data = angleState.copy();
			dataTrans->timestamp.setTimeMS(time);
			dataTrans->data = transState.copy();
	
			angleStateList.push_back(dataAngle);
			transStateList.push_back(dataTrans);
		}

		file.close();
	}
	else
		cout << "Couldn't find " << filename.c_str() << endl;
}
