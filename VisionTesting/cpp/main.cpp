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
#include "../../Rover/cpp/QuadLogger.h"

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
				list<shared_ptr<DataVector> > &errCovDataList,
				list<shared_ptr<DataVector> > &motorCmdsDataList,
				list<shared_ptr<DataVector> > &thrustDirDataList
				);
void loadPcLog(String filename,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList
		);
vector<string> tokenize(string str);

void findPoints(cv::Mat const &img, vector<cv::Point2f> &pts, float const &minDistance, float const &qualityLevel);
void findPoints(cv::Mat const &img, vector<cv::KeyPoint> &pts, float const &minDistance, float const &qualityLevel);

static void HarrisResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize, float harris_k);
static void EigenValResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize);

void doTimeUpdateKF(Array2D<double> const &accel, double const &dt, Array2D<double> &state, Array2D<double> &errCov, Array2D<double> const &dynCov);
void doMeasUpdateKF_posOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov);
void doMeasUpdateKF_velOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov);
Time applyData(Time const &curTime, shared_ptr<Data> const &data, 
				Array2D<double> &kfState, Array2D<double> &kfErrCov, Array2D<double> const &kfMeasCov, Array2D<double> const &kfDynCov,
				double &curThrust, Array2D<double> &curThrustDir, Array2D<double> &curGravDir, 
				double const &mass);

int main(int argv, char* argc[])
{
	cout << "start chadding" << endl;

	string imgDir = "../video";

	vector<pair<int, Time> > imgIdList;
	list<shared_ptr<DataVector> > attDataList, transDataList, errCovDataList, motorCmdsDataList, thrustDirDataList;
	loadPhoneLog("../log.txt", imgIdList, attDataList, transDataList, errCovDataList, motorCmdsDataList, thrustDirDataList);
	if(imgIdList.size() == 0)
	{
		cout << "Failed loading data" << endl;
		return 0;
	}

	// preload all images
	list<pair<int, cv::Mat> > imgList;
	int imgId = 1700;
	for(int i=0; i<500; i++)
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

	Array2D<double> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<double> rotPhoneToCam2 = transpose(rotCamToPhone2);

	int keypress = 0;
	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);

	cv::Mat img, imgGray, imgGrayRaw, imgPrev;
	vector<cv::Point2f> prevPoints, curPoints;
	Time prevTime, curTime;
	Array2D<double> attPrev, attCur, attChange;

	Array2D<double> Sv(3,3,0.0);
	double sz;
	double focalLength = 3.7*640/5.76;
	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = 2*pow(5,2);
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

	cv::Point2f center(320,240);

	tbb::task_scheduler_init init;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// KF vars
	Array2D<double> kfState(6,1,0.0), kfErrCov(6,6,0.0);
	Array2D<double> kfA = createIdentity(6);
	Array2D<double> kfB(6,3,0.0);
	Array2D<double> kfDynCov = 0.02*0.02*createIdentity(6);
	Array2D<double> kfMeasCov(6,6,0.0);
	kfMeasCov[0][0] = kfMeasCov[1][1] = kfMeasCov[2][2] = 0.001*0.001;
	kfMeasCov[3][3] = kfMeasCov[4][4] = 0.1*0.1;
	kfMeasCov[5][5] = 0.5*0.5;
	Array2D<double> kfPosMeasCov = submat(kfMeasCov,0,2,0,2);
	Array2D<double> kfVelMeasCov = submat(kfMeasCov,3,5,3,5);
	double mass = 1.1; // kg
	double thrust = mass*GRAVITY;
	Array2D<double> accel(3,1,0.0);
	Array2D<double> attBias(3,1);
	attBias[0][0] = 0.004; attBias[1][0] = 0.018; attBias[2][0] = 0.00;
	Array2D<double> e3(3,1,0.0);
	e3[2][0] = 1;
	Array2D<double> curGravDir = -1.0*e3.copy();
	Array2D<double> thrustDir(3,1,0.0);
	thrustDir[2][0] = 1;
	Array2D<double> motorCmds(4,1);
	////////////////////////////////////////////////////////////////////////////////////////////////////

	cout << "//////////////////////////////////////////////////" << endl;

	float qualityLevel = 0.01;
	float minDistance = 30;

//	int maxCorners = 1000;
//	double qualityLevel = 0.02;
//	double minDistance = 30;
//	int blockSize = 3;
//	bool useHarrisDetector = false;
//	double k=0.04;
//	cv::GoodFeaturesToTrackDetector featureDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k);

	bool useViconState = false;

	//////////////////////////////////////////////////////////////////
	// using ORB descriptors
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
//		long long numMatchesAccum = 0;
//		vector<cv::KeyPoint> prevKp, curKp;
//		cv::Mat prevDescriptors, curDescriptors;
//
//		Array2D<double> attState(6,1), transState(6,1), viconAttState(6,1), viconTransState(6,1);
//		Array2D<double> errCov1(9,1), errCov(6,6,0.0);
//
//		Array2D<double> mv(3,1), omega(3,1), vel;
//		double mz, dt, z;
//
//		fstream fs("../orbResults.txt", fstream::out);
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
//			attState.inject(Data::interpolate(curTime, attDataList));
//			transState.inject(Data::interpolate(curTime, transDataList));
//			viconAttState.inject(Data::interpolate(curTime, viconAttDataList));
//			viconTransState.inject(Data::interpolate(curTime, viconTransDataList));
//
//			errCov1.inject(Data::interpolate(curTime, errCovDataList));
//			for(int i=0; i<3; i++)
//			{
//				errCov[i][i] = errCov1[i][0];
//				errCov[i][i+3] = errCov1[i+3][0];
//				errCov[i+3][i+3] = errCov1[i+6][0];
//			}
//
//			// Rotate to camera coords
//			attState = matmult(rotPhoneToCam2, attState);
//			transState = matmult(rotPhoneToCam2, transState);
//			errCov = matmult(rotPhoneToCam2, matmult(errCov, rotCamToPhone)));
//			viconAttState = matmult(rotPhoneToCam2, viconAttState);
//			viconTransState = matmult(rotPhoneToCam2, viconTransState);
//
//			attPrev.inject(attCur);
//			if(useViconState)
//				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
//			else
//				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
////				attChange.inject( matmult(attCur, transpose(attPrev)) );
//			attChange.inject( matmult(transpose(attCur), attPrev) );
//
//			/////////////////////////////////////////////////////
//			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
//			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);
////			cv::cvtColor(img, imgGray, CV_BGR2GRAY);
//
//			curKp.swap(prevKp);
//			curKp.clear();
////			featureDetector.detect(imgGray, curKp);
//			findPoints(imgGray, curKp, minDistance, qualityLevel);
//
//			numFeaturesAccum += curKp.size();
//
//			prevDescriptors = curDescriptors;
//			curDescriptors.release();
//			curDescriptors.data = NULL;
//			cv::OrbFeatureDetector extractor(500, 2.0f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
//			extractor.compute(imgGray, curKp, curDescriptors);
//
//			curDescriptors.convertTo(curDescriptors, CV_32F);
//
//			cv::FlannBasedMatcher matcher;
//			vector<cv::DMatch> matches;
//			matcher.match(prevDescriptors, curDescriptors, matches);
//
//			float minDist = 1000;
//			for(int i=0; i<matches.size(); i++)
//				minDist = min(matches[i].distance, minDist);
//
//			vector<cv::DMatch> goodMatches(matches);
////			for(int i=0; i<matches.size(); i++)
////				if(matches[i].distance < 5.0*minDist) goodMatches.push_back(matches[i]);
//			numMatchesAccum += goodMatches.size();
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
//				prevPoints[i] = prevKp[idx1].pt-center;
//				curPoints[i] = curKp[idx2].pt-center;
//			}
//
//			// MAP velocity and height
//			if(useViconState)
//			{
//				mv[0][0] = viconTransState[3][0];
//				mv[1][0] = viconTransState[4][0];
//				mv[2][0] = viconTransState[5][0];
//
//				mz = -viconTransState[2][0]; // in camera coords, z is flipped
//			}
//			else
//			{
//				mv[0][0] = transState[3][0];
//				mv[1][0] = transState[4][0];
//				mv[2][0] = transState[5][0];
//
//				mz = -transState[2][0]; // in camera coords, z is flipped
//			}
//
//			mz -= 0.1; // camera to vicon markers offset
//
//			double dt;
//			if(prevTime.getMS() > 0)
//				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
//			else
//				dt = 0;
//			Array2D<double> omega = logSO3(attChange, dt);
//
//			sz = sqrt( errCov[2][2]);
//			Sv[0][0] = errCov[3][3];
//			Sv[1][1] = errCov[4][4];
//			Sv[2][2] = errCov[5][5];
//
//			if(curPoints.size() > 0)
//			{
//				computeMAPEstimate(vel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
//
//				fs << curTime.getMS() << "\t" << 98 << "\t";
//				for(int i=0; i<vel.dim1(); i++)
//					fs << vel[i][0] << "\t";
//				fs << endl;
//				fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;
//			}
//
////			if(imgPrev.data != NULL)
////			{
////				cv::Mat imgMatches(img.rows,2*img.cols,img.type());
////				cv::Mat imgL = imgPrev.clone();
////				cv::Mat imgR = img.clone();
////				for(int i=0; i<prevPoints.size(); i++)
////					cv::circle(imgL, prevPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
////				for(int j=0; j<curPoints.size(); j++)
////					cv::circle(imgR, curPoints[j]+center, 4, cv::Scalar(0,0,255), -1);
////				imgL.copyTo(imgMatches(cv::Rect(0,0,img.cols,img.rows)));
////				imgR.copyTo(imgMatches(cv::Rect(img.cols,0,img.cols,img.rows)));
//////			drawMatches(imgPrev, prevKp, img, curKp, goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
////				imshow("chad",imgMatches);
////			}
////			keypress = cv::waitKey() % 256;
//
//			imgPrev = img;
//
////			img.release();
////			img.data = NULL;
//		}
//
//		fs.close();
//
//		cout << "Avg num ORB features: " << ((double)numFeaturesAccum)/imgList.size() << endl;
//		cout << "Avg num ORB matches: " << ((double)numMatchesAccum)/imgList.size() << endl;
//		cout << "ORB time: " << begin.getElapsedTimeMS()/1.0e3 << endl;
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
		double numMatchesAccum = 0;

		Array2D<double> attState(6,1), transState(6,1), viconAttState(6,1), viconTransState(6,1);
		Array2D<double> errCov1(9,1), errCov(6,6,0.0);

		errCov[0][0] = errCov[1][1] = errCov[2][2] = 0.01*0.01;
		errCov[3][3] = errCov[4][4] = errCov[5][5] = 0.1*0.1;

		Array2D<double> mv(3,1), omega(3,1), vel;
		double mz, dt, z;

		imgId = iter_imgList->first;
		while(imgIdList[imgIdx].first < imgId && imgIdx < imgIdList.size()) 
			imgIdx++;
		imgIdx--;
		Time curTime = imgIdList[imgIdx].second;
		viconTransState.inject(Data::interpolate(curTime, viconTransDataList));
		kfState.inject(viconTransState);
		Time lastMeasTime(curTime);

		// truncate the lists to the current start time
		shared_ptr<DataVector> attData, transData, errCovData, motorCmdsData, thrustDirData;
		while(attDataList.size() > 0 && attDataList.front()->timestamp < curTime)
		{ attData= attDataList.front(); attDataList.pop_front(); }
		while(transDataList.size() > 0 && transDataList.front()->timestamp < curTime)
		{ transData= transDataList.front(); transDataList.pop_front(); }
		while(errCovDataList.size() > 0 && errCovDataList.front()->timestamp < curTime)
		{ errCovData= errCovDataList.front(); errCovDataList.pop_front(); }
		while(motorCmdsDataList.size() > 0 && motorCmdsDataList.front()->timestamp < curTime)
		{ motorCmdsData= motorCmdsDataList.front(); motorCmdsDataList.pop_front(); }
		while(thrustDirDataList.size() > 0 && thrustDirDataList.front()->timestamp < curTime)
		{ thrustDirData= thrustDirDataList.front(); thrustDirDataList.pop_front(); }

		vector<pair<Array2D<double>, Array2D<double> > > priorDistList;

		Array2D<double> C;

double ts0, ts1, ts2, ts3, ts4, ts5, ts6, ts7;
ts0 = ts1 = ts2 = ts3 = ts4 = ts5 = ts6 = ts7 = 0;
Time t0;

		fstream fs("../mapResults.txt", fstream::out);
		for(int i=0; i<10; i++)
			fs << i << "\t";
		fs << endl;
		Time begin;
		while(keypress != (int)'q' && iter_imgList != imgList.end())
		{
t0.setTime();
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
			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;

			attState.inject(Data::interpolate(curTime, attDataList));
//			attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );

			// process events up to now
			list<shared_ptr<Data> > events;
			while(attDataList.size() > 0 && attDataList.front()->timestamp < curTime)
			{ events.push_back(attDataList.front()); attDataList.pop_front(); }
			while(transDataList.size() > 0 && transDataList.front()->timestamp < curTime)
			{ /*events.push_back(transDataList.front());*/ transDataList.pop_front(); }
			while(errCovDataList.size() > 0 && errCovDataList.front()->timestamp < curTime)
			{ /*events.push_back(errCovDataList.front());*/ errCovDataList.pop_front(); }
			while(motorCmdsDataList.size() > 0 && motorCmdsDataList.front()->timestamp < curTime)
			{ events.push_back(motorCmdsDataList.front()); motorCmdsDataList.pop_front(); }
			while(thrustDirDataList.size() > 0 && thrustDirDataList.front()->timestamp < curTime)
			{ events.push_back(thrustDirDataList.front()); thrustDirDataList.pop_front(); }
			events.sort( Data::timeSortPredicate );
			Time updateTime = prevTime;
			while(events.size() > 0)
			{
				updateTime = applyData(updateTime, events.front(), 
										kfState, kfErrCov, kfMeasCov, kfDynCov, 
										thrust, thrustDir, curGravDir, 
										mass);
				events.pop_front();
			}

			// remaining time update
//			thrustDir.inject( Data::interpolate(curTime, thrustDirDataList) );
			accel.inject(thrust/mass*thrustDir);
//			curGravDir.inject(-1.0*matmult(transpose(attCur), e3));
			accel += GRAVITY*curGravDir;
			double dtRem = Time::calcDiffNS(updateTime, curTime)/1.0e9;
			doTimeUpdateKF(accel, dtRem, kfState, kfErrCov, kfDynCov);

			// KF measurement update
			if(lastMeasTime.getElapsedTimeMS() > 1)
			{
				lastMeasTime.setTime(curTime);
//				viconAttState.inject(Data::interpolate(curTime, viconAttDataList));
				viconTransState.inject(Data::interpolate(curTime, viconTransDataList));
				doMeasUpdateKF_posOnly( submat(viconTransState,0,2,0,0), kfPosMeasCov, kfState, kfErrCov);
				doMeasUpdateKF_velOnly( submat(viconTransState,3,5,0,0), kfPosMeasCov, kfState, kfErrCov);
			}

			// Rotate to camera coords
			attState.inject(matmult(rotPhoneToCam2, attState));
			transState.inject(matmult(rotPhoneToCam2, kfState));
			errCov.inject(matmult(rotPhoneToCam2, matmult(kfErrCov, rotCamToPhone)));
//			viconAttState = matmult(rotPhoneToCam2, viconAttState);

			attPrev.inject(attCur);
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//			attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );

ts0 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);
//			cv::cvtColor(img, imgGray, CV_BGR2GRAY);

ts1 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			curPoints.swap(prevPoints);
			findPoints(imgGray, curPoints, minDistance, qualityLevel);
			numFeaturesAccum += curPoints.size();

ts2 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			/////////////////////////////////////////////////////
			//  Prior distributions
			for(int i=0; i<curPoints.size(); i++)
				curPoints[i] -= center;
			mv[0][0] = transState[3][0];
			mv[1][0] = transState[4][0];
			mv[2][0] = transState[5][0];

			mz = -transState[2][0]; // in camera coords, z is flipped
			mz -= 0.1; // camera to vicon markers offset

			omega.inject(logSO3(attChange, dt));
			priorDistList.clear();
			priorDistList = calcPriorDistributions(prevPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);

ts3 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			// Correspondence
			C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv);

ts4 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			for(int i=0; i<C.dim1()-1; i++)
				for(int j=0; j<C.dim2()-1; j++)
					numMatchesAccum += C[i][j];

			sz = sqrt( errCov[2][2]);
			Sv[0][0] = errCov[3][3];
			Sv[1][1] = errCov[4][4];
			Sv[2][2] = errCov[5][5];

			// MAP velocity and height
			if(prevPoints.size() > 0)
			{
				computeMAPEstimate(vel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
ts6 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();

				fs << curTime.getMS() << "\t" << 98 << "\t";
				for(int i=0; i<vel.dim1(); i++)
					fs << vel[i][0] << "\t";
				fs << endl;

				fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;

				fs << curTime.getMS() << "\t" << 100 << "\t";
				for(int i=0; i<kfState.dim1(); i++)
					fs << kfState[i][0] << "\t";
				fs << endl;
			}

ts7 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			//////////////////////////////////////////////////
			//Drawing
			// prior distributions
//			if(priorDistList.size() > 0)
//				maxSigma = 0;
//			cv::Mat overlay = img.clone();
//			for(int i=0; i<priorDistList.size(); i++)
//			{
//				Array2D<double> Sd = priorDistList[i].second;
//				JAMA::Eigenvalue<double> eig_Sd(Sd);
//				Array2D<double> V, D;
//				eig_Sd.getV(V);
//				eig_Sd.getD(D);
//
//				for(int j=0; j<D.dim1(); j++)
//					maxSigma = max(maxSigma, sqrt(D[j][j]));

//				cv::Point2f pt(priorDistList[i].first[0][0], priorDistList[i].first[1][0]);
//				double width = 2*sqrt(D[0][0]);
//				double height = 2*sqrt(D[1][1]);
//				double theta = atan2(V[1][0], V[0][0]);
//				cv::RotatedRect rect(pt+center, cv::Size(width, height), theta);
//				cv::ellipse(overlay, rect, cv::Scalar(255,0,0), -1);
//			}
//			double opacity = 0.3;
//			cv::addWeighted(overlay, opacity, img, 1-opacity, 0, img);

			// current points
//			for(int i=0; i<curPoints.size(); i++)
//				cv::circle(img, curPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
//			imshow("chad",img);
  
//			keypress = cv::waitKey() % 256;

			img.release();
			img.data = NULL;
		}

		fs.close();

		cout << "Avg num features: " << ((double)numFeaturesAccum)/imgList.size() << endl;
		cout << "Avg num matches: " << ((double)numMatchesAccum)/imgList.size() << endl;
		cout << "New total time: " << begin.getElapsedTimeMS()/1.0e3 << endl;
		cout << "\tts0: " << ts0 << endl;
		cout << "\tts1: " << ts1 << endl;
		cout << "\tts2: " << ts2 << endl;
		cout << "\tts3: " << ts3 << endl;
		cout << "\tts4: " << ts4 << endl;
		cout << "\tts5: " << ts5 << endl;
		cout << "\tts6: " << ts6 << endl;
		cout << "\tts7: " << ts7 << endl;

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
				list<shared_ptr<DataVector> > &errCovDataList,
				list<shared_ptr<DataVector> > &motorCmdsDataList,
				list<shared_ptr<DataVector> > &thrustDirDataList
				)
{
	imgIdList.clear();
	angleStateList.clear();
	transStateList.clear();
	errCovDataList.clear();
	motorCmdsDataList.clear();
	thrustDirDataList.clear();

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
						data->type = DATA_TYPE_ATTITUDE;
						angleStateList.push_back(data);

//						Array2D<double> attBias(3,1);
//						attBias[0][0] = 0.00; attBias[1][0] = 0.00; attBias[2][0] = 0.00;
//						double s1 = sin(attState[2][0]); double c1 = cos(attState[2][0]);
//						double s2 = sin(attState[1][0]-attBias[1][0]); double c2 = cos(attState[1][0]-attBias[1][0]);
//						double s3 = sin(attState[0][0]-attBias[0][0]); double c3 = cos(attState[0][0]-attBias[0][0]);
//						Array2D<double> r(3,1);
//						r[0][0] = s1*s3+c1*c3*s2;
//						r[1][0] = c3*s1*s2-c1*s3;
//						r[2][0] = c2*c3;
//						shared_ptr<DataVector> dataThrustDir(new DataVector() );
//						dataThrustDir->timestamp.setTimeMS(time);
//						dataThrustDir->data = r.copy();
//						dataThrustDir->type = DATA_TYPE_THRUST_DIR;
//						thrustDirDataList.push_back(dataThrustDir);
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
						data->type = DATA_TYPE_STATE_TRAN;
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
						data->type = DATA_TYPE_KF_ERR_COV;
						errCovDataList.push_back(data);

					}
					break;
				case LOG_ID_MOTOR_CMDS:
					{
						Array2D<double> motorCmds(4,1);
						for(int i=0; i<4; i++)
							ss >> motorCmds[i][0];
						shared_ptr<DataVector> data(new DataVector());
						data->timestamp.setTimeMS(time);
						data->data = motorCmds.copy();
						data->type = DATA_TYPE_MOTOR_CMDS;
						motorCmdsDataList.push_back(data);
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
			dataTrans->type = DATA_TYPE_VICON_POS;
	
			angleStateList.push_back(dataAngle);
			transStateList.push_back(dataTrans);
		}

		file.close();
	}
	else
		cout << "Couldn't find " << filename.c_str() << endl;
}

void findPoints(cv::Mat const &img, vector<cv::Point2f> &pts, float const &minDistance, float const &qualityLevel)
{
	vector<cv::KeyPoint> kp;
	findPoints(img, kp, minDistance, qualityLevel);
	cv::KeyPoint::convert(kp, pts);
}

void findPoints(cv::Mat const &img, vector<cv::KeyPoint> &pts, float const &minDistance, float const &qualityLevel)
{
	vector<cv::KeyPoint> tempKp1;
	cv::Ptr<cv::FastFeatureDetector> fastDetector(new cv::FastFeatureDetector(10));
	int maxKp = 1000;
	int gridRows = 3;
	int gridCols = 3;
	cv::GridAdaptedFeatureDetector detector(fastDetector, maxKp, gridRows, gridCols);
	detector.detect(img, tempKp1);
//	FAST(img, tempKp1, 10, true);
//	HarrisResponses(img, tempKp, 7, 0.04f);
	EigenValResponses(img, tempKp1, 5);

	float maxScore = -0xFFFFFFF;
	for(int i=0; i<tempKp1.size(); i++)
		maxScore = max(maxScore, tempKp1[i].response);

	vector<cv::KeyPoint> tempKp;
	tempKp.reserve(tempKp1.size());
	float threshold = qualityLevel*maxScore;
	for(int i=0; i<tempKp1.size(); i++)
		if(tempKp1[i].response > threshold)
			tempKp.push_back(tempKp1[i]);

//	cv::Mat eig;
//	eig.create(img.size(), CV_32F);
//	int MINEIGVAL = 0;
//	int HARRIS = 1;
//	cornerMinEigenVal(img, eig, 7, 3, MINEIGVAL);
//	for(int i=0; i<tempKp.size(); i++)
//	{
//		int x = tempKp[i].pt.x;
//		int y = tempKp[i].pt.y;
//		const float* row = (const float*)(eig.data + y*eig.step);
//		float eig = row[x];
//cout << "loc: (" << x << ", " << y << ")" << endl;
//cout << "point " << i << " eig 1: " << eig << endl;
//cout << "point " << i << " eig 2: " << tempKp[i].response << endl;
//cout << "----------------------------------" << endl;
//	}

	// group points into grid
	const int cellSize = minDistance+0.5;
	const int nGridX= (img.cols+cellSize-1)/cellSize;
	const int nGridY= (img.rows+cellSize-1)/cellSize;

	vector<vector<int> > grids(nGridX*nGridY); // stores index of keypoints in each grid
	vector<int> gridId(tempKp.size());
	for(int kpId=0; kpId < tempKp.size(); kpId++)
	{
		int xGrid = tempKp[kpId].pt.x / cellSize;
		int yGrid = tempKp[kpId].pt.y / cellSize;
		int gid = yGrid*nGridX+xGrid;
		grids[gid].push_back(kpId);
		gridId[kpId] = gid;
	}

	// Now pick the strongest points 
	// Right now, for reduced computation, it is using a minDistance x minDistance box for exclusion rather
	// than a minDistance radius circle
	vector<bool> isActive(gridId.size(), true);
	pts.clear();
	for(int i=0; i<gridId.size(); i++)
	{
		if(!isActive[i])
			continue;

		float curResponse = tempKp[i].response;
		cv::Point2f *curPt = &tempKp[i].pt;

		// turn off other points in this grid
		int gid = gridId[i];
		bool isStrongest = true;;
//		for(int j=0; j<grids[gid].size(); j++)
		int j=0;
		while(isStrongest && j < grids[gid].size() )
		{
			int kpId = grids[gid][j];
			if(kpId != i && curResponse >= tempKp[kpId].response)
				isActive[kpId] = false;
			else if(kpId != i && isActive[kpId])
				isStrongest = false;

			j++;
		}

		if(!isStrongest)
		{
			isActive[i] = false;
			continue;
		}

		// need to check neighbor grids too
		int xGrid = gid % nGridX;
		int yGrid = gid / nGridX;

		int neighborOffsetX[] = {-1, 0, 1, -1, 1, -1, 0, 1};
		int neighborOffsetY[] = {-1, -1, -1, 0, 0, 1, 1, 1};
//		for(int j=0; j<8; j++)
		j = 0;
		while(isStrongest && j < 8)
		{
			int nx = xGrid+neighborOffsetX[j];
			int ny = yGrid+neighborOffsetY[j];
			if(nx < 0 || nx > nGridX-1 || ny < 0 || ny > nGridY-1)
			{
				j++;
				continue;
			}

			int ngid = ny*nGridX+nx;
//			for(int k=0; k<grids[ngid].size(); k++)
			int k=0;
			while(isStrongest && k < grids[ngid].size() )
			{
				int kpId = grids[ngid][k];
				if(!isActive[kpId])
				{
					k++;
					continue;
				}

				cv::Point2f *pt = &tempKp[kpId].pt;
				if(abs(pt->x - curPt->x) <= minDistance && abs(pt->y - curPt->y) <= minDistance)
				{
					if(kpId != i && curResponse >= tempKp[kpId].response)
						isActive[kpId] = false;
					else if(kpId != i && isActive[kpId])
						isStrongest = false;
				}

				k++;
			}

			j++;
		}

		if(!isStrongest)
		{
			isActive[i] = false;
			continue;
		}

//		double minD = 0xFFFFFF;
//		double minJ = 0;
//		for(int j=0; j<tempKp.size(); j++)
//		{
//			if(j == i || !isActive[j])
//				continue;
//			cv::Point2f *pt = &tempKp[j].pt;
//			double dist = sqrt( pow(curPt->x - pt->x, 2) + pow(curPt->y - pt->y,2));
//			if(dist < minD)
//			{
//				minD = dist;
//				minJ =j;
//			}
//		}

		pts.push_back(tempKp[i]);
	}

	sort(pts.begin(), pts.end(), [&](cv::KeyPoint const &a, cv::KeyPoint const &b){return a.response > b.response;});
}

// taken from OpenCV ORB code
static void HarrisResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize, float harris_k)
{
    CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );

    size_t ptidx, ptsize = pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    int step = (int)(img.step/img.elemSize1());
    int r = blockSize/2;

    float scale = (1 << 2) * blockSize * 255.0f;
    scale = 1.0f / scale;
    float scale_sq_sq = scale * scale * scale * scale;

	cv::AutoBuffer<int> ofsbuf(blockSize*blockSize);
    int* ofs = ofsbuf;
    for( int i = 0; i < blockSize; i++ )
        for( int j = 0; j < blockSize; j++ )
            ofs[i*blockSize + j] = (int)(i*step + j);

    for( ptidx = 0; ptidx < ptsize; ptidx++ )
    {
        int x0 = cvRound(pts[ptidx].pt.x - r);
        int y0 = cvRound(pts[ptidx].pt.y - r);

        const uchar* ptr0 = ptr00 + y0*step + x0;
        int a = 0, b = 0, c = 0;

        for( int k = 0; k < blockSize*blockSize; k++ )
        {
            const uchar* ptr = ptr0 + ofs[k];
            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        }
        pts[ptidx].response = ((float)a * b - (float)c * c -
                               harris_k * ((float)a + b) * ((float)a + b))*scale_sq_sq;
    }
}

static void EigenValResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize)
{
    CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );

    size_t ptidx, ptsize = pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    int step = (int)(img.step/img.elemSize1());
    int r = blockSize/2;

    float scale = (1 << 2) * blockSize * 255.0f;
    scale = 1.0f / scale;
    float scale_sq = scale * scale;

	cv::AutoBuffer<int> ofsbuf(blockSize*blockSize);
    int* ofs = ofsbuf;
    for( int i = 0; i < blockSize; i++ )
        for( int j = 0; j < blockSize; j++ )
            ofs[i*blockSize + j] = (int)(i*step + j);

    for( ptidx = 0; ptidx < ptsize; ptidx++ )
    {
        int x0 = cvRound(pts[ptidx].pt.x - r);
        int y0 = cvRound(pts[ptidx].pt.y - r);

        const uchar* ptr0 = ptr00 + y0*step + x0;
        int a = 0, b = 0, c = 0;

        for( int k = 0; k < blockSize*blockSize; k++ )
        {
            const uchar* ptr = ptr0 + ofs[k];
			// this is sobel
//            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
//            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
			// this is Scharr
            int Ix = (ptr[1] - ptr[-1])*10 + (ptr[-step+1] - ptr[-step-1])*3 + (ptr[step+1] - ptr[step-1])*3;
            int Iy = (ptr[step] - ptr[-step])*10 + (ptr[step-1] - ptr[-step-1])*3 + (ptr[step+1] - ptr[-step+1])*3;
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        }

		double  m00 = 0.5*a*scale_sq;
		double  m01 =     c*scale_sq;
		double m11 = 0.5*b*scale_sq;
        pts[ptidx].response = m00 + m11 - sqrt( (m00-m11)*(m00-m11) + m01*m01);
		
//cout << pts[ptidx].response << endl;
		int chad = 0;
    }
}

void doTimeUpdateKF(Array2D<double> const &accel, double const &dt, Array2D<double> &state, Array2D<double> &errCov, Array2D<double> const &dynCov)
{
	for(int i=0; i<3; i++)
		state[i][0] += dt*state[i+3][0];
	for(int i=3; i<6; i++)
		state[i][0] += dt*accel[i-3][0];
	for(int i=0; i<3; i++)
	{
		errCov[i][i] += 2*errCov[i][i+3]*dt+errCov[i+3][i+3]*dt*dt+dynCov[i][i];
		errCov[i][i+3] += errCov[i+3][i+3]*dt+dynCov[i][i+3];
		errCov[i+3][i] += errCov[i+3][i+3]*dt+dynCov[i+3][i];
	}
	for(int i=3; i<6; i++)
		errCov[i][i] += dynCov[i][i];
}

void doMeasUpdateKF_posOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
{
	Array2D<double> gainKF(6,3,0.0);
	for(int i=0; i<3; i++)
	{
		gainKF[i][i] = errCov[i][i]/(errCov[i][i]+measCov[i][i]);
		gainKF[i+3][i] = errCov[i][i+3]/(errCov[i][i]+measCov[i][i]);
	}

	// \hat{x} = \hat{x} + K (meas - C \hat{x})
	Array2D<double> err2= meas-submat(state,0,2,0,0);
	for(int i=0; i<3; i++)
		state[i][0] += gainKF[i][i]*err2[i][0];
	for(int i=3; i<6; i++)
		state[i][0] += gainKF[i][i-3]*err2[i-3][0];

	// S = (I-KC) S
	for(int i=3; i<6; i++)
		errCov[i][i] -= gainKF[i][i-3]*errCov[i-3][i];
	for(int i=0; i<3; i++)
	{
		errCov[i][i] -= gainKF[i][i]*errCov[i][i];
		errCov[i][i+3] -= gainKF[i][i]*errCov[i][i+3];
		errCov[i+3][i] = errCov[i][i+3];
	}
}

void doMeasUpdateKF_velOnly(Array2D<double> const &meas, Array2D<double> const &measCov, Array2D<double> &state, Array2D<double> &errCov)
{
	if(norm2(meas) > 10)
		return; // screwy measuremnt

	Array2D<double> gainKF(6,3,0.0);
	for(int i=0; i<3; i++)
	{
		gainKF[i][i] = errCov[i][i+3]/(errCov[i+3][i+3]+measCov[i][i]);
		gainKF[i+3][i] = errCov[i+3][i+3]/(errCov[i+3][i+3]+measCov[i][i]);
	}

	// \hat{x} = \hat{x} + K (meas - C \hat{x})
	Array2D<double> err = meas-submat(state,3,5,0,0);
	for(int i=0; i<3; i++)
		state[i][0] += gainKF[i][i]*err[i][0];
	for(int i=3; i<6; i++)
		state[i][0] += gainKF[i][i-3]*err[i-3][0];

	// S = (I-KC) S
	for(int i=0; i<3; i++)
	{
		errCov[i][i] -= gainKF[i][i]*errCov[i][i+3];
		errCov[i][i+3] -= gainKF[i][i]*errCov[i+3][i+3];
		errCov[i+3][i] = errCov[i][i+3];
	}
	for(int i=3; i<6; i++)
		errCov[i][i] -= gainKF[i][i-3]*errCov[i][i];

}

Time applyData(Time const &curTime, shared_ptr<Data> const &data, 
				Array2D<double> &kfState, Array2D<double> &kfErrCov, Array2D<double> const &kfMeasCov, Array2D<double> const &kfDynCov,
				double &thrust, Array2D<double> &thrustDir, Array2D<double> &curGravDir, 
				double const &mass)
{
	Time dataTime = data->timestamp;
	Array2D<double> accel = thrust/mass*thrustDir;
	accel += GRAVITY*curGravDir;
	double dt = Time::calcDiffNS(curTime, dataTime)/1.0e9;
	doTimeUpdateKF(accel, dt, kfState, kfErrCov, kfDynCov);

	switch(data->type)
	{
		case DATA_TYPE_VICON_POS:
			doMeasUpdateKF_posOnly(static_pointer_cast<DataVector>(data)->data, submat(kfMeasCov,0,2,0,2), kfState, kfErrCov);
			break;
		case DATA_TYPE_VICON_VEL:
			doMeasUpdateKF_velOnly(static_pointer_cast<DataVector>(data)->data, submat(kfMeasCov,3,5,3,5), kfState, kfErrCov);
			break;
		case DATA_TYPE_OPTIC_FLOW_VEL:
			doMeasUpdateKF_velOnly(static_pointer_cast<DataVector>(data)->data, submat(kfMeasCov,3,5,3,5), kfState, kfErrCov);
			break;
		case DATA_TYPE_THRUST_DIR:
			thrustDir.inject( static_pointer_cast<DataVector>(data)->data );
			break;
		case DATA_TYPE_ATTITUDE:
			{
				Array2D<double> attState = static_pointer_cast<DataVector>(data)->data;
				Array2D<double> attSO3 = createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]);
				Array2D<double> attBias(3,1);
				attBias[0][0] = 0.004; attBias[1][0] = 0.018; attBias[2][0] = 0.00;
				double s1 = sin(attState[2][0]); double c1 = cos(attState[2][0]);
				double s2 = sin(attState[1][0]-attBias[1][0]); double c2 = cos(attState[1][0]-attBias[1][0]);
				double s3 = sin(attState[0][0]-attBias[0][0]); double c3 = cos(attState[0][0]-attBias[0][0]);
				thrustDir[0][0] = s1*s3+c1*c3*s2;
				thrustDir[1][0] = c3*s1*s2-c1*s3;
				thrustDir[2][0] = c2*c3;

				curGravDir[0][0] = -attSO3[2][0];
				curGravDir[1][0] = -attSO3[2][1];
				curGravDir[2][0] = -attSO3[2][2];
			}
			break;
		case DATA_TYPE_MOTOR_CMDS:
			{
				Array2D<double> cmds = static_pointer_cast<DataVector>(data)->data;
				thrust = 0;
				for(int i=0; i<4; i++)
					thrust += cmds[i][0];
				double forceGain = 0.002;
				thrust *= forceGain;
			}
			break;
		default:
			Log::alert(String()+"Observer_Translational::applyData() --> Unknown data type: "+data->type);
	}

	return dataTime;
}
