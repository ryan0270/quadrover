#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <memory>
#include <vector>
#include <list>
#include <random>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>

#include "toadlet/egg.h"

#include "../../Rover/cpp/Data.h"
#include "../../Rover/cpp/TNT/tnt.h"
#include "../../Rover/cpp/TNT/jama_cholesky.h"
#include "../../Rover/cpp/TNT/jama_lu.h"
#include "../../Rover/cpp/TNT/jama_eig.h"
#include "../../Rover/cpp/TNT_Utils.h"
#include "../../Rover/cpp/Time.h"
#include "../../Rover/cpp/constants.h"
#include "../../Rover/cpp/QuadLogger.h"

#include "mapFuncs.h"


typedef float DTYPE;

using namespace std;
using namespace ICSL::Rover;
using namespace ICSL::Quadrotor;
using namespace ICSL::Constants;
using namespace TNT;

template <class T>
void loadPhoneLog(String filename, 
				vector<pair<int, Time> > &imgIdList,
				list<shared_ptr<DataVector<T> > > &angleStateList,
				list<shared_ptr<DataVector<T> > > &transStateList,
				list<shared_ptr<DataVector<T> > > &errCovDataList,
				list<shared_ptr<DataVector<T> > > &motorCmdsDataList,
				list<shared_ptr<DataVector<T> > > &thrustDirDataList
				);
template <class T>
void loadPcLog(String filename,
				list<shared_ptr<DataVector<T> > > &angleStateList,
				list<shared_ptr<DataVector<T> > > &transStateList
		);
vector<string> tokenize(string str);

void findPoints(cv::Mat const &img, vector<cv::Point2f> &pts, float const &minDistance, float const &qualityLevel);
void findPoints(cv::Mat const &img, vector<cv::KeyPoint> &pts, float const &minDistance, float const &qualityLevel);

//static void HarrisResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize, float harris_k);
static void EigenValResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize);

void doTimeUpdateKF(Array2D<DTYPE> const &accel, float const &dt, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov, Array2D<DTYPE> const &dynCov);
void doMeasUpdateKF_posOnly(Array2D<DTYPE> const &meas, Array2D<DTYPE> const &measCov, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov);
void doMeasUpdateKF_heightOnly(float const &meas, float const &measCov, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov);
void doMeasUpdateKF_velOnly(Array2D<DTYPE> const &meas, Array2D<DTYPE> const &measCov, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov);
Time applyData(Time const &curTime, shared_ptr<Data> const &data, 
				Array2D<DTYPE> &kfState, Array2D<DTYPE> &kfErrCov, 
				Array2D<DTYPE> const &kfPosMeasCov, Array2D<DTYPE> const &kfVelMeasCov, Array2D<DTYPE> const &kfDynCov,
				float &thrust, Array2D<DTYPE> &thrustDir,
				float const &mass);

extern float ICSL::Rover::rs0, 
	   		  ICSL::Rover::rs1, 
			  ICSL::Rover::rs2,
			  ICSL::Rover::rs3,
			  ICSL::Rover::rs4,
			  ICSL::Rover::rs5;
int main(int argv, char* argc[])
{
	cout << "start chadding" << endl;

	rs0 = rs1 = rs2 = rs3 = rs4 = rs5 = 0;

	string imgDir = "../video_Jun5_3";
//	string imgDir = "../video_Jun10_1";

	vector<pair<int, Time> > imgIdList;
	list<shared_ptr<DataVector<DTYPE> > > attDataListOrig, transDataListOrig, errCovDataListOrig, motorCmdsDataListOrig, thrustDirDataListOrig;
	loadPhoneLog(String(imgDir.c_str())+String("/log.txt"), imgIdList, attDataListOrig, transDataListOrig, errCovDataListOrig, motorCmdsDataListOrig, thrustDirDataListOrig);
	if(imgIdList.size() == 0)
	{
		cout << "Failed loading data" << endl;
		return 0;
	}

	// preload all images
	list<pair<int, cv::Mat> > imgList;
	int imgId = 750;
//	int imgId = 5800;
	for(int i=0; i<900; i++)
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

cout << "final image: " << imgId << endl;

	list<shared_ptr<DataVector<float> > > viconAttDataList, viconTransDataList;
	loadPcLog(String(imgDir.c_str())+String("/pcData_fullState.txt"), viconAttDataList, viconTransDataList);

	Array2D<DTYPE> rotCamToPhone = matmult(createRotMat(2,(float)(-0.5*PI)), createRotMat(0,(float)PI));
	Array2D<DTYPE> rotCamToPhone2 = blkdiag(rotCamToPhone, rotCamToPhone);

	Array2D<DTYPE> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<DTYPE> rotPhoneToCam2 = transpose(rotCamToPhone2);

	int keypress = 0;
	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);

	cv::Mat img, imgGray, imgGrayRaw, imgPrev;
	vector<cv::Point2f> prevPoints, curPoints;
	Time prevTime, curTime;
	Array2D<DTYPE> attPrev, attCur, attChange;

	Array2D<DTYPE> Sv(3,3,0.0);
	float sz;
	Array2D<DTYPE> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = 2*pow(5,2);
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

	cv::Point2f center(317,249); // from camera calibration

	int focalLength = 524; // from camera calibration
//	int focalLength = 700;
	float camOffset = 0.05;

	tbb::task_scheduler_init init;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// KF vars
	Array2D<DTYPE> kfState(6,1,0.0), kfErrCov(6,6,0.0);
	Array2D<DTYPE> kfA = createIdentity(6.0f);
	Array2D<DTYPE> kfB(6,3,0.0);
	Array2D<DTYPE> kfDynCov(6,6,0.0);
	kfDynCov[0][0] = kfDynCov[1][1] = kfDynCov[2][2] = 0.01*0.01;
	kfDynCov[3][3] = kfDynCov[4][4] = 0.5*0.5;
	kfDynCov[5][5] = 0.5*0.5;
	Array2D<DTYPE> kfViconPosMeasCov(3,3,0.0), kfViconVelMeasCov(3,3,0.0);
	kfViconPosMeasCov[0][0] = kfViconPosMeasCov[1][1] = 0.01*0.01;
	kfViconPosMeasCov[2][2] = 0.005*0.005;
	kfViconVelMeasCov[0][0] = kfViconVelMeasCov[1][1] = kfViconVelMeasCov[2][2] = 0.1*0.1;
	Array2D<DTYPE> kfImageVelMeasCov(3,3,0.0);
	kfImageVelMeasCov[0][0] = kfImageVelMeasCov[1][1] = 0.2*0.2;
	kfImageVelMeasCov[2][2] =0.2*0.2;
	float kfImageHeightMeasCov = 0.1*0.1;
	float mass = 1.1; // kg
	float thrust = mass*GRAVITY;
	Array2D<DTYPE> accel(3,1,0.0);
	//	Array2D<DTYPE> attBias(3,1);
	//	attBias[0][0] = 0.004; attBias[1][0] = 0.018; attBias[2][0] = 0.00;
	Array2D<DTYPE> thrustDir(3,1,0.0);
	thrustDir[2][0] = 1;
	Array2D<DTYPE> motorCmds(4,1);

	Array2D<DTYPE> kfErrCovStart(6,6,0.0);
	kfErrCovStart[0][0] = kfErrCovStart[1][1] = kfErrCovStart[2][2] = 0.005*0.005;
	kfErrCovStart[3][3] = kfErrCovStart[4][4] = 0.1*0.1;
	kfErrCovStart[5][5] = 0.2*0.2;

	////////////////////////////////////////////////////////////////////////////////////////////////////

	float qualityLevel = 0.01;
	float minDistance = 10;

	bool useViconState = false;

	float viconUpdateDT = 100;

	//////////////////////////////////////////////////////////////////////////////////////////////////
	// Vicon measurment randomization
	Array2D<DTYPE> transStateNoiseStd(3,1,0.0);
	transStateNoiseStd[0][0] = transStateNoiseStd[1][0] = 0.01;
	transStateNoiseStd[2][0] = 0.01;
	srand(1);
	default_random_engine randGenerator;
	normal_distribution<DTYPE> stdGaussDist(0,1);

	vector<Array2D<DTYPE> > measNoise(imgId);
	for(int i=0; i<imgId; i++)
	{
		Array2D<DTYPE> noise(3,1);
		for(int j=0; j<3; j++)
			noise[j][0] = transStateNoiseStd[j][0]*stdGaussDist(randGenerator);
		measNoise[i] = noise.copy();
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////

	cout << "//////////////////////////////////////////////////" << endl;
	//////////////////////////////////////////////////////////////////
	// using ORB descriptors
	//////////////////////////////////////////////////////////////////
	{
		curTime.setTimeMS(0);
		attPrev = createIdentity(3.0f);
		attCur = createIdentity(3.0f);
		attChange = createIdentity(3.0f);
		int imgIdx = 0;
		list<pair<int, cv::Mat> >::iterator iter_imgList;
		iter_imgList = imgList.begin();
		long long numFeaturesAccum = 0;
		long long numMatchesAccum = 0;
		vector<cv::KeyPoint> prevKp, curKp;
		cv::Mat prevDescriptors, curDescriptors;
	
		Array2D<DTYPE> attState(6,1), transState(6,1), viconAttState(6,1), viconTransState(6,1);
		Array2D<DTYPE> errCov1(9,1), errCov(6,6,0.0);

		kfErrCov.inject(kfErrCovStart);

		Array2D<DTYPE> mv(3,1), omega(3,1), vel;
		float mz, dt, z;
	
		Array2D<DTYPE> velLS(3,1,0.0);

		imgId = iter_imgList->first;
		while(imgIdList[imgIdx].first < imgId && imgIdx < imgIdList.size()) 
			imgIdx++;
		imgIdx--;
		Time curTime = imgIdList[imgIdx].second;
		viconTransState.inject(Data::interpolate(curTime, viconTransDataList));
		kfState.inject(viconTransState);
		Time lastMeasTime(curTime);

		// Make local copy of data
		list<shared_ptr<DataVector<DTYPE> > > attDataList(attDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > transDataList(transDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > errCovDataList(errCovDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > motorCmdsDataList(motorCmdsDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > thrustDirDataList;(thrustDirDataListOrig);

		// truncate the lists to the current start time
		shared_ptr<DataVector<DTYPE> > attData, transData, errCovData, motorCmdsData, thrustDirData;
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
	
		float ts0, ts1, ts2, ts3, ts4, ts5, ts6, ts7;
		ts0 = ts1 = ts2 = ts3 = ts4 = ts5 = ts6 = ts7 = 0;
		Time t0;

		fstream fs("../orbResults.txt", fstream::out);
		for(int i=0; i<10; i++)
			fs << i << "\t";
		fs << endl;
		Time orbStartTime;
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
				cout << "imgIdx exceeded vector" << endl;;
				return 0;
			}
			prevTime.setTime(curTime);
			curTime = imgIdList[imgIdx].second;
			float dt;
			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;
	
			attState.inject(Data::interpolate(curTime, attDataList));

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
						kfState, kfErrCov, 
						kfViconPosMeasCov, kfViconVelMeasCov, kfDynCov, 
						thrust, thrustDir,
						mass);
				events.pop_front();
			}

			// Checking to make sure we haven't broken the error covariance
			JAMA::Eigenvalue<DTYPE> eig_kfErrCov(kfErrCov);
			Array2D<DTYPE> eigs;
			eig_kfErrCov.getD(eigs);
			float minEig = eigs[0][0];
			for(int i=1; i<eigs.dim1(); i++)
				minEig = min(minEig, (float)eigs[i][i]);
			while(minEig <= 0)
			{
				cout << "crap! kfErrCov is not definite" << endl;;

				kfErrCov = kfErrCov-(1.1*minEig-1e-6)*createIdentity(6.0f);

				JAMA::Eigenvalue<DTYPE> eig_kfErrCov(kfErrCov);
				eig_kfErrCov.getD(eigs);
				minEig = eigs[0][0];
				for(int i=1; i<eigs.dim1(); i++)
					minEig = min(minEig, (float)eigs[i][i]);
			}

			// remaining time update
//			thrustDir.inject( Data::interpolate(curTime, thrustDirDataList) );
			accel.inject(thrust/mass*thrustDir);
			accel[2][0] -= GRAVITY;
			float dtRem = Time::calcDiffNS(updateTime, curTime)/1.0e9;
			doTimeUpdateKF(accel, dtRem, kfState, kfErrCov, kfDynCov);

			// KF measurement update
			viconTransState.inject(Data::interpolate(curTime, viconTransDataList));
			if( Time::calcDiffMS(lastMeasTime, curTime) > viconUpdateDT)
			{
				// add noise
				Array2D<DTYPE> meas = submat(viconTransState,0,2,0,0);
				meas += measNoise[imgId];

				float dt = Time::calcDiffNS(lastMeasTime, curTime)/1.0e9;
				lastMeasTime.setTime(curTime);
				doMeasUpdateKF_posOnly( meas, kfViconPosMeasCov, kfState, kfErrCov);
			}

			attPrev.inject(attCur);
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//			attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );
			omega.inject(logSO3(attChange, dt));
	
			// Rotate to camera coords
			attState = matmult(rotPhoneToCam2, attState);
			transState.inject(matmult(rotPhoneToCam2, kfState));
			errCov.inject(matmult(rotPhoneToCam2, matmult(kfErrCov, rotCamToPhone2)));
//			viconAttState = matmult(rotPhoneToCam2, viconAttState);
			omega.inject(matmult(rotPhoneToCam, omega));
	
ts0 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);
//			cv::cvtColor(img, imgGray, CV_BGR2GRAY);
	
ts1 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			curKp.swap(prevKp);
			findPoints(imgGray, curKp, minDistance, qualityLevel);
			numFeaturesAccum += curKp.size();
	
ts2 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			prevDescriptors = curDescriptors;
			curDescriptors.release();
			curDescriptors.data = NULL;
			cv::OrbFeatureDetector extractor(1000, 2.0f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
			extractor.compute(imgGray, curKp, curDescriptors);
	
//			curDescriptors.convertTo(curDescriptors, CV_32F);
			cv::BFMatcher matcher(cv::NORM_L2, true);
//			cv::BFMatcher matcher(cv::NORM_HAMMING2, true); // L2 norm seems to be doing best both for quality and speed
			vector<cv::DMatch> matches;
			if(prevDescriptors.rows> 0 && curDescriptors.rows > 0)
				matcher.match(prevDescriptors, curDescriptors, matches);
	
ts3 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
//			float minDist = 10000;
//			for(int i=0; i<matches.size(); i++)
//				minDist = min(matches[i].distance, minDist);
//			// filter out low quality matches
			vector<cv::DMatch> goodMatches;
			matches.swap(goodMatches);
	
			if(goodMatches.size() > 0)
			{
				// Now filter on point location distance
				vector<cv::DMatch> tempMatches;
				goodMatches.swap(tempMatches);
	
				vector<DTYPE> distances(tempMatches.size());
				for(int i=0; i<tempMatches.size(); i++)
				{
					cv::Point2f p1 = prevKp[tempMatches[i].queryIdx].pt;
					cv::Point2f p2 = curKp[tempMatches[i].trainIdx].pt;
					distances[i] = sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2));
				}
				int medIndex = distances.size()/2;
				vector<DTYPE> distancesCopy(distances);
				// this finds the median
				nth_element(distancesCopy.begin(), distancesCopy.begin()+medIndex, distancesCopy.end(),
						[&](float const &a, float const &b){return a < b;});
				float medDist = distancesCopy[medIndex];
				float maxDelta = 40;
				float lowEnd = min(0.5f*medDist,medDist-maxDelta);
				float highEnd = max(1.5f*medDist,medDist+maxDelta);
				for(int i=0; i<tempMatches.size(); i++)
					if( lowEnd < distances[i] && distances[i] < highEnd)
						goodMatches.push_back(tempMatches[i]);
				numMatchesAccum += goodMatches.size();
			}
	
			int N1, N2;
			N1 = N2 = goodMatches.size();
			Array2D<DTYPE> C(N1+1, N2+1, 0.0); // the extra row and column will stay at zero
			vector<cv::Point2f> prevPoints(N1), curPoints(N2);
			for(int i=0; i<N1; i++)
			{
				C[i][i] = 1;
	
				int idx1 = goodMatches[i].queryIdx;
				int idx2 = goodMatches[i].trainIdx;
				prevPoints[i] = prevKp[idx1].pt-center;
				curPoints[i] = curKp[idx2].pt-center;
			}
	
ts4 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			if(N1 > 0)
			{
				// LS Estimate
				Array2D<DTYPE> deltaStack, LvStack;
				float f = focalLength;
				float fInv = 1.0/f;
				for(int i=0; i<N1; i++)
				{
					float x = prevPoints[i].x;
					float y = prevPoints[i].y;
	
					Array2D<DTYPE> Lv(2,3), Lw(2,3);
					Lv[0][0] = -f; Lv[0][1] = 0;  Lv[0][2] = x;
					Lv[1][0] = 0;  Lv[1][1] = -f; Lv[1][2] = y;
					Lw[0][0] = fInv*x*y; 		Lw[0][1] = -(f+fInv*x*x); 	Lw[0][2] = y;
					Lw[1][0] = f+fInv*y*y;		Lw[1][1] = -fInv*x*y;		Lw[1][2] = -x;
	
					Array2D<DTYPE> q1(2,1), q2(2,1);
					q1[0][0] = prevPoints[i].x; q1[1][0] = prevPoints[i].y;
					q2[0][0] = curPoints[i].x;  q2[1][0] = curPoints[i].y;
					Array2D<DTYPE> delta = q2-q1-dt*matmult(Lw, omega);
					if(deltaStack.dim1() == 0)
					{
						deltaStack = delta;
						LvStack = Lv;
					}
					else
					{
						deltaStack = stackVertical(deltaStack, delta);
						LvStack = stackVertical(LvStack, Lv);
					}
				}
				float z = viconTransState[2][0]-camOffset;
//				float z = -transState[2][0]-camOffset;
				Array2D<DTYPE> temp1 = dt/z*matmult(transpose(LvStack), LvStack);
				JAMA::LU<DTYPE> lu_temp1(temp1);
				Array2D<DTYPE> temp2 = matmult(transpose(LvStack), deltaStack);
				if(LvStack.dim1() > 2)
					velLS = lu_temp1.solve(temp2);
				else
					cout << "bad chad" << endl;
			}
	
ts5 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			// MAP velocity and height
			mv[0][0] = transState[3][0];
			mv[1][0] = transState[4][0];
			mv[2][0] = transState[5][0];

			mz = -transState[2][0]; // in camera coords, z is flipped
			mz -= camOffset; // camera to vicon markers offset
	
			sz = sqrt( errCov[2][2]);
			Sv[0][0] = errCov[3][3];
			Sv[1][1] = errCov[4][4];
			Sv[2][2] = errCov[5][5];
	
			if(N1 > 5)
			{
				Array2D<DTYPE> covVel;
				int maxPointCnt = 15;
				computeMAPEstimate(vel, covVel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega, maxPointCnt);
ts6 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
				z += camOffset;

				// Apply measurement to the state estimate
				vel = matmult(rotCamToPhone, vel);
//				covVel = matmult(rotCamToPhone, matmult(covVel, rotPhoneToCam));
				doMeasUpdateKF_velOnly(vel, kfImageVelMeasCov, kfState, kfErrCov);
				doMeasUpdateKF_heightOnly(z, kfImageHeightMeasCov, kfState, kfErrCov);
			}

			// save data
			fs << curTime.getMS() << "\t" << 98 << "\t";
			for(int i=0; i<vel.dim1(); i++)
				fs << vel[i][0] << "\t";
			fs << endl;

			fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;

			fs << curTime.getMS() << "\t" << 100 << "\t";
			for(int i=0; i<kfState.dim1(); i++)
				fs << kfState[i][0] << "\t";
			fs << endl;

			fs << curTime.getMS() << "\t" << 101 << "\t";
			for(int i=0; i<velLS.dim1(); i++)
				fs << velLS[i][0] << "\t";
			fs << endl;
	
//			if(imgPrev.data != NULL)
//			{
//				cv::Mat imgMatches(img.rows,2*img.cols,img.type());
////				cv::Mat imgL = imgPrev.clone();
////				cv::Mat imgR = img.clone();
////				for(int i=0; i<prevPoints.size(); i++)
////					cv::circle(imgL, prevPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
////				for(int j=0; j<curPoints.size(); j++)
////					cv::circle(imgR, curPoints[j]+center, 4, cv::Scalar(0,0,255), -1);
////				imgL.copyTo(imgMatches(cv::Rect(0,0,img.cols,img.rows)));
////				imgR.copyTo(imgMatches(cv::Rect(img.cols,0,img.cols,img.rows)));
//				drawMatches(imgPrev, prevKp, img, curKp, goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//				imshow("chad",imgMatches);
//			}
//			keypress = cv::waitKey() % 256;
	
			imgPrev = img;
  
			img.release();
			img.data = NULL;
		}
	
		fs.close();
	
		cout << "Avg num ORB features: " << ((double)numFeaturesAccum)/imgList.size() << endl;
		cout << "Avg num ORB matches: " << ((double)numMatchesAccum)/(imgList.size()-1) << endl;
		cout << "ORB time: " << orbStartTime.getElapsedTimeMS()/1.0e3 << endl;
		cout << "\tts0: " << ts0 << endl;
		cout << "\tts1: " << ts1 << endl;
		cout << "\tts2: " << ts2 << endl;
		cout << "\tts3: " << ts3 << endl;
		cout << "\tts4: " << ts4 << endl;
		cout << "\tts5: " << ts5 << endl;
		cout << "\tts6: " << ts6 << endl;
		cout << "\tts7: " << ts7 << endl;

		double avgCalcTime = orbStartTime.getElapsedTimeNS()/1.0e9 - ts0 - ts5;
		avgCalcTime /= imgList.size()-1;
		cout << "ORB avg calc time: " << avgCalcTime << endl;
	}

	cout << "//////////////////////////////////////////////////" << endl;
	//////////////////////////////////////////////////////////////////
	// Once for new algo
	//////////////////////////////////////////////////////////////////
	{
		curTime.setTimeMS(0);
		attPrev = createIdentity(3.0f);
		attCur = createIdentity(3.0f);
		attChange = createIdentity(3.0f);
		int imgIdx = 0;
		list<pair<int, cv::Mat> >::iterator iter_imgList;
		iter_imgList = imgList.begin();
		long long numFeaturesAccum = 0;
		float numMatchesAccum = 0;

		Array2D<DTYPE> attState(6,1), transState(6,1), viconAttState(6,1), viconTransState(6,1);
		Array2D<DTYPE> errCov1(9,1), errCov(6,6,0.0);

		kfErrCov.inject(kfErrCovStart);

		Array2D<DTYPE> mv(3,1), omega(3,1), vel;
		float mz, dt, z;

		imgId = iter_imgList->first;
		while(imgIdList[imgIdx].first < imgId && imgIdx < imgIdList.size()) 
			imgIdx++;
		imgIdx--;
		Time curTime = imgIdList[imgIdx].second;
		viconTransState.inject(Data::interpolate(curTime, viconTransDataList));
		kfState.inject(viconTransState);
		Time lastMeasTime(curTime);

		// Make local copy of data
		list<shared_ptr<DataVector<DTYPE> > > attDataList(attDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > transDataList(transDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > errCovDataList(errCovDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > motorCmdsDataList(motorCmdsDataListOrig);
		list<shared_ptr<DataVector<DTYPE> > > thrustDirDataList;(thrustDirDataListOrig);

		// truncate the lists to the current start time
		shared_ptr<DataVector<DTYPE> > attData, transData, errCovData, motorCmdsData, thrustDirData;
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

		vector<pair<Array2D<DTYPE>, Array2D<DTYPE> > > priorDistList;

		Array2D<DTYPE> C;

		float ts0, ts1, ts2, ts3, ts4, ts5, ts6, ts7;
		ts0 = ts1 = ts2 = ts3 = ts4 = ts5 = ts6 = ts7 = 0;
		Time t0;

		Array2D<DTYPE> lastViconPos;
		fstream fs("../mapResults.txt", fstream::out);
		for(int i=0; i<10; i++)
			fs << i << "\t";
		fs << endl;
		Time mapStartTime;
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
						kfState, kfErrCov, 
						kfViconPosMeasCov, kfViconVelMeasCov, kfDynCov, 
						thrust, thrustDir,
						mass);
				events.pop_front();
			}

			// Checking to make sure we haven't broken the error covariance
			JAMA::Eigenvalue<DTYPE> eig_kfErrCov(kfErrCov);
			Array2D<DTYPE> eigs;
			eig_kfErrCov.getD(eigs);
			float minEig = eigs[0][0];
			for(int i=1; i<eigs.dim1(); i++)
				minEig = min(minEig, (float)eigs[i][i]);
			while(minEig <= 0)
			{
				cout << "crap! kfErrCov is not definite" << endl;
				printArray("kfErrCov:\n",kfErrCov);

				kfErrCov = kfErrCov-(1.1*minEig-1e-6)*createIdentity(6.0f);

				JAMA::Eigenvalue<DTYPE> eig_kfErrCov(kfErrCov);
				eig_kfErrCov.getD(eigs);
				minEig = eigs[0][0];
				for(int i=1; i<eigs.dim1(); i++)
					minEig = min(minEig, (float)eigs[i][i]);
			}

			// remaining time update
//			thrustDir.inject( Data::interpolate(curTime, thrustDirDataList) );
			accel.inject(thrust/mass*thrustDir);
			accel[2][0] -= GRAVITY;
			float dtRem = Time::calcDiffNS(updateTime, curTime)/1.0e9;
			doTimeUpdateKF(accel, dtRem, kfState, kfErrCov, kfDynCov);

			// KF measurement update
			if( Time::calcDiffMS(lastMeasTime, curTime) > viconUpdateDT)
			{
				viconTransState.inject(Data::interpolate(curTime, viconTransDataList));
				// add noise
				Array2D<DTYPE> meas = submat(viconTransState,0,2,0,0);
				meas += measNoise[imgId];

				float dt = Time::calcDiffNS(lastMeasTime, curTime)/1.0e9;
				lastMeasTime.setTime(curTime);
				doMeasUpdateKF_posOnly( meas, kfViconPosMeasCov, kfState, kfErrCov);
			}

			attPrev.inject(attCur);
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//			attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );
			omega.inject(logSO3(attChange, dt));

			// Rotate to camera coords
			attState.inject(matmult(rotPhoneToCam2, attState));
			transState.inject(matmult(rotPhoneToCam2, kfState));
			errCov.inject(matmult(rotPhoneToCam2, matmult(kfErrCov, rotCamToPhone2)));
//			viconAttState = matmult(rotPhoneToCam2, viconAttState);
			omega.inject(matmult(rotPhoneToCam, omega));

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
			mz -= camOffset; // camera to vicon markers offset

			sz = sqrt( errCov[2][2]);
			Sv[0][0] = errCov[3][3];
			Sv[1][1] = errCov[4][4];
			Sv[2][2] = errCov[5][5];

			priorDistList.clear();
			priorDistList = calcPriorDistributions(prevPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);

ts3 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			// Correspondence
			C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv);

ts4 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			for(int i=0; i<C.dim1()-1; i++)
				for(int j=0; j<C.dim2()-1; j++)
					numMatchesAccum += C[i][j];
			
			// MAP velocity and height
			if(prevPoints.size() > 0)
			{
				Array2D<DTYPE> covVel;
				computeMAPEstimate(vel, covVel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
ts6 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
				z += camOffset;

				// Apply measurement to the state estimate
				vel = matmult(rotCamToPhone, vel);
				covVel = matmult(rotCamToPhone, matmult(covVel, rotPhoneToCam));
				doMeasUpdateKF_velOnly(vel, kfImageVelMeasCov, kfState, kfErrCov);
				doMeasUpdateKF_heightOnly(z, kfImageHeightMeasCov, kfState, kfErrCov);
			}

			// save data
			fs << curTime.getMS() << "\t" << 98 << "\t";
			for(int i=0; i<vel.dim1(); i++)
				fs << vel[i][0] << "\t";
			fs << endl;

			fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;

			fs << curTime.getMS() << "\t" << 100 << "\t";
			for(int i=0; i<kfState.dim1(); i++)
				fs << kfState[i][0] << "\t";
			fs << endl;

ts7 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			//////////////////////////////////////////////////
			//Drawing
			// prior distributions
//			cv::Mat overlay = img.clone();
//			for(int i=0; i<priorDistList.size(); i++)
//			{
//				Array2D<double> Sd = priorDistList[i].second;
//				JAMA::Eigenvalue<double> eig_Sd(Sd);
//				Array2D<double> V, D;
//				eig_Sd.getV(V);
//				eig_Sd.getD(D);
//
//				cv::Point2f pt(priorDistList[i].first[0][0], priorDistList[i].first[1][0]);
//				cv::circle(img, pt+center, 4, cv::Scalar(255,0,0), -1);
////				double width = 2*sqrt(D[0][0]);
////				double height = 2*sqrt(D[1][1]);
////				double theta = atan2(V[1][0], V[0][0]);
////				cv::RotatedRect rect(pt+center, cv::Size(width, height), theta);
////				cv::ellipse(overlay, rect, cv::Scalar(255,0,0), -1);
//			}
//			double opacity = 0.3;
//			cv::addWeighted(overlay, opacity, img, 1-opacity, 0, img);
//
//			// current points
//			for(int i=0; i<curPoints.size(); i++)
//				cv::circle(img, curPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
//			cv::Mat dblImg(img.rows, 2*img.cols, img.type());
//			if(imgPrev.data != NULL)
//				imgPrev.copyTo(dblImg(cv::Rect(0,0,img.cols,img.rows)));
//			img.copyTo(dblImg(cv::Rect(img.cols,0,img.cols,img.rows)));
//			cout << "imgID: " << imgId << endl;
//			imshow("chad",dblImg);
//
//			keypress = cv::waitKey() % 256;
//
			imgPrev.release();
			imgPrev = img;
		}

		fs.close();

		cout << "Avg num features: " << ((double)numFeaturesAccum)/imgList.size() << endl;
		cout << "Avg num matches: " << ((double)numMatchesAccum)/(imgList.size()-1) << endl;
		cout << "New total time: " << mapStartTime.getElapsedTimeMS()/1.0e3 << endl;
		cout << "\tts0: " << ts0 << endl;
		cout << "\tts1: " << ts1 << endl;
		cout << "\tts2: " << ts2 << endl;
		cout << "\tts3: " << ts3 << endl;
		cout << "\tts4: " << ts4 << endl;
		cout << "\tts5: " << ts5 << endl;
		cout << "\tts6: " << ts6 << endl;
		cout << "\tts7: " << ts7 << endl;
//		cout << "--------------------" << endl;
//		cout << "\trs0: " << rs0 << endl;
//		cout << "\trs1: " << rs1 << endl;
//		cout << "\trs2: " << rs2 << endl;
//		cout << "\trs3: " << rs3 << endl;
//		cout << "\trs4: " << rs4 << endl;
//		cout << "\trs5: " << rs5 << endl;

		float avgCalcTime = mapStartTime.getElapsedTimeNS()/1.0e9 - ts0;
		avgCalcTime /= imgList.size()-1;
		cout << "NEW avg calc time: " << avgCalcTime << endl;
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

template <class T>
void loadPhoneLog(String filename, 
				vector<pair<int, Time> > &imgIdList,
				list<shared_ptr<DataVector<T> > > &angleStateList,
				list<shared_ptr<DataVector<T> > > &transStateList,
				list<shared_ptr<DataVector<T> > > &errCovDataList,
				list<shared_ptr<DataVector<T> > > &motorCmdsDataList,
				list<shared_ptr<DataVector<T> > > &thrustDirDataList
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
			float time;
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
						Array2D<DTYPE> attState(6,1);
						for(int i=0; i<6; i++)
							ss >> attState[i][0];

						shared_ptr<DataVector<T> > data(new DataVector<T> ());
						data->timestamp.setTimeMS(time);
						data->data = attState.copy();
						data->type = DATA_TYPE_ATTITUDE;
						angleStateList.push_back(data);
					}
					break;
				case LOG_ID_CUR_TRANS_STATE:
					{
						Array2D<DTYPE> transState(6,1);
						for(int i=0; i<6; i++)
							ss >> transState[i][0];

						shared_ptr<DataVector<T> > data(new DataVector<T> ());
						data->timestamp.setTimeMS(time);
						data->data = transState.copy();
						data->type = DATA_TYPE_STATE_TRAN;
						transStateList.push_back(data);
					}
					break;
				case LOG_ID_KALMAN_ERR_COV:
					{
						Array2D<DTYPE> errCov(9,1);
						for(int i=0; i<9; i++)
							ss >> errCov[i][0];

						shared_ptr<DataVector<T> > data(new DataVector<T> ());
						data->timestamp.setTimeMS(time);
						data->data = errCov.copy();
						data->type = DATA_TYPE_KF_ERR_COV;
						errCovDataList.push_back(data);

					}
					break;
				case LOG_ID_MOTOR_CMDS:
					{
						Array2D<DTYPE> motorCmds(4,1);
						for(int i=0; i<4; i++)
							ss >> motorCmds[i][0];
						shared_ptr<DataVector<T> > data(new DataVector<T> ());
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

template <class T>
void loadPcLog(String filename, 
				list<shared_ptr<DataVector<T> > > &angleStateList,
				list<shared_ptr<DataVector<T> > > &transStateList
				)
{
	angleStateList.clear();
	transStateList.clear();

	Array2D<DTYPE> rotViconToQuad = createRotMat(0, (float)PI);
	Array2D<DTYPE> rotQuadToPhone = matmult(createRotMat(2,(float)(-0.25*PI)), createRotMat(0,(float)PI));
	Array2D<DTYPE> rotCamToPhone = matmult(createRotMat(2,(float)(-0.5*PI)), createRotMat(0,(float)PI));
	Array2D<DTYPE> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<DTYPE> rotViconToPhone = matmult(rotQuadToPhone, rotViconToQuad);

	Array2D<DTYPE> rotQuadToPhone2 = blkdiag(rotQuadToPhone, rotQuadToPhone);
	Array2D<DTYPE> rotViconToPhone2 = blkdiag(rotViconToPhone, rotViconToPhone);

//	Array2D<DTYPE> transStateNoiseStd(6,1,0.0);
//	transStateNoiseStd[0][0] = transStateNoiseStd[1][0] = 0.01;
//	transStateNoiseStd[2][0] = 0.005;
//	default_random_engine randGenerator;
//	normal_distribution<DTYPE> stdGaussDist(0,1);

	string line;
	ifstream file(filename.c_str());
	if(file.is_open())
	{
		vector<string> tokens;

		while(file.good())
		{
			getline(file, line);
			stringstream ss(line);
			float time;
			int type;
			ss >> time >> type;
time -= 0;

			// This file is assumed to be all state data
			Array2D<DTYPE> angleState(6,1), transState(6,1);
			for(int i=0; i<6; i++)
				ss >> angleState[i][0];
			for(int i=0; i<6; i++)
				ss >> transState[i][0];

			// add noise
//			for(int i=0; i<6; i++)
//				transState[i][0] += transStateNoiseStd[i][0]*stdGaussDist(randGenerator);
			
			angleState = matmult(rotQuadToPhone2, angleState);
			transState = matmult(rotViconToPhone2, transState);

			shared_ptr<DataVector<T> > dataAngle(new DataVector<T> ()), dataTrans(new DataVector<T> ());;
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
	int fastFeatureThreshold = 25;
	cv::Ptr<cv::FastFeatureDetector> fastDetector(new cv::FastFeatureDetector(fastFeatureThreshold));
	int maxKp = 1000;
	int gridRows = 3;
	int gridCols = 3;
	cv::GridAdaptedFeatureDetector detector(fastDetector, maxKp, gridRows, gridCols);
	detector.detect(img, tempKp1);
//	FAST(img, tempKp1, fastFeatureThreshold, true);
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
	float curResponse;
	int neighborOffsetX[] = {-1, 0, 1, -1, 1, -1, 0, 1};
	int neighborOffsetY[] = {-1, -1, -1, 0, 0, 1, 1, 1};
	for(int i=0; i<gridId.size(); i++)
	{
		if(!isActive[i])
			continue;

		curResponse = tempKp[i].response;
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

//		float minD = 0xFFFFFF;
//		float minJ = 0;
//		for(int j=0; j<tempKp.size(); j++)
//		{
//			if(j == i || !isActive[j])
//				continue;
//			cv::Point2f *pt = &tempKp[j].pt;
//			float dist = sqrt( pow(curPt->x - pt->x, 2) + pow(curPt->y - pt->y,2));
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
//static void HarrisResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize, float harris_k)
//{
//    CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );
//
//    size_t ptidx, ptsize = pts.size();
//
//    const uchar* ptr00 = img.ptr<uchar>();
//    int step = (int)(img.step/img.elemSize1());
//    int r = blockSize/2;
//
//    float scale = (1 << 2) * blockSize * 255.0f;
//    scale = 1.0f / scale;
//    float scale_sq_sq = scale * scale * scale * scale;
//
//	cv::AutoBuffer<int> ofsbuf(blockSize*blockSize);
//    int* ofs = ofsbuf;
//    for( int i = 0; i < blockSize; i++ )
//        for( int j = 0; j < blockSize; j++ )
//            ofs[i*blockSize + j] = (int)(i*step + j);
//
//    for( ptidx = 0; ptidx < ptsize; ptidx++ )
//    {
//        int x0 = cvRound(pts[ptidx].pt.x - r);
//        int y0 = cvRound(pts[ptidx].pt.y - r);
//
//        const uchar* ptr0 = ptr00 + y0*step + x0;
//        int a = 0, b = 0, c = 0;
//
//        for( int k = 0; k < blockSize*blockSize; k++ )
//        {
//            const uchar* ptr = ptr0 + ofs[k];
//            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
//            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
//            a += Ix*Ix;
//            b += Iy*Iy;
//            c += Ix*Iy;
//        }
//        pts[ptidx].response = ((float)a * b - (float)c * c -
//                               harris_k * ((float)a + b) * ((float)a + b))*scale_sq_sq;
//    }
//}

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

	float m00, m01, m11;
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

		m00 = 0.5*a*scale_sq;
		m01 =     c*scale_sq;
		m11 = 0.5*b*scale_sq;
        pts[ptidx].response = m00 + m11 - sqrt( (m00-m11)*(m00-m11) + m01*m01);
		
//cout << pts[ptidx].response << endl;
		int chad = 0;
    }
}

void doTimeUpdateKF(Array2D<DTYPE> const &accel, float const &dt, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov, Array2D<DTYPE> const &dynCov)
{
	for(int i=0; i<3; i++)
		state[i][0] += dt*state[i+3][0];
	for(int i=3; i<6; i++)
		state[i][0] += dt*accel[i-3][0];

	// This is based on the discrete time formulation
//	for(int i=0; i<3; i++)
//	{
//		errCov[i][i] += 2*errCov[i][i+3]*dt+errCov[i+3][i+3]*dt*dt+dynCov[i][i];
//		errCov[i][i+3] += errCov[i+3][i+3]*dt+dynCov[i][i+3];
//		errCov[i+3][i] += errCov[i+3][i+3]*dt+dynCov[i+3][i];
//	}
//	for(int i=3; i<6; i++)
//		errCov[i][i] += dynCov[i][i];

	// This is based on the continuous time formulation (with Euler time integration)
	Array2D<DTYPE> A(6,6,0.0), A_T(6,6,0.0);
	A[0][3] = A[1][4] = A[2][5] = 1;
	A_T[3][0] = A_T[4][1] = A_T[5][2] = 1; 
	Array2D<DTYPE> Winv(6,6,0.0);
	Array2D<DTYPE> Pdot(6,6);
	Pdot = matmult(A, errCov) + matmult(errCov, A_T) + dynCov;

	errCov = errCov+dt*Pdot;
}

void doMeasUpdateKF_posOnly(Array2D<DTYPE> const &meas, Array2D<DTYPE> const &measCov, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov)
{
	Array2D<DTYPE> gainKF(6,3,0.0);
	for(int i=0; i<3; i++)
	{
		gainKF[i][i] = errCov[i][i]/(errCov[i][i]+measCov[i][i]);
		gainKF[i+3][i] = errCov[i][i+3]/(errCov[i][i]+measCov[i][i]);
	}

	// \hat{x} = \hat{x} + K (meas - C \hat{x})
	Array2D<DTYPE> err2= meas-submat(state,0,2,0,0);
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

void doMeasUpdateKF_heightOnly(float const &meas, float const &measCov, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov)
{
	Array2D<DTYPE> gainKF(6,1,0.0);
	gainKF[2][0] = errCov[2][2]/(errCov[2][2]+measCov);
	gainKF[5][0] = errCov[2][5]/(errCov[2][2]+measCov);

	// \hat{x} = \hat{x} + K (meas - C \hat{x})
	float err = meas-state[2][0];
	state[2][0] += gainKF[2][0]*err;
	state[5][0] += gainKF[5][0]*err;

	// S = (I-KC) S
	errCov[2][2] -= gainKF[2][0]*errCov[2][2];
	errCov[2][5] -= gainKF[2][0]*errCov[2][5];
	errCov[5][5] -= gainKF[5][0]*errCov[2][5];
	errCov[5][2] = errCov[2][5];
}

void doMeasUpdateKF_velOnly(Array2D<DTYPE> const &meas, Array2D<DTYPE> const &measCov, Array2D<DTYPE> &state, Array2D<DTYPE> &errCov)
{
	if(norm2(meas) > 10)
		return; // screwy measuremnt

	Array2D<DTYPE> gainKF(6,3,0.0);
	for(int i=0; i<3; i++)
	{
		gainKF[i][i] = errCov[i][i+3]/(errCov[i+3][i+3]+measCov[i][i]);
		gainKF[i+3][i] = errCov[i+3][i+3]/(errCov[i+3][i+3]+measCov[i][i]);
	}

	// \hat{x} = \hat{x} + K (meas - C \hat{x})
	Array2D<DTYPE> err = meas-submat(state,3,5,0,0);
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
				Array2D<DTYPE> &kfState, Array2D<DTYPE> &kfErrCov, 
				Array2D<DTYPE> const &kfPosMeasCov, Array2D<DTYPE> const &kfVelMeasCov, Array2D<DTYPE> const &kfDynCov,
				float &thrust, Array2D<DTYPE> &thrustDir,
				float const &mass)
{
	Time dataTime = data->timestamp;
	Array2D<DTYPE> accel = thrust/mass*thrustDir;
	accel[2][0] -= GRAVITY;
	float dt = Time::calcDiffNS(curTime, dataTime)/1.0e9;
	doTimeUpdateKF(accel, dt, kfState, kfErrCov, kfDynCov);

	switch(data->type)
	{
		case DATA_TYPE_VICON_POS:
			doMeasUpdateKF_posOnly(static_pointer_cast<DataVector<DTYPE> >(data)->data, submat(kfPosMeasCov,0,2,0,2), kfState, kfErrCov);
			break;
		case DATA_TYPE_VICON_VEL:
			cout << "Doing vicon vel" << endl;
			doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<DTYPE> >(data)->data, submat(kfVelMeasCov,3,5,3,5), kfState, kfErrCov);
			break;
		case DATA_TYPE_OPTIC_FLOW_VEL:
			doMeasUpdateKF_velOnly(static_pointer_cast<DataVector<DTYPE> >(data)->data, submat(kfVelMeasCov,3,5,3,5), kfState, kfErrCov);
			break;
		case DATA_TYPE_THRUST_DIR:
			thrustDir.inject( static_pointer_cast<DataVector<DTYPE> >(data)->data );
			break;
		case DATA_TYPE_ATTITUDE:
			{
				Array2D<DTYPE> attState = static_pointer_cast<DataVector<DTYPE> >(data)->data;
				Array2D<DTYPE> attSO3 = createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]);
				Array2D<DTYPE> attBias(3,1);
				attBias[0][0] = 0.005; attBias[1][0] = 0.01; attBias[2][0] = 0.00;
				float s1 = sin(attState[2][0]); float c1 = cos(attState[2][0]);
				float s2 = sin(attState[1][0]-attBias[1][0]); float c2 = cos(attState[1][0]-attBias[1][0]);
				float s3 = sin(attState[0][0]-attBias[0][0]); float c3 = cos(attState[0][0]-attBias[0][0]);
				thrustDir[0][0] = s1*s3+c1*c3*s2;
				thrustDir[1][0] = c3*s1*s2-c1*s3;
				thrustDir[2][0] = c2*c3;
			}
			break;
		case DATA_TYPE_MOTOR_CMDS:
			{
				Array2D<DTYPE> cmds = static_pointer_cast<DataVector<DTYPE> >(data)->data;
				thrust = 0;
				for(int i=0; i<4; i++)
					thrust += cmds[i][0];
				float forceGain = 0.00235;
				thrust *= forceGain;
			}
			break;
		default:
			cout << "Observer_Translational::applyData() --> Unknown data type: " << data->type << endl;
	}


	return dataTime;
}
