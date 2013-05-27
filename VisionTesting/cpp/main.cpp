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
				list<shared_ptr<DataVector> > &errCovList
				);
void loadPcLog(String filename,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList
		);
vector<string> tokenize(string str);

void findPoints(cv::Mat const &img, vector<cv::Point2f> &pts, float const &minDistance, float const &qualityLevel);
void findPoints(cv::Mat const &img, vector<cv::KeyPoint> &pts, float const &minDistance, float const &qualityLevel);

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
	Array2D<double> rotCamToPhone3 = blkdiag(rotCamToPhone, blkdiag(rotCamToPhone, rotCamToPhone));

	Array2D<double> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<double> rotPhoneToCam2 = transpose(rotCamToPhone2);
	Array2D<double> rotPhoneToCam3 = transpose(rotCamToPhone3);

	int keypress = 0;
	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);

	cv::Mat img, imgGray, imgGrayRaw, imgPrev;
	vector<cv::Point2f> prevPoints, curPoints;
	Time prevTime, curTime;
	Array2D<double> attPrev, attCur, attChange;

	Array2D<double> Sv(3,3,0.0);
	double sz;
//	Array2D<double> Sv = pow(0.2,2)*createIdentity(3);
//	double sz = 0.01;
//		double sz = errCov[2][0];
	double focalLength = 3.7*640/5.76;
	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = 2*pow(5,2);
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

	cv::Point2f center(330,240);

	tbb::task_scheduler_init init;

	cout << "//////////////////////////////////////////////////" << endl;

	float qualityLevel = 0.1;
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
		long long numMatchesAccum = 0;
		vector<cv::KeyPoint> prevKp, curKp;
		cv::Mat prevDescriptors, curDescriptors;

		fstream fs("../orbResults.txt", fstream::out);
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
			Array2D<double> viconAttState = Data::interpolate(curTime, viconAttDataList);
			Array2D<double> viconTransState = Data::interpolate(curTime, viconTransDataList);

			Array2D<double> errCov1 = Data::interpolate(curTime, errCovList);
			Array2D<double> errCov(6,6,0.0);
			for(int i=0; i<3; i++)
			{
				errCov[i][i] = errCov1[i][0];
				errCov[i][i+3] = errCov1[i+3][0];
				errCov[i+3][i+3] = errCov1[i+6][0];
			}

			// Rotate to camera coords
			attState = matmult(rotPhoneToCam2, attState);
			transState = matmult(rotPhoneToCam2, transState);
			errCov = matmult(rotPhoneToCam2, matmult(errCov, transpose(rotPhoneToCam2)));
			viconAttState = matmult(rotPhoneToCam2, viconAttState);
			viconTransState = matmult(rotPhoneToCam2, viconTransState);

			attPrev.inject(attCur);
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//				attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );

			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);

			curKp.swap(prevKp);
			curKp.clear();
//			featureDetector.detect(imgGray, curKp);
			findPoints(imgGray, curKp, minDistance, qualityLevel);

			numFeaturesAccum += curKp.size();

			prevDescriptors = curDescriptors;
			curDescriptors.release();
			curDescriptors.data = NULL;
			cv::OrbFeatureDetector extractor(500, 2.0f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
			extractor.compute(imgGray, curKp, curDescriptors);

			curDescriptors.convertTo(curDescriptors, CV_32F);

			cv::FlannBasedMatcher matcher;
			vector<cv::DMatch> matches;
			matcher.match(prevDescriptors, curDescriptors, matches);

			float minDist = 100;
			for(int i=0; i<matches.size(); i++)
				minDist = min(matches[i].distance, minDist);

			vector<cv::DMatch> goodMatches(matches);
//			for(int i=0; i<matches.size(); i++)
//				if(matches[i].distance < 5.0*minDist) goodMatches.push_back(matches[i]);
			numMatchesAccum += goodMatches.size();

			int N1, N2;
			N1 = N2 = goodMatches.size();
			Array2D<double> C(N1+1, N2+1, 0.0); // the extra row and column will stay at zero
			vector<cv::Point2f> prevPoints(N1), curPoints(N2);
			for(int i=0; i<N1; i++)
			{
				C[i][i] = 1;

				int idx1 = goodMatches[i].queryIdx;
				int idx2 = goodMatches[i].trainIdx;
				prevPoints[i] = prevKp[idx1].pt-center;
				curPoints[i] = curKp[idx2].pt-center;
			}

			// MAP velocity and height
			Array2D<double> mv(3,1,0.0);
			double mz;
			if(useViconState)
			{
				mv[0][0] = viconTransState[3][0];
				mv[1][0] = viconTransState[4][0];
				mv[2][0] = viconTransState[5][0];

				mz = -viconTransState[2][0]; // in camera coords, z is flipped
			}
			else
			{
				mv[0][0] = transState[3][0];
				mv[1][0] = transState[4][0];
				mv[2][0] = transState[5][0];

				mz = -transState[2][0]; // in camera coords, z is flipped
			}

			mz -= 0.1; // camera to vicon markers offset

			double dt;
			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;
			Array2D<double> omega = logSO3(attChange, dt);

			sz = sqrt( errCov[2][2]);
			Sv[0][0] = errCov[3][3];
			Sv[1][1] = errCov[4][4];
			Sv[2][2] = errCov[5][5];

			if(curPoints.size() > 0)
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

//			if(imgPrev.data != NULL)
//			{
//				cv::Mat imgMatches(img.rows,2*img.cols,img.type());
//				cv::Mat imgL = imgPrev.clone();
//				cv::Mat imgR = img.clone();
//				for(int i=0; i<prevPoints.size(); i++)
//					cv::circle(imgL, prevPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
//				for(int j=0; j<curPoints.size(); j++)
//					cv::circle(imgR, curPoints[j]+center, 4, cv::Scalar(0,0,255), -1);
//				imgL.copyTo(imgMatches(cv::Rect(0,0,img.cols,img.rows)));
//				imgR.copyTo(imgMatches(cv::Rect(img.cols,0,img.cols,img.rows)));
////			drawMatches(imgPrev, prevKp, img, curKp, goodMatches, imgMatches, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//				imshow("chad",imgMatches);
//			}
//			keypress = cv::waitKey() % 256;

			imgPrev = img;

//			img.release();
//			img.data = NULL;
		}

		fs.close();

		cout << "Avg num ORB features: " << ((double)numFeaturesAccum)/imgList.size() << endl;
		cout << "Avg num ORB matches: " << ((double)numMatchesAccum)/imgList.size() << endl;
		cout << "ORB time: " << begin.getElapsedTimeMS()/1.0e3 << endl;

	}

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

		Array2D<double> mv(3,1), omega(3,1), vel;
		double mz, dt, z;

		vector<pair<Array2D<double>, Array2D<double> > > priorDistList;

		Array2D<double> C;

double ts0, ts1, ts2, ts3, ts4, ts5, ts6, ts7;
ts0 = ts1 = ts2 = ts3 = ts4 = ts5 = ts6 = ts7 = 0;
Time t0;

		fstream fs("../mapResults.txt", fstream::out);
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
			attState.inject(Data::interpolate(curTime, attDataList));
			transState.inject(Data::interpolate(curTime, transDataList));
			viconAttState.inject(Data::interpolate(curTime, viconAttDataList));
			viconTransState.inject(Data::interpolate(curTime, viconTransDataList));

			errCov1.inject(Data::interpolate(curTime, errCovList));
			for(int i=0; i<3; i++)
			{
				errCov[i][i] = errCov1[i][0];
				errCov[i][i+3] = errCov1[i+3][0];
				errCov[i+3][i+3] = errCov1[i+6][0];
			}

			// Rotate to camera coords
			attState = matmult(rotPhoneToCam2, attState);
			transState = matmult(rotPhoneToCam2, transState);
			errCov = matmult(rotPhoneToCam2, matmult(errCov, transpose(rotPhoneToCam2)));
			viconAttState = matmult(rotPhoneToCam2, viconAttState);
			viconTransState = matmult(rotPhoneToCam2, viconTransState);

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

ts1 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			curPoints.swap(prevPoints);
			findPoints(imgGray, curPoints, minDistance, qualityLevel);
			numFeaturesAccum += curPoints.size();

ts2 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			/////////////////////////////////////////////////////
			//  Prior distributions
			for(int i=0; i<curPoints.size(); i++)
				curPoints[i] -= center;
			if(useViconState)
			{
				mv[0][0] = viconTransState[3][0];
				mv[1][0] = viconTransState[4][0];
				mv[2][0] = viconTransState[5][0];

				mz = -viconTransState[2][0]; // in camera coords, z is flipped
			}
			else
			{
				mv[0][0] = transState[3][0];
				mv[1][0] = transState[4][0];
				mv[2][0] = transState[5][0];

				mz = -transState[2][0]; // in camera coords, z is flipped
			}

			mz -= 0.1; // camera to vicon markers offset

			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;
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
		cout << "\tts1: " << ts1 << endl;

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

void findPoints(cv::Mat const &img, vector<cv::Point2f> &pts, float const &minDistance, float const &qualityLevel)
{
	vector<cv::KeyPoint> kp;
	findPoints(img, kp, minDistance, qualityLevel);
	cv::KeyPoint::convert(kp, pts);
}

void findPoints(cv::Mat const &img, vector<cv::KeyPoint> &pts, float const &minDistance, float const &qualityLevel)
{
	vector<cv::KeyPoint> tempKp;
	FAST(img, tempKp, 4, true);

	sort(tempKp.begin(), tempKp.end(), [&](cv::KeyPoint const &a, cv::KeyPoint const &b){return a.response > b.response;});
	int maxScore = tempKp[0].response;

	// group points into grid
	const int cellSize = minDistance+0.5;
	const int nGridX= (img.cols+cellSize-1)/cellSize;
	const int nGridY= (img.rows+cellSize-1)/cellSize;

	vector<vector<int> > grids(nGridX*nGridY); // stores index of keypoints in each grid
	vector<int> gridId;
	int kpId = 0;
	while(tempKp[kpId].response > qualityLevel*maxScore && kpId < tempKp.size())
	{
		int xGrid = tempKp[kpId].pt.x / cellSize;
		int yGrid = tempKp[kpId].pt.y / cellSize;
		int gid = yGrid*nGridX+xGrid;
		grids[gid].push_back(kpId);
		gridId.push_back(gid);

		kpId++;
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

		// since tempKp is sorted already, the current point is the strongest
		// remaining point
		pts.push_back(tempKp[i]);

		// turn off other points in this grid
		int gid = gridId[i];
		for(int j=0; j<grids[gid].size(); j++)
		{
			int kpId = grids[gid][j];
			if(kpId != i)
				isActive[kpId] = false;
		}

		// need to check neighbor grids too
		int xGrid = gid % nGridX;
		int yGrid = gid / nGridX;

		cv::Point2f *curPt = &tempKp[i].pt;
		int neighborOffsetX[] = {-1, 0, 1, -1, 1, -1, 0, 1};
		int neighborOffsetY[] = {-1, -1, -1, 0, 0, 1, 1, 1};
		for(int j=0; j<8; j++)
		{
			int nx = xGrid+neighborOffsetX[j];
			int ny = yGrid+neighborOffsetY[j];
			if(nx < 0 || nx > nGridX-1 || ny < 0 || ny > nGridY-1)
				continue;

			int ngid = ny*nGridX+nx;
			for(int k=0; k<grids[ngid].size(); k++)
			{
				int kpId = grids[ngid][k];
				if(!isActive[kpId])
					continue;

				cv::Point2f *pt = &tempKp[kpId].pt;
				if(abs(pt->x - curPt->x) <= minDistance && abs(pt->y - curPt->y) <= minDistance)
					isActive[kpId] = false;
			}
		}
	}
}
