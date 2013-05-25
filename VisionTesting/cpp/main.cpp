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

//enum LogIDs
//{
//	LOG_ID_ACCEL = 1,
//	LOG_ID_GYRO = 2,
//	LOG_ID_MAGNOMETER = 3,
//	LOG_ID_PRESSURE = 4,
//	LOG_ID_IMAGE = 10,
//	LOG_ID_GYRO_BIAS = -1003,
//	LOG_ID_OBSV_ANG_INNOVATION = -1004,
//	LOG_ID_OPTIC_FLOW = 12345,
//	LOG_ID_OBSV_TRANS_ATT_BIAS = -710,
//	LOG_ID_OBSV_TRANS_FORCE_GAIN = -711,
//	LOG_ID_BAROMETER_HEIGHT = 1234,
//	LOG_ID_MOTOR_CMDS = -1000,
//	LOG_ID_CUR_ATT = -1002,
//	LOG_ID_CUR_TRANS_STATE = -1012,
//	LOG_ID_RECEIVE_VICON = 700,
//	LOG_ID_CAMERA_POS = 800,
//	LOG_ID_KALMAN_ERR_COV = -720,
//};

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

	Array2D<double> Sv = pow(0.2,2)*createIdentity(3);
	double sz = 0.01;
//		double sz = errCov[2][0];
	double focalLength = 3.7*640/5.76;
	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = 2*pow(5,2);
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

	cv::Point2f center(330,240);

	tbb::task_scheduler_init init;

	cout << "//////////////////////////////////////////////////" << endl;

//	cv::FastFeatureDetector featureDetector;
	int maxCorners = 1000;
	double qualityLevel = 0.01;
	double minDistance = 30;
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k=0.04;
	cv::GoodFeaturesToTrackDetector featureDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k);

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

double ts2=0;
Time t0;

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
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//				attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );

			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);

t0.setTime();
			curKp.swap(prevKp);
			curKp.clear();
			featureDetector.detect(imgGray, curKp);
			numFeaturesAccum += curKp.size();

ts2 += t0.getElapsedTimeNS()/1.0e9;
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

		cout << "ts2: " << ts2 << endl;
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

		fstream fs("../mapResults.txt", fstream::out);
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
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//			attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );

			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);

			curPoints.swap(prevPoints);
//			int maxPoints= 100;
//			double qualityLevel = 0.05;
//			double minDistance = 2*maxSigma;
//			int blockSize = 3;
//			bool useHarrisDetector = false;
//			cv::goodFeaturesToTrack(imgGray, curPoints, maxPoints, qualityLevel, minDistance, cv::noArray(), blockSize, useHarrisDetector);
			cv::FastFeatureDetector featureDectector;
			vector<cv::KeyPoint> kp;
////////////////////////////////////////////////////////////////////////////////////////////////////
//			featureDetector.detect(imgGray, kp);
//			int nGrid = 2;
//			cv::Size imgSize = imgGray.size();
//			vector<cv::Mat> frames(nGrid*nGrid);
//			vector<cv::Point2f> maskOffset(nGrid*nGrid);
//			for(int i=0; i<nGrid; i++)
//			{
//				int left = max(0,(int)(i*imgSize.width/nGrid+0.5));
//				int width = min(imgSize.width-left, (int)(imgSize.width/nGrid+0.5));
//				for(int j=0; j<nGrid; j++)
//				{
//					int top = max(0,(int)(j*imgSize.height/nGrid+0.5));
//					int height = min(imgSize.height-top, (int)(imgSize.height/nGrid+0.5));
//					cv::Rect mask(left, top, width, height);
//					cv::Point2f offset(left, top);
//					frames[i*nGrid+j] = imgGray(mask);
//					maskOffset[i*nGrid+j] = offset;
//				}
//			}
//			vector<vector<cv::KeyPoint> > kpList(nGrid*nGrid);
//			for(int i=0; i<kpList.size(); i++)
//				featureDetector.detect(frames[i], kpList[i]);
//			for(int i=0; i<kpList.size(); i++)
//			{
//				for(int j=0; j<kpList[i].size(); j++)
//					kpList[i][j].pt = kpList[i][j].pt+maskOffset[i];
//				move(kpList[i].begin(), kpList[i].end(), back_inserter(kp));
//			}
////////////////////////////////////////////////////////////////////////////////////////////////////
			{
				cv::Mat eig, tmp;
//////////////////////////////////////////////////
//				int blockSize = 3;
//				cornerMinEigenVal(imgGray, eig, blockSize, 3);
//				double maxVal = 0;
//				minMaxLoc(eig, 0, &maxVal, 0, 0);
//				threshold(eig, eig, maxVal*qualityLevel, 0, cv::THRESH_TOZERO);
//				dilate(eig, tmp, cv::Mat());
//				vector<const float*> tmpCorners;
//
//				cv::Size imgsize = imgGray.size();
//
//				// collect list of pointers to features - put them into temporary image
//				for( int y = 1; y < imgsize.height - 1; y++ )
//				{
//					const float* eig_data = (const float*)eig.ptr(y);
//					const float* tmp_data = (const float*)tmp.ptr(y);
//
//					for( int x = 1; x < imgsize.width - 1; x++ )
//					{
//						float val = eig_data[x];
//						if( val != 0 && val == tmp_data[x]/* && (!mask_data || mask_data[x]) */)
//							tmpCorners.push_back(eig_data + x);
//					}
//				}
//
//				sort(tmpCorners.begin(), tmpCorners.end(), [&](const float *a, const float *b) {return *a > *b;});
//				int chad = 0;
//////////////////////////////////////////////////
				vector<cv::KeyPoint> tempKp;
				FAST(imgGray, tempKp, 5, true);

				double scale = (double)(1 << 2)*blockSize;
				scale = 1.0/scale;
				cv::Mat Dx, Dy;
				Scharr(imgGray, Dx, CV_32F, 1, 0, scale);
				Scharr(imgGray, Dy, CV_32F, 0, 1, scale);

				eig.create(imgGray.size(), CV_32F);
				eig = cv::Scalar(0);

				for(int i=0; i<tempKp.size(); i++)
				{
					int y = tempKp[i].pt.x;
					int x = tempKp[i].pt.y;

					float *eig_data = (float*)(eig.data+x*eig.step);

					const float *dxdata_m1, *dxdata, *dxdata_p1;
					const float *dydata_m1, *dydata, *dydata_p1;
					if(x > 0)
					{
						dxdata_m1 = (const float*)(Dx.data+(x-1)*Dx.step);
						dydata_m1 = (const float*)(Dy.data+(x-1)*Dy.step);
					}
					else
						dxdata_m1 = dydata_m1 = NULL;

					dxdata = (const float*)(Dx.data+x*Dx.step);
					dydata = (const float*)(Dy.data+x*Dy.step);
					if(x < (imgGray.rows-1) )
					{
						dxdata_p1 = (const float*)(Dx.data+(x+1)*Dx.step);
						dydata_p1 = (const float*)(Dy.data+(x+1)*Dy.step);
					}
					else
						dxdata_p1 = dydata_p1 = NULL;

					float a, b, c;
					a = b = c = 0;

					int kLow, kHigh;
					if(y == 0)
						kLow = 0;
					else
						kLow = -1;
					if(y == imgGray.cols-1)
						kHigh = 0;
					else
						kHigh = 1;
					for(int k=kLow; k<=kHigh; k++)
					{
						double multiple = 1;
						if(x > 0)
						{
							a += pow(dxdata_m1[y+k],2);
							b += dxdata_m1[y+k]*dydata_m1[y+k];
							c += pow(dydata_m1[y+k],2);
						}
						else
							multiple = 2;

						if(x < imgGray.rows-1)
						{
							a += pow(dxdata_p1[y+k],2);
							b += dxdata_p1[y+k]*dydata_p1[y+k];
							c += pow(dydata_p1[y+k],2);
						}
						else
							multiple = 2;

						a += multiple*pow(dxdata[y+k],2);
						b += multiple*dxdata[y+k]*dydata[y+k];
						c += multiple*pow(dydata[y+k],2);
					}

					a *= 0.5f;
					c *= 0.5;
					eig_data[y] = (float)((a+c)-sqrt((a-c)*(a-c) + b*b));
				}


//				for(int i=0; i<imgGray.rows; i++)
//				{
//					float *eig_data = (float*)(eig.data+i*eig.step);
//
//					const float *dxdata_m1, *dxdata, *dxdata_p1;
//					const float *dydata_m1, *dydata, *dydata_p1;
//					if(i > 0)
//					{
//						dxdata_m1 = (const float*)(Dx.data+(i-1)*Dx.step);
//						dydata_m1 = (const float*)(Dy.data+(i-1)*Dy.step);
//					}
//					else
//						dxdata_m1 = dydata_m1 = NULL;
//
//					dxdata = (const float*)(Dx.data+i*Dx.step);
//					dydata = (const float*)(Dy.data+i*Dy.step);
//					if(i < (imgGray.rows-1) )
//					{
//						dxdata_p1 = (const float*)(Dx.data+(i+1)*Dx.step);
//						dydata_p1 = (const float*)(Dy.data+(i+1)*Dy.step);
//					}
//					else
//						dxdata_p1 = dydata_p1 = NULL;
//
//					for(int j=0; j<imgGray.cols; j++)
//					{
//						float a, b, c;
//						a = b = c = 0;
//
//						int kLow, kHigh;
//						if(j == 0)
//							kLow = 0;
//						else
//							kLow = -1;
//						if(j == imgGray.cols-1)
//							kHigh = 0;
//						else
//							kHigh = 1;
//						for(int k=kLow; k<=kHigh; k++)
//						{
//							double multiple = 1;
//							if(i > 0)
//							{
//								a += pow(dxdata_m1[j+k],2);
//								b += dxdata_m1[j+k]*dydata_m1[j+k];
//								c += pow(dydata_m1[j+k],2);
//							}
//							else
//								multiple = 2;
//
//							if(i < imgGray.rows-1)
//							{
//								a += pow(dxdata_p1[j+k],2);
//								b += dxdata_p1[j+k]*dydata_p1[j+k];
//								c += pow(dydata_p1[j+k],2);
//							}
//							else
//								multiple = 2;
//
//							a += multiple*pow(dxdata[j+k],2);
//							b += multiple*dxdata[j+k]*dydata[j+k];
//							c += multiple*pow(dydata[j+k],2);
//						}
//
//						a *= 0.5f;
//						c *= 0.5;
//						eig_data[j] = (float)((a+c)-sqrt((a-c)*(a-c) + b*b));
//					}
//
//				}

//				cv::Mat cov(imgGray.size(), CV_32FC3);
//				for(int i=0; i<imgGray.rows; i++)
//				{
//					float *cov_data = (float*)(cov.data+i*cov.step);
//					const float* dxdata = (const float*)(Dx.data+i*Dx.step);
//					const float* dydata = (const float*)(Dy.data+i*Dy.step);
//					
//					for(int j=0; j<imgGray.cols; j++)
//					{
//						float dx = dxdata[j];
//						float dy = dydata[j];
//
//						cov_data[j*3] = dx*dx;
//						cov_data[j*3+1] = dx*dy;
//						cov_data[j*3+2] = dy*dy;
//					}
//				}
//
//				cv::boxFilter(cov, cov, cov.depth(), cv::Size(blockSize, blockSize), cv::Point(-1,-1), false);
//				eig.create(imgGray.size(), CV_32F);
//				// assume cov is continuous
//				for(int i=0; i<cov.rows; i++)
//				{
//					const float *cv = (const float*)(cov.data+cov.step*i);
//					float *dst = (float*)(eig.data+eig.step*i);
//
//					for(int j=0; j<cov.cols; j++)
//					{
//						float a = cv[j*3]*0.5f;
//						float b = cv[j*3+1];
//						float c = cv[j*3+2]*0.5;
//						dst[j] = (float)((a+c)-sqrt((a-c)*(a-c)+b*b));
//					}
//				}

				double maxVal = 0;
				minMaxLoc(eig, 0, &maxVal, 0, 0);
				threshold(eig, eig, maxVal*qualityLevel, 0, cv::THRESH_TOZERO);
				dilate(eig, tmp, cv::Mat());
				vector<const float*> tmpCorners;

				cv::Size imgsize = imgGray.size();

				// collect list of pointers to features - put them into temporary image
				for( int y = 1; y < imgsize.height - 1; y++ )
				{
					const float* eig_data = (const float*)eig.ptr(y);
					const float* tmp_data = (const float*)tmp.ptr(y);

					for( int x = 1; x < imgsize.width - 1; x++ )
					{
						float val = eig_data[x];
						if( val != 0 && val == tmp_data[x]/* && (!mask_data || mask_data[x]) */)
							tmpCorners.push_back(eig_data + x);
					}
				}

				sort(tmpCorners.begin(), tmpCorners.end(), [&](const float *a, const float *b) {return *a > *b;});
//////////////////////////////////////////////////
				vector<cv::Point2f> corners;
				size_t i, j, total = tmpCorners.size(), ncorners = 0;
				{
					// Partition the image into larger grids
					int w = imgGray.cols;
					int h = imgGray.rows;

					const int cell_size = cvRound(minDistance);
					const int grid_width = (w + cell_size - 1) / cell_size;
					const int grid_height = (h + cell_size - 1) / cell_size;

					std::vector<std::vector<cv::Point2f> > grid(grid_width*grid_height);

					int minDistanceSq = minDistance*minDistance;

					for( i = 0; i < total; i++ )
					{
						int ofs = (int)((const uchar*)tmpCorners[i] - eig.data);
						int y = (int)(ofs / eig.step);
						int x = (int)((ofs - y*eig.step)/sizeof(float));

						bool good = true;

						int x_cell = x / cell_size;
						int y_cell = y / cell_size;

						int x1 = x_cell - 1;
						int y1 = y_cell - 1;
						int x2 = x_cell + 1;
						int y2 = y_cell + 1;

						// boundary check
						x1 = std::max(0, x1);
						y1 = std::max(0, y1);
						x2 = std::min(grid_width-1, x2);
						y2 = std::min(grid_height-1, y2);

						for( int yy = y1; yy <= y2; yy++ )
						{
							for( int xx = x1; xx <= x2; xx++ )
							{
								vector <cv::Point2f> &m = grid[yy*grid_width + xx];

								if( m.size() )
								{
									for(j = 0; j < m.size(); j++)
									{
										float dx = x - m[j].x;
										float dy = y - m[j].y;

										if( dx*dx + dy*dy < minDistanceSq )
										{
											good = false;
											goto break_out;
										}
									}
								}
							}
						}

break_out:

						if(good)
						{
							// printf("%d: %d %d -> %d %d, %d, %d -- %d %d %d %d, %d %d, c=%d\n",
							//    i,x, y, x_cell, y_cell, (int)minDistance, cell_size,x1,y1,x2,y2, grid_width,grid_height,c);
							grid[y_cell*grid_width + x_cell].push_back(cv::Point2f((float)x, (float)y));

							corners.push_back(cv::Point2f((float)x, (float)y));
							++ncorners;

							if( maxCorners > 0 && (int)ncorners == maxCorners )
								break;
						}
					}
				}

				kp.resize(corners.size());
				vector<cv::Point2f>::const_iterator corner_it = corners.begin();
				vector<cv::KeyPoint>::iterator keypoint_it = kp.begin();
				for( ; corner_it != corners.end(); ++corner_it, ++keypoint_it )
					*keypoint_it = cv::KeyPoint( *corner_it, (float)blockSize );
			}
////////////////////////////////////////////////////////////////////////////////////////////////////
			curPoints.clear();
			for(int i=0; i<kp.size(); i++)
				curPoints.push_back(kp[i].pt);
			numFeaturesAccum += curPoints.size();

			/////////////////////////////////////////////////////
			//  Prior distributions
			for(int i=0; i<curPoints.size(); i++)
				curPoints[i] -= center;
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
			vector<pair<Array2D<double>, Array2D<double> > > priorDistList;
			priorDistList = calcPriorDistributions(prevPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);

			// Correspondence
			Array2D<double> C;
			C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv);

			for(int i=0; i<C.dim1()-1; i++)
				for(int j=0; j<C.dim2()-1; j++)
					numMatchesAccum += C[i][j];

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

//				cv::Point2f pt(priorDistList[i].first[0][0], priorDistList[i].first[1][0]);
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
		cout << "Avg num matches: " << ((double)numMatchesAccum)/imgList.size() << endl;
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
