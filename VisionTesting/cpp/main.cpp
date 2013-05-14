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

#include "toadlet/egg.h"

//#include "mser.h"
#include "../../Rover/cpp/Data.h"
#include "../../Rover/cpp/TNT/tnt.h"
#include "../../Rover/cpp/TNT/jama_cholesky.h"
#include "../../Rover/cpp/TNT/jama_eig.h"
#include "../../Rover/cpp/TNT_Utils.h"
#include "../../Rover/cpp/Time.h"
#include "../../Rover/cpp/constants.h"

using namespace std;
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

inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
													Array2D<double> const &mv, Array2D<double> const &Sv, 
													double const &mz, double const &varz, 
													double const &focalLength, double const &dt,
													Array2D<double> const &omega);
Array2D<double> calcCorrespondence(vector<pair<Array2D<double>, 
									Array2D<double> > > const &priorDistList, 
									vector<cv::Point2f> const &curPointList, 
									Array2D<double> const &Sn, 
									Array2D<double> const &SnInv);

void computeMAPEstimate(Array2D<double> &velMAP /*out*/, double &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<double> const &C, // correspondence matrix
						Array2D<double> const &mv, // velocity mean
						Array2D<double> const &Sv, // velocity covariance
						double const &mz, // height mean
						double const &vz, // height variance
						Array2D<double> const &Sn, // feature measurement covariance
						double const &focalLength, double const &dt, Array2D<double> const &omega);

template<class T>
Array2D<T> logSO3(Array2D<T> const &R, double theta);

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
//	int imgId = 1482;
	int imgId = 1700;
	stringstream ss;
	cv::Mat img, imgGray, imgGrayRaw;
	vector<cv::Point2f> prevPoints, curPoints;
	Time prevTime, curTime;
	curTime.setTimeMS(0);
	int imgIdx = 0;
	double maxSigma = 30;
	Array2D<double> attPrev, attCur, attChange;
	attPrev = createIdentity(3);
	attCur = createIdentity(3);
	attChange = createIdentity(3);
	while(keypress != (int)'q' && imgIdx < imgIdList.size())
	{
		ss.str("");
		ss << "img_" << imgId++ << ".bmp";
		img = cv::imread(imgDir+"/"+ss.str());
		cout << imgDir << "/" << ss.str() << endl;
		if(img.data != NULL)
		{
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
			Time start;
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
//			cv::bilateralFilter(imgGrayRaw, imgGray, 5, 11, 3);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);

			int maxPoints= 300;
			double qualityLevel = 0.05;
			double minDistance = maxSigma;
			curPoints.swap(prevPoints);
			cv::goodFeaturesToTrack(imgGray, curPoints, maxPoints, qualityLevel, minDistance);
			cv::Point2f center(320,240);
			cout << "\t" << "Proc time: " << start.getElapsedTimeNS()/1.0e9 << endl;
			cout << "\t" << curPoints.size() << " points found" << endl;

			start.setTime();
			/////////////////////////////////////////////////////
			//  Prior distributions
			for(int i=0; i<curPoints.size(); i++)
				curPoints[i] -= center;
			Array2D<double> mv(3,1,0.0);
			mv[0][0] = viconTransState[3][0];
			mv[1][0] = viconTransState[4][0];
			mv[2][0] = viconTransState[5][0];
			Array2D<double> Sv = 0.1*0.1*createIdentity(3);
			double mz = -viconTransState[2][0]; // in camera coords, z is flipped
			mz -= 0.1; // camera to vicon markers offset
			double sz = 0.02;
//			double sz = errCov[2][0];
			double focalLength = 3.7*640/5.76;
			double dt;
			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;
			Array2D<double> omega = logSO3(attChange, dt);
			vector<pair<Array2D<double>, Array2D<double> > > priorDistList;
			priorDistList = calcPriorDistributions(prevPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);

			// Correspondence
			Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
			Sn[0][0] = Sn[1][1] = 2*5*5;
			SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];
			Array2D<double> C;
			C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv);

			// MAP velocity and height
			if(prevPoints.size() > 0)
			{
				Array2D<double> vel;
				double z;
				computeMAPEstimate(vel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
//cout << "    mv:\t" << mv[0][0] << "\t" << mv[1][0] << "\t" << mv[2][0] << endl;
//cout << "velEst:\t" << vel[0][0] << "\t" << vel[1][0] << "\t" << vel[2][0] << endl;
//cout << "  mz:\t" << mz << endl;
//cout << "zEst:\t" << z << endl;
			}

			cout << "\tVel calc time: " << start.getElapsedTimeNS()/1.0e9 << endl;


			//////////////////////////////////////////////////
			//Drawing
			// prior distributions
			if(priorDistList.size() > 0)
				maxSigma = 0;
			cv::Mat overlay = img.clone();
			for(int i=0; i<priorDistList.size(); i++)
			{
				Array2D<double> Sd = priorDistList[i].second;
				JAMA::Eigenvalue<double> eig_Sd(Sd);
				Array2D<double> V, D;
				eig_Sd.getV(V);
				eig_Sd.getD(D);

				for(int i=0; i<D.dim1(); i++)
					maxSigma = max(maxSigma, sqrt(D[i][i]));

				cv::Point2f pt(priorDistList[i].first[0][0], priorDistList[i].first[1][0]);
				double width = 2*sqrt(D[0][0]);
				double height = 2*sqrt(D[1][1]);
				double theta = atan2(V[1][0], V[0][0]);
				cv::RotatedRect rect(pt+center, cv::Size(width, height), theta);
				cv::ellipse(overlay, rect, cv::Scalar(255,0,0), -1);
			}
			double opacity = 0.3;
			cv::addWeighted(overlay, opacity, img, 1-opacity, 0, img);

			// current points
			for(int i=0; i<curPoints.size(); i++)
				cv::circle(img, curPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
			imshow("chad",img);

			keypress = cv::waitKey() % 256;

			img.release();
			img.data = NULL;
		}
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
//		for(int i=0; i<10000; i++)
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

vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
							Array2D<double> const &mv, Array2D<double> const &Sv, 
							double const &mz, double const &varz, 
							double const &focalLength, double const &dt, 
							Array2D<double> const &omega)
{
	double mvx = mv[0][0];
	double mvy = mv[1][0];
	double mvz = mv[2][0];
	double svx = sqrt(Sv[0][0]);
	double svy = sqrt(Sv[1][1]);
	double svz = sqrt(Sv[2][2]);
	double sz = sqrt(varz);
	double f = focalLength;

	// delta_x = q_x*v_z*dt-f*v_x*dt
	vector<Array2D<double> > mDeltaList, SDeltaList;
	double x, y;
	Array2D<double> mDelta(2,1), SDelta(2,2,0.0);
	for(int i=0; i<points.size(); i++)
	{
		x = points[i].x;
		y = points[i].y;

		mDelta[0][0] = x*mvz*dt-f*mvx*dt;
		mDelta[1][0] = y*mvz*dt-f*mvy*dt;

		SDelta[0][0] = pow(x*svz*dt,2)+pow(f*svx*dt, 2);
		SDelta[1][1] = pow(y*svz*dt,2)+pow(f*svy*dt, 2);

		mDeltaList.push_back(mDelta.copy());
		SDeltaList.push_back(SDelta.copy());
	}

	// calc distribution of Z^-1
	double mz1Inv = 1.0/mz;
	double mz2Inv = 1.0/mz/mz;
	double log_sz = log(sz);
	double log_mz = log(mz);
	double del1LnOld = 0xFFFFFFFF;
	double del2LnOld = 0xFFFFFFFF;
	double del1Ln = 0;
	double del2Ln = 0;
	double fact;
	for(int k=1; k<10; k++)
	{
		fact = fact2ln(k);
		del1Ln = 2*k*log_sz+fact-(2*k+1)*log_mz;
		del2Ln = log(2*k+1)+2*k*log_sz+fact-(2*k+2)*log_mz;
		if(del1Ln < del1LnOld)
		{
			mz1Inv = mz1Inv+exp(del1Ln);
			del1LnOld = del1Ln;
		}
		if(del2Ln < del2LnOld)
		{
			mz2Inv = mz2Inv+exp(del2Ln);
			del2LnOld = del2Ln;
		}
	}

	// calc distribution moments
	vector<pair<Array2D<double>, Array2D<double> > > dDistList;
	for(int i=0; i<mDeltaList.size(); i++)
	{
		Array2D<double> mDelta = mDeltaList[i];
		Array2D<double> SDelta = SDeltaList[i];
		Array2D<double> md(2,1), Sd(2,2,0.0);
		md = mDelta*mz1Inv;

		// Shift the mean to so the distribution is for the location of q2 instead of q2-q1-Lw*w
		double x = points[i].x;
		double y = points[i].y;

		Array2D<double> Lw(2,3);
		Lw[0][0] = 1.0/f*x*y; 		Lw[0][1] = -1.0/f*(1+x*x); 	Lw[0][2] = y;
		Lw[1][0] = 1.0/f*(1+y*y);	Lw[1][1] = -1.0/f*x*y;		Lw[1][2] = -x;
		md = md+dt*matmult(Lw, omega);

		md[0][0] += x;
		md[1][0] += y;

		Sd[0][0] = (pow(mDelta[0][0],2)+SDelta[0][0])*mz2Inv-pow(mDelta[0][0],2)*pow(mz1Inv,2);
		Sd[1][1] = (pow(mDelta[1][0],2)+SDelta[1][1])*mz2Inv-pow(mDelta[1][0],2)*pow(mz1Inv,2);

		Sd[0][1] = Sd[1][0] = x*y*pow(dt,2)*pow(svz,2)*mz2Inv + mDelta[0][0]*mDelta[1][0]*(mz2Inv-pow(mz1Inv,2));

		dDistList.push_back(pair<Array2D<double>, Array2D<double> >(md.copy(), Sd.copy()));
	}

	return dDistList;
}

Array2D<double> calcCorrespondence(vector<pair<Array2D<double>, Array2D<double> > > const &priorDistList, vector<cv::Point2f> const &curPointList, Array2D<double> const &Sn, Array2D<double> const &SnInv)
{
	int N1 = priorDistList.size();
	int N2 = curPointList.size();

	if(N1 == 0 || N2 == 0)
		return Array2D<double>();
	Array2D<double> C(N1+1, N2+1, 0.0);
	double x, y;
	Array2D<double> mq(2,1), md(2,1);
	Array2D<double> Sd, SdInv, Sa, SaInv;
	Array2D<double> eye2 = createIdentity(2);
	vector<double> colSum(N2,0.0);
	double probNoCorr = 1.0/640.0/480.0;
	for(int j=0; j<N2; j++)
	{
		x = curPointList[j].x;
		y = curPointList[j].y;
		mq[0][0] = x;
		mq[1][0] = y;

		for(int i=0; i<N1; i++)
		{
			md = priorDistList[i].first;
			Sd = priorDistList[i].second;
			double xrange = 3*sqrt(Sd[0][0]+Sn[0][0]);
			double yrange = 3*sqrt(Sd[1][1]+Sn[1][1]);
			if(abs(x-md[0][0]) > xrange || abs(y-md[1][0]) > yrange )
				continue;
			JAMA::Cholesky<double> chol_Sd(Sd);
			SdInv = chol_Sd.solve(eye2);
			SaInv = (SdInv + SnInv);
			JAMA::Cholesky<double> chol_SaInv(SaInv);
			Sa = chol_SaInv.solve(eye2);
			Array2D<double> ma = matmult(Sa,matmult(SdInv,md)+matmult(SnInv,mq));
			double F = -1.0*matmultS(transpose(ma),matmult(SaInv,ma))+matmultS(transpose(md),matmult(SdInv,md))+matmultS(transpose(mq),matmult(SnInv,mq));
			double det_Sa = Sa[0][0]*Sa[1][1] - Sa[0][1]*Sa[1][0];
			double det_Sd = Sd[0][0]*Sd[1][1] - Sd[0][1]*Sd[1][0];
			double det_Sn = Sn[0][0]*Sn[1][1] - Sn[0][1]*Sn[1][0];
			C[i][j] = sqrt(det_Sa)/2.0/ICSL::Constants::PI/sqrt(det_Sd*det_Sn)*exp(-0.5*F);
			colSum[j] += C[i][j];
		}

		C[N1][j] = probNoCorr;
		colSum[j] += probNoCorr;
	}

	// Scale colums to unit sum
	for(int j=0; j<N2; j++)
		for(int i=0; i<N1+1; i++)
			C[i][j] /= colSum[j];

	// Now check if any of the rows sum to over 1
	for(int i=0; i<N1; i++)
	{
		double rowSum = 0;
		for(int j=0; j<N2; j++)
			rowSum += C[i][j];

		if(rowSum > 1)
		{
			for(int j=0; j<N2; j++)
				C[i][j] /= rowSum;

			C[i][N2] = 0;
		}
		else
			C[i][N2] = 1-rowSum;
	}

	// and, finally, reset the virtual point correspondence so columns sum to 1 again
	for(int j=0; j<N2; j++)
	{
		double colSum = 0;
		for(int i=0; i<N1; i++)
			colSum += C[i][j];
		C[N1][j] = 1-colSum;
	}

	return C;
}

template<class T>
Array2D<T> logSO3(Array2D<T> const &R, double theta)
{
	Array2D<T> w(3,1);
	if(theta > 0)
	{
		double sTheta = sin(theta);
		w[0][0] = (R[2][1]-R[1][2])/2.0/sTheta;
		w[1][0] = (R[0][2]-R[2][0])/2.0/sTheta;
		w[2][0] = (R[1][0]-R[0][1])/2.0/sTheta;
	}
	else
		w = Array2D<double>(3,1,0.0);

	return w;
}

void computeMAPEstimate(Array2D<double> &velMAP /*out*/, double &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<double> const &C, // correspondence matrix
						Array2D<double> const &mv, // velocity mean
						Array2D<double> const &Sv, // velocity covariance
						double const &mz, // height mean
						double const &vz, // height variance
						Array2D<double> const &Sn, // feature measurement covariance
						double const &focalLength, double const &dt, Array2D<double> const &omega)
{
	int N1 = prevPoints.size();
	int N2 = curPoints.size();
	double f = focalLength;

	JAMA::Cholesky<double> chol_Sv(Sv), chol_Sn(Sn);
	Array2D<double> SvInv = chol_Sv.solve(createIdentity(3));
	Array2D<double> SnInv = chol_Sn.solve(createIdentity(2));

	double sz = sqrt(vz);
	
	///////////////////////////////////////////////////////////////
	// Build up constant matrices
	///////////////////////////////////////////////////////////////
	list<Array2D<double> > LvList, LwList;
	Array2D<double> Lv(2,3), Lw(2,3);
	double x, y;
	for(int i=0; i<N1; i++)
	{
		double x = prevPoints[i].x;
		double y = prevPoints[i].y;
		
		Lv[0][0] = -f; Lv[0][1] = 0;  Lv[0][2] = x;
		Lv[1][0] = 0;  Lv[1][1] = -f; Lv[1][2] = y;
		LvList.push_back(Lv.copy());

		Lw[0][0] = 1.0/f*x*y; 		Lw[0][1] = -1.0/f*(1+x*x); 	Lw[0][2] = y;
		Lw[1][0] = 1.0/f*(1+y*y);	Lw[1][1] = -1.0/f*x*y;		Lw[1][2] = -x;
		LwList.push_back(Lw.copy());
	}

	list<Array2D<double> > AjList;
	list<Array2D<double> >::const_iterator iterLv;
	for(int j=0; j<N2; j++)
	{
		iterLv = LvList.begin();
		Array2D<double> Aj(2,3,0.0);
		for(int i=0; i<N1; i++)
		{
			Lv = *iterLv;
			Aj += C[i][j]*Lv;
			iterLv++;
		}
		Aj = dt*Aj;

		AjList.push_back(Aj.copy());
	}

	list<Array2D<double> >::const_iterator iterAj, iterLw;
	iterAj = AjList.begin();
	double s0 = 0;
	Array2D<double> s1_T(1,3,0.0), S2(3,3,0.0);
	Array2D<double> Aj;
	for(int j=0; j<N2; j++)
	{
		if( (1-C[N1][j]) < 1e-1)
		{
			iterAj++;
			continue;
		}

		Array2D<double> q1(2,1), q2(2,1);
		q2[0][0] = curPoints[j].x;
		q2[1][0] = curPoints[j].y;
		Array2D<double> Lw;
		iterLw = LwList.begin();
		Array2D<double> temp1(2,1,0.0);
		for(int i=0; i<N1; i++)
		{
			if(C[i][j] > 0.01)
			{
				q1[0][0] = prevPoints[i].x;
				q1[1][0] = prevPoints[i].y;
				Lw = *iterLw;

				temp1 += C[i][j]*(q2-q1-matmult(Lw,omega));
			}

			iterLw++;
		}

		Aj = *iterAj;
		Array2D<double> temp2 = matmult(SnInv, Aj);
		s0   += (1-C[N1][j])*matmultS(transpose(temp1), matmult(SnInv, temp1));
		s1_T += (1-C[N1][j])*matmult(transpose(temp1), temp2);
		S2   += (1-C[N1][j])*matmult(transpose(Aj), temp2);

		iterAj++;
	}

	Array2D<double> s1 = transpose(s1_T);

	// For easy evaluation of the objective function
	auto scoreFunc = [&](Array2D<double> const &vel, double const &z){ return -0.5*(
							s0
							-2.0/z*matmultS(s1_T, vel)
							+1.0/z/z*matmultS(transpose(vel), matmult(S2, vel))
							+matmultS( transpose(vel-mv), matmult(SvInv, vel-mv))
							+pow(z-mz,2)/vz
							);};

	// unique solution for optimal vel, given z
	auto solveVel =  [&](double const &z){
		Array2D<double> temp1 = 1.0/z*s1+matmult(SvInv,mv);
		Array2D<double> temp2 = 1.0/z/z*S2+SvInv;
		JAMA::Cholesky<double> chol_temp2(temp2);
		return chol_temp2.solve(temp1);
	};

	///////////////////////////////////////////////////////////////
	// Find a good starting point
	///////////////////////////////////////////////////////////////
	double scoreBest = -0xFFFFFF;
	Array2D<double> velBest(3,1), velTemp(3,1);
	double zBest;
	double interval = 6.0*sz/10.0;
	for(double zTemp=mz-3*sz; zTemp < mz+3.1*sz; zTemp += interval )
	{
		velTemp.inject(solveVel(zTemp));
		double score = scoreFunc(velTemp, zTemp);
		if(score > scoreBest)
		{
			scoreBest = score;
			velBest.inject(velTemp);
			zBest = zTemp;
		}
	}

	///////////////////////////////////////////////////////////////
	// Line search to find the optimum
	///////////////////////////////////////////////////////////////
	double zL= zBest-interval;
	double zR= zBest+interval;
	Array2D<double> velL = solveVel(zL);
	Array2D<double> velR = solveVel(zR);
	Array2D<double> vel1, vel2;
	double z1, z2;
	double score1, score2;
	double offset;
	double range = zR-zL;
	while(range > 1e-3)
	{
		offset = 0.618*range;
		z1 = zR-offset;
		z2 = zL-offset;
		vel1 = solveVel(z1);
		vel2 = solveVel(z2);
		score1 = scoreFunc(vel1, z1);
		score2 = scoreFunc(vel2, z2);
		if( score1 < score2 ) // keep right region
		{
			zL = z1;
			velL = vel1;
		}
		else
		{
			zR = z2;
			velR = vel2;
		}

		range = zR-zL;
	}

	velMAP = 0.5*(velL+velR);
	heightMAP = 0.5*(zL+zR);
}
