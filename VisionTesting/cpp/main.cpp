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
#include "../../Rover/cpp/TNT_Utils.h"
#include "../../Rover/cpp/Time.h"
#include "../../Rover/cpp/constants.h"

using namespace std;
using namespace ICSL::Quadrotor;
using namespace TNT;

void loadLog(String filename, 
				vector<pair<int, Time> > &imgIdList,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList,
				list<shared_ptr<DataVector> > &errCovList
				);
vector<string> tokenize(string str);

vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
													Array2D<double> const &mv, Array2D<double> const &Sv, 
													double const &mz, double const &varz, 
													double const &focalLength, double const &dt);
inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}

enum LogIDs
{
	LOG_ID_ACCEL = 1,
	LOG_ID_GYRO = 2,
	LOG_ID_MAGNOMETER = 3,
	LOG_ID_PRESSURE = 4,
	LOG_ID_IMAGE = 10,
	LOG_ID_PHONE_TEMP = 500,
	LOG_ID_CPU_USAGE = -2000,
	LOG_ID_TIME_SYNC = -500,
	LOG_ID_GYRO_BIAS = -1003,
	LOG_ID_OBSV_ANG_INNOVATION = -1004,
	LOG_ID_SET_YAW_ZERO = -805,
	LOG_ID_OBSV_ANG_RESET = -200,
	LOG_ID_OBSV_ANG_GAINS_UPDATED = -210,
	LOG_ID_OPTIC_FLOW = 12345,
	LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS = 12346,
	LOG_ID_OPTIC_FLOW_LS = 123457,
	LOG_ID_OBSV_TRANS_ATT_BIAS = -710,
	LOG_ID_OBSV_TRANS_FORCE_GAIN = -711,
	LOG_ID_BAROMETER_HEIGHT = 1234,
	LOG_ID_MOTOR_CMDS = -1000,
	LOG_ID_DES_ATT = -1001,
	LOG_ID_CUR_ATT = -1002,
	LOG_ID_DES_TRANS_STATE = -1011,
	LOG_ID_CUR_TRANS_STATE = -1012,
	LOG_ID_IMG_PROC_TIME_FEATURE_MATCH = -600,
	LOG_ID_IMG_PROC_TIME_TARGET_FIND = -601,
	LOG_ID_IBVS_ENABLED = -605,
	LOG_ID_IBVS_DISABLED = -606,
	LOG_ID_RECEIVE_VICON = 700,
	LOG_ID_CAMERA_POS = 800,
	LOG_ID_KALMAN_ERR_COV = -720,
	LOG_ID_NUM_FEATURE_POINTS = 1300,
	LOG_ID_IMG_TARGET_POINTS = 1310,
	LOG_ID_OBSV_TRANS_PROC_TIME = 10000,
};

int main(int argv, char* argc[])
{
	cout << "start chadding" << endl;

	string imgDir = "../video";

	vector<pair<int, Time> > imgIdList;
	list<shared_ptr<DataVector> > attDataList, transDataList, errCovList;
	loadLog("../log.txt", imgIdList, attDataList, transDataList, errCovList);
	if(imgIdList.size() == 0)
	{
		cout << "Failed loading data" << endl;
		return 0;
	}

//	int delta = 5;
//	int minArea = 1e3;
//	int maxArea = 640*480/5;
//	double maxVariation = 0.25;
//	double minDiversity = 0.2;
//	int maxEvolution = 200;
//	double areaThreshold = 1.01;
//	double minMargin = 0.003;
//	int edgeBlurSize = 5;
//	ICSL::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity, maxEvolution, areaThreshold, minMargin, edgeBlurSize);

	int keypress = 0;
	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);
//	int imgId = 1482;
	int imgId = 1700;
	stringstream ss;
	cv::Mat img, imgGray;
	vector<vector<cv::Point> > regions, hulls;
	vector<cv::Point> hull;
	vector<cv::Vec4i> heirarchy;
	int imgIdx = 0;
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
			Time t = imgIdList[imgIdx].second;
			Array2D<double> attState  = Data::interpolate(t, attDataList);
			Array2D<double> transState = Data::interpolate(t, transDataList);
			Array2D<double> errCov = Data::interpolate(t, errCovList);

			Time start;

//			cv::cvtColor(img, imgGray, CV_BGR2GRAY);
//			hulls.clear();
//			mserDetector(imgGray, regions);
//			cout << "\t" << regions.size() << " regions found " << endl;
//			for(int i=0; i<regions.size(); i++)
//			{
//				cv::convexHull(regions[i], hull);
//				hulls.push_back(hull);
//			}
//
//			cv::drawContours(img, hulls, -1, cv::Scalar(0,0,255), 2);

//			vector<cv::KeyPoint> kpList;
//			cv::FAST(img, kpList, 50);
//			cout << "\t" << kpList.size() << " keypoints found" << endl;
//			cv::drawKeypoints(img, kpList, img, cv::Scalar(0,0,255));

			cv::Mat imgGray;
			cv::cvtColor(img, imgGray, CV_BGR2GRAY);
			vector<cv::Point2f> corners;
			int maxCorners = 100;
			double qualityLevel = 0.05;
			double minDistance = 10;
			cv::goodFeaturesToTrack(imgGray, corners, maxCorners, qualityLevel, minDistance);
			cout << "\t" << "Proc time: " << start.getElapsedTimeNS()/1.0e9 << endl;
			cout << "\t" << corners.size() << " corners found" << endl;
			for(int i=0; i<corners.size(); i++)
				cv::circle(img, corners[i], 4, cv::Scalar(0,0,255), -1);

			imshow("chad",img);

			start.setTime();
			Array2D<double> mv(3,1,0.0);
			mv[1][0] = 0.2;
			Array2D<double> Sv = 0.2*createIdentity(3);
			Sv[0][0] = Sv[1][1] = 0.1*0.1;
			Sv[2][2] = 0.01*0.01;
			double mz = 0.12;
			double sz = 0.04;
			double focalLength = 3.7*640/5.76;
			double dt = 0.1;
			calcPriorDistributions(corners, mv, Sv, mz, sz*sz, focalLength, dt);
			cout << "\tVel calc time: " << start.getElapsedTimeNS()/1.0e9 << endl;

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

void loadLog(String filename, 
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

vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
							Array2D<double> const &mv, Array2D<double> const &Sv, 
							double const &mz, double const &varz, 
							double const &focalLength, double const &dt)
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

		Sd[0][0] = (pow(mDelta[0][0],2)+SDelta[0][0])*mz2Inv-pow(mDelta[0][0],2)*pow(mz1Inv,2);
		Sd[1][1] = (pow(mDelta[1][0],2)+SDelta[1][1])*mz2Inv-pow(mDelta[1][0],2)*pow(mz1Inv,2);

		double x = points[i].x;
		double y = points[i].y;
		Sd[0][1] = Sd[1][0] = x*y*pow(dt,2)*pow(svz,2)*mz2Inv + mDelta[0][0]*mDelta[1][0]*(mz2Inv-pow(mz1Inv,2));

		dDistList.push_back(pair<Array2D<double>, Array2D<double> >(md, Sd));
	}

	return dDistList;
}

Array2D<double> calcCorrespondence(vector<pair<Array2D<double>, Array2D<double> > > const &priorDistList, vector<cv::Point2f> const &curPointList, Array2D<double> const &Sn, Array2D<double> const &SnInv)
{
	int N1 = priorDistList.size();
	int N2 = curPointList.size();
	Array2D<double> C(N1+1, N2+1, 0.0);
	double x, y;
	Array2D<double> mq(2,1), md(2,1);
	Array2D<double> Sd, SdInv, Sa, SaInv;
	Array2D<double> eye2 = createIdentity(2);
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
			double xrange = 5*sqrt(Sd[0][0]+Sn[0][0]);
			double yrange = 5*sqrt(Sd[1][1]+Sn[1][1]);
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
		}
	}

	return C;
}
