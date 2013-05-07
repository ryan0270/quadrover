#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>
#include <memory>
#include <vector>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "toadlet/egg.h"

#include "mser.h"
#include "../../Rover/cpp/Data.h"
#include "../../Rover/cpp/TNT/tnt.h"
#include "../../Rover/cpp/TNT_Utils.h"

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

	int delta = 5;
	int minArea = 10e3;
	int maxArea = 140400;
	double maxVariation = 0.25;
	double minDiversity = 0.2;
	int maxEvolution = 200;
	double areaThreshold = 1.01;
	double minMargin = 0.003;
	int edgeBlurSize = 5;
	ICSL::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity, maxEvolution, areaThreshold, minMargin, edgeBlurSize);

	int keypress = 0;
	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);
	int imgId = 1482;
	stringstream ss;
	cv::Mat img, imgGray;
	vector<vector<cv::Point> > regions, hulls;
	vector<cv::Point> hull;
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

			printArray("\terrcov:\t", errCov);

			cv::cvtColor(img, imgGray, CV_BGR2GRAY);
			hulls.clear();
			mserDetector(imgGray, regions);
			cout << "\t" << "Image time: " << t.getMS()/1.0e3 << endl;
			cout << "\t" << regions.size() << " regions found " << endl;
			for(int i=0; i<regions.size(); i++)
			{
				cv::convexHull(regions[i], hull);
				hulls.push_back(hull);
			}

			cv::drawContours(img, hulls, -1, cv::Scalar(0,0,255), 2);
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
