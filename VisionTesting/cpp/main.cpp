#include <fstream>
#include <iostream>
#include <sstream>
#include <iterator>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "toadlet/egg.h"

#include "mser.h"

using namespace std;

void loadLog(String filename);
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

//	loadLog("../log.txt");

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
	int imgId = 1034;
	stringstream ss;
	cv::Mat img, imgGray;
	vector<vector<cv::Point> > regions, hulls;
	vector<cv::Point> hull;
	while(keypress != (int)'q' && imgID < 2612)
	{
		ss.str("");
		ss << "img_" << imgId++ << ".bmp";
		img = cv::imread(imgDir+"/"+ss.str());
		cout << imgDir << "/" << ss.str();
		if(img.data != NULL)
		{
			cv::cvtColor(img, imgGray, CV_BGR2GRAY);
			cout << endl;
			hulls.clear();
			mserDetector(imgGray, regions);
			cout << "\t" << regions.size() << " regions found " << endl;
			for(int i=0; i<regions.size(); i++)
			{
				cv::convexHull(regions[i], hull);
				hulls.push_back(hull);
			}

			cv::drawContours(img, hulls, -1, cv::Scalar(0,0,255), 2);
			imshow("chad",img);

			keypress = cv::waitKey() % 256;
		}
		else
			cout << "*" << endl;
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

void loadLog(String filename)
{
	string line;
	ifstream file(filename.c_str());
	if(file.is_open())
	{
		getline(file, line); // first line is a throw-away
		vector<string> tokens;
		while(file.good())
//		for(int i=0; i<10; i++)
		{
			getline(file, line);
			stringstream ss(line);
			double time;
			int type;
			ss >> time >> type;
			time = time/1.0e3;
			switch(type)
			{
				case LOG_ID_IMAGE:
					{
						int imgID;
						ss >> imgID;
						cout << time << "\t" << type << imgID << endl;
					}
					break;
			}
		}

		file.close();
	}
	else
		cout << "Couldn't find " << filename.c_str() << endl;
}
