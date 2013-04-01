#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mser.h"

using namespace std;

int main(int argv, char* argc[])
{
	cout << "start chadding" << endl;

	string imgDir = "../images";

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
	int imgId = 200;
	stringstream ss;
	cv::Mat img, imgGray;
	vector<vector<cv::Point> > regions, hulls;
	vector<cv::Point> hull;
	while(keypress != (int)'q')
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
		}
		else
			cout << "*" << endl;
		keypress = cv::waitKey() % 256;
	}

    return 0;
}


