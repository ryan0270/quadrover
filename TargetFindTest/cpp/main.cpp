#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "Time.h"
#include "TargetFinder2.h"
#include "ActiveRegion.h"

int main(int argv, char* argc[])
{
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
	using namespace TNT;
	using namespace std;
	using toadlet::egg::Log;
	using toadlet::egg::String;
	cout << "start chadding" << endl;

	string dataDir;
	int dataSet = 4;
	int startImg=0, endImg=0;
	switch(dataSet)
	{
		case 0:
			dataDir = "../dataSets/Sep8";
			startImg = 6895;
			endImg = 9307;
			break;
		case 1:
			dataDir = "../dataSets/Sep12";
			startImg = 3702;
			endImg = 5713;
			break;
		case 2:
			dataDir = "../dataSets/Sep19";
			startImg = 971;
			startImg = 1360;
			endImg = 2874;
			break;
		case 3:
			dataDir = "../dataSets/Sep23";
			startImg = 3286;
			endImg = 5954;
			break;
		case 4:
			dataDir = "../dataSets/Oct3_2";
			startImg = 989;
//			startImg += 150;
			endImg = 3850;
			break;
	}

	string imgDir;
	imgDir = dataDir + "/video";

	vector<pair<int, Time>> imgIdList;
	// preload all images
	list<pair<int, shared_ptr<cv::Mat>>> imgList;
	int imgId = startImg;
	int numImages;
	numImages = endImg-startImg;;
	for(int i=0; i<numImages; i++)
	{
		cv::Mat img;
		while(img.data == NULL)
		{
			stringstream ss;
			ss << "image_" << ++imgId << ".bmp";
			img = cv::imread(imgDir+"/"+ss.str());
		}

		shared_ptr<cv::Mat> pImg(new cv::Mat);
		img.copyTo(*pImg);

		imgList.push_back(pair<int, shared_ptr<cv::Mat>>(imgId, pImg));
	}

	// Camera calibration
	cv::Point2f center;
	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;
	cv::FileStorage fs;
	string filename = dataDir + "/s3Calib_640x480.yml";
	fs.open(filename.c_str(), cv::FileStorage::READ);
	if( fs.isOpened() )
	{
		mCameraMatrix_640x480 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraDistortionCoeffs = shared_ptr<cv::Mat>(new cv::Mat());

		fs["camera_matrix"] >> *mCameraMatrix_640x480;
		fs["distortion_coefficients"] >> *mCameraDistortionCoeffs;
		cout << "Camera calib loaded from " << filename.c_str() << endl;

		mCameraMatrix_320x240 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraMatrix_640x480->copyTo( *mCameraMatrix_320x240 );
		(*mCameraMatrix_320x240) = (*mCameraMatrix_320x240)*0.5;

		center.x = mCameraMatrix_320x240->at<double>(0,2);
		center.y = mCameraMatrix_320x240->at<double>(1,2);
	}
	else
	cout << "Failed to open " <<  filename.c_str();
	fs.release();

	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);
	cv::namedWindow("bob",1);
	cv::moveWindow("bob",321,0);
	cv::namedWindow("tom",1);
	cv::moveWindow("tom",0,261);

	vector<shared_ptr<ActiveRegion>> activeRegions;

	Array2D<double> Sn = 10*10*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
	double varxi = pow(500,2);
	double probNoCorr = 0.0000001;

	TargetFinder2 targetFinder;

	int keypress = 0;
	list<pair<int, shared_ptr<cv::Mat>>>::const_iterator imgIter = imgList.begin();
	cv::Mat img(240,320, CV_8UC3), oldImg(240,320,CV_8UC3), imgGray;
	int activeCnt = 0;
	int imgCnt = 0;
	Time curTime;
	while(keypress != (int)'q' && imgIter != imgList.end())
	{
		curTime.addTimeMS(33);
		img = *(imgIter->second);
		cvtColor(img,imgGray,CV_BGR2GRAY);

Time start;
		vector<vector<cv::Point>> allContours = targetFinder.findContours(imgGray);

		vector<shared_ptr<ActiveRegion>> curRegions = targetFinder.objectify(allContours,Sn,SnInv,varxi,probNoCorr,curTime);

		/////////////////// Get location priors for active regions ///////////////////////
		Array2D<double> mv(3,1,0.0);
		Array2D<double> Sv = 0.2*0.2*createIdentity((double)3);
		double mz = 1;
		double sz = 0.05;
		double f = mCameraMatrix_640x480->at<double>(0,0);
		Array2D<double> omega(3,1,0.0);

		activeRegions = targetFinder.getActiveRegions();

		for(int i=0; i<activeRegions.size(); i++)
		{
			shared_ptr<ActiveRegion> ao = activeRegions[i];
			ao->updatePositionDistribution(mv, Sv, mz, sz*sz, f, center, omega, curTime);
		}

		/////////////////// make matches ///////////////////////
		vector<RegionMatch> goodMatches;
		vector<shared_ptr<ActiveRegion>> repeatRegions, newRegions;
		targetFinder.matchify(curRegions, goodMatches, repeatRegions, newRegions, Sn, SnInv, varxi, probNoCorr, curTime);
		activeRegions = targetFinder.getActiveRegions();
		activeCnt += activeRegions.size();


		imshow("chad",oldImg);

		cv::Mat dblImg(img.rows, 2*img.cols, img.type());
		oldImg.copyTo(dblImg(cv::Rect(0,0,oldImg.cols,oldImg.rows)));

		vector<vector<cv::Point>> curContours(curRegions.size());
		for(int i=0; i<curRegions.size(); i++)
			curContours[i] = curRegions[i]->getContour();
		cv::drawContours(img, curContours, -1, cv::Scalar(255,0,0), 2);
		img.copyTo(oldImg);

		/////////////////// draw principal axes ///////////////////////
		cv::Mat axisImg;
		img.copyTo(axisImg);
		for(int i=0; i<curRegions.size(); i++)
		{
			shared_ptr<ActiveRegion> obj = curRegions[i];
			cv::Point2f cen = obj->getLastFoundPos();
			cv::Point2f p1, p2;
			const Array2D<double> principalAxes = obj->getPrincipalAxes();
			const vector<double> principalAxesEigVal = obj->getPrincipalAxesEigVal();
			double ratio = principalAxesEigVal[0]/principalAxesEigVal[1];
			if(ratio > 2)
			{
				p1.x = cen.x + 10*principalAxes[0][0] * principalAxesEigVal[0];
				p1.y = cen.y + 10*principalAxes[1][0] * principalAxesEigVal[0];
				p2.x = cen.x + 10*principalAxes[0][1] * principalAxesEigVal[1];
				p2.y = cen.y + 10*principalAxes[1][1] * principalAxesEigVal[1];

				line(axisImg,cen,p1,cv::Scalar(0,255,255),1);
				line(axisImg,cen,p2,cv::Scalar(255,0,255),1);
			}
		}

		imshow("bob",axisImg);
		vector<vector<cv::Point>> repeatContours(repeatRegions.size());
		
		for(int i=0; i<repeatRegions.size(); i++)
			repeatContours[i] = repeatRegions[i]->getContour();
		cv::drawContours(img, repeatContours, -1, cv::Scalar(0,0,255), 2);

//		stringstream name;
//		name << imgDir << "/annotated/img_" << imgCnt << ".bmp";
//		imwrite(name.str().c_str(),img);

		img.copyTo(dblImg(cv::Rect(oldImg.cols,0,img.cols,img.rows)));
		cv::Point2f offset(321,0);
		for(int i=0; i<goodMatches.size(); i++)
			line(dblImg,goodMatches[i].aoPrev->getLastFoundPos(), goodMatches[i].aoCur->getLastFoundPos()+offset, cv::Scalar(0,255,0), 2);
		imshow("tom",dblImg);

		keypress = cv::waitKey(1) % 256;

		imgIter++;
		imgCnt++;
	}

//	for(int i=0; i<TimeKeeper::times.size(); i++)
//		if(TimeKeeper::times[i] != 0)
//			Log::alert(String()+"t" + i +":\t" + (TimeKeeper::times[i]/imgCnt));
//	Log::alert(String()+"avg algo time: " + TimeKeeper::sum(0,4)/imgCnt);
//	Log::alert(String()+"num regions:\t" + activeCnt/imgCnt);

    return 0;
}

