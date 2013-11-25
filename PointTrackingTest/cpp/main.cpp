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
#include "FeatureFinder.h"
#include "TrackedPoint.h"

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
	int dataSet = 0;
	int startImg=0, endImg=0;
	switch(dataSet)
	{
		case 0:
			dataDir = "../dataSets/Nov13_3";
			startImg = 4686;
			endImg = 7486;
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
			ss << "image_" << imgId++ << ".bmp";
			string filename = imgDir+"/"+ss.str();
			img = cv::imread(filename);
			if(img.data == NULL)
				cout << "Failed to load " << filename << endl;
		}

		shared_ptr<cv::Mat> pImg(new cv::Mat);
		img.copyTo(*pImg);

		imgList.push_back(pair<int, shared_ptr<cv::Mat>>(imgId, pImg));
	}

	// Camera calibration
	double f;
	cv::Point2f center;
	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;
	cv::FileStorage fs;
	string filename = dataDir + "/calib_640x480.yml";
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

		f = mCameraMatrix_320x240->at<double>(0,0);
		center.x = mCameraMatrix_320x240->at<double>(0,2);
		center.y = mCameraMatrix_320x240->at<double>(1,2);

		cout << "Loaded camera matrix" << endl;
	}
	else
		cout << "Failed to open " <<  filename.c_str();
	fs.release();
	cv::Mat cameraMatrix = *mCameraMatrix_320x240;
	cv::Mat distCoeffs = *mCameraDistortionCoeffs;

	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);
	cv::namedWindow("bob",1);
	cv::moveWindow("bob",321,0);
	cv::namedWindow("tom",1);
	cv::moveWindow("tom",0,261);

//	vector<shared_ptr<ActiveRegion>> activeRegions;
//
//	Array2D<double> Sn = 10*10*createIdentity((double)2);
//	Array2D<double> SnInv(2,2,0.0);
//	SnInv[0][0] = 1.0/Sn[0][0];
//	SnInv[1][1] = 1.0/Sn[1][1];
////	double varxi = pow(500,2);
//	double varxi_ratio = 0.1;
//	double probNoCorr = 0.0000001;

	FeatureFinder featureFinder;
	featureFinder.initialize();

	int keypress = 0;
	list<pair<int, shared_ptr<cv::Mat>>>::const_iterator imgIter = imgList.begin();
	cv::Mat img(240,320, CV_8UC3, cv::Scalar(0)), oldImg(240,320,CV_8UC3,cv::Scalar(0)), imgGray;
	int activeCnt = 0;
	int imgCnt = 0;
	Time curTime;
	vector<cv::Point2f> points;
	float qualityLevel = 0.05;
	float fastThresh = 30;
	int sepDist = 10;
	float pointCntTarget = 30;
	float fastAdaptRate = 0.01;
	while(keypress != (int)'q' && imgIter != imgList.end())
	{
//Log::alert(String()+"imgCnt: "+imgCnt);
		curTime.addTimeMS(33);
		img = *(imgIter->second);
		cvtColor(img,imgGray,CV_BGR2GRAY);

Time start;
		points = FeatureFinder::findFeaturePoints(imgGray, qualityLevel, sepDist, fastThresh);
		if(points.size() > 0)
		{
			cv::undistortPoints(points, points, cameraMatrix, distCoeffs);
			for(int i=0; i<points.size(); i++)
				points[i] = points[i]*f+center;
		}
		
//		vector<vector<cv::Point>> allContours = targetFinder.findContours(imgGray);
//
//		vector<shared_ptr<ActiveRegion>> curRegions = targetFinder.objectify(allContours,Sn,SnInv,varxi_ratio,probNoCorr,curTime);
//
//		/////////////////// Get location priors for active regions ///////////////////////
//		Array2D<double> mv(3,1,0.0);
//		Array2D<double> Sv = 0.1*0.1*createIdentity((double)3);
//		double mz = 1;
//		double sz = 0.05;
//		double f = mCameraMatrix_640x480->at<double>(0,0);
//		Array2D<double> omega(3,1,0.0);
//
//		activeRegions = targetFinder.getActiveRegions();
//
//		for(int i=0; i<activeRegions.size(); i++)
//		{
//			shared_ptr<ActiveRegion> ao = activeRegions[i];
//			ao->updatePositionDistribution(mv, Sv, mz, sz*sz, f, center, omega, curTime);
//		}
//
//		/////////////////// make matches ///////////////////////
//		vector<RegionMatch> goodMatches;
//		vector<shared_ptr<ActiveRegion>> repeatRegions, newRegions;
//		targetFinder.matchify(curRegions, goodMatches, repeatRegions, newRegions, Sn, SnInv, varxi_ratio, probNoCorr, curTime);
//		activeRegions = targetFinder.getActiveRegions();
//		activeCnt += activeRegions.size();
//
//
		imshow("chad",oldImg);

//		cv::Mat dblImg(img.rows, 2*img.cols, img.type());
//		oldImg.copyTo(dblImg(cv::Rect(0,0,oldImg.cols,oldImg.rows)));

		for(int i=0; i<points.size(); i++)
		{
			if(imgCnt % 2 == 0)
			circle(img, points[i], 3, cv::Scalar(0,255,0), -1);
			else
			circle(img, points[i], 3, cv::Scalar(255,0,0), -1);
		}
//
//		vector<vector<cv::Point>> curContours(curRegions.size());
//		for(int i=0; i<curRegions.size(); i++)
//			curContours[i] = curRegions[i]->getContour();
//		cv::drawContours(img, curContours, -1, cv::Scalar(255,0,0), 2);
		img.copyTo(oldImg);
		imshow("bob",img);

//		/////////////////// draw principal axes ///////////////////////
//		cv::Mat axisImg;
//		img.copyTo(axisImg);
//		targetFinder.drawTarget(axisImg, curRegions, repeatRegions); 
//
//		imshow("bob",axisImg);
//		vector<vector<cv::Point>> repeatContours(repeatRegions.size());
//		
//		for(int i=0; i<repeatRegions.size(); i++)
//			repeatContours[i] = repeatRegions[i]->getContour();
//		cv::drawContours(img, repeatContours, -1, cv::Scalar(0,0,255), 2);
//
//		stringstream name;
//		name << imgDir << "/annotated/img_" << imgCnt << ".bmp";
//		imwrite(name.str().c_str(),img);
//
//		img.copyTo(dblImg(cv::Rect(oldImg.cols,0,img.cols,img.rows)));
//		cv::Point2f offset(321,0);
//		for(int i=0; i<goodMatches.size(); i++)
//			line(dblImg,goodMatches[i].aoPrev->getPrevFoundPos(), goodMatches[i].aoCur->getFoundPos()+offset, cv::Scalar(0,255,0), 2);
//		imshow("tom",dblImg);
//
		keypress = cv::waitKey(0) % 256;

		imgIter++;
		imgCnt++;
	}
//
////	for(int i=0; i<TimeKeeper::times.size(); i++)
////		if(TimeKeeper::times[i] != 0)
////			Log::alert(String()+"t" + i +":\t" + (TimeKeeper::times[i]/imgCnt));
////	Log::alert(String()+"avg algo time: " + TimeKeeper::sum(0,4)/imgCnt);
////	Log::alert(String()+"num regions:\t" + activeCnt/imgCnt);

    return 0;
}

