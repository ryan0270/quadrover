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

#include "Time.h"

#include "ActiveObject.h"
#include "funcs.h"

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
			dataDir = "../dataSets/Sep19";
//			startImg = 971;
			startImg = 1360;
			endImg = 2874;
			endImg = 2600;
			endImg = startImg+200;
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

	vector<shared_ptr<ActiveObject>> activeObjects;

	Array2D<double> Sn = 10*10*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
	double varxi = pow(500,2);
	double probNoCorr = 0.0000001;

	int keypress = 0;
	list<pair<int, shared_ptr<cv::Mat>>>::const_iterator imgIter = imgList.begin();
	cv::Mat img(240,320, CV_8UC3), oldImg(240,320,CV_8UC3);
	TimeKeeper::times.resize(100);
	for(int i=0; i<TimeKeeper::times.size(); i++)
		TimeKeeper::times[i] = 0;
	int activeCnt = 0;
	int imgCnt = 0;
	Time curTime;
	while(keypress != (int)'q' && imgIter != imgList.end())
	{
		curTime.addTimeMS(33);
		img = *(imgIter->second);

Time start;
		vector<vector<cv::Point>> allContours = findContours(img);

TimeKeeper::times[0] += start.getElapsedTimeNS()/1.0e6; start.setTime();
		
		vector<shared_ptr<ActiveObject>> curObjects = objectify(allContours,Sn,SnInv,varxi,probNoCorr,curTime);

TimeKeeper::times[1] += start.getElapsedTimeNS()/1.0e6; start.setTime();
		/////////////////// Get location priors for active objects ///////////////////////
		Array2D<double> mv(3,1,0.0);
		Array2D<double> Sv = 0.2*0.2*createIdentity((double)3);
		double mz = 1;
		double sz = 0.05;
		double f = mCameraMatrix_640x480->at<double>(0,0);
		Array2D<double> omega(3,1,0.0);

		for(int i=0; i<activeObjects.size(); i++)
		{
			shared_ptr<ActiveObject> ao = activeObjects[i];
			ao->updatePosition(mv, Sv, mz, sz*sz, f, center, omega, curTime);
		}

TimeKeeper::times[2] += start.getElapsedTimeNS()/1.0e6; start.setTime();
		/////////////////// make matches ///////////////////////
		vector<Match> goodMatches;
		vector<shared_ptr<ActiveObject>> repeatObjects;
		matchify(activeObjects, curObjects, goodMatches, repeatObjects, Sn, SnInv, varxi, probNoCorr, curTime);
		activeCnt += activeObjects.size();

TimeKeeper::times[3] += start.getElapsedTimeNS()/1.0e6; start.setTime();

		imshow("chad",oldImg);

		cv::Mat dblImg(img.rows, 2*img.cols, img.type());
		oldImg.copyTo(dblImg(cv::Rect(0,0,oldImg.cols,oldImg.rows)));

		vector<vector<cv::Point>> curContours(curObjects.size());
		for(int i=0; i<curObjects.size(); i++)
			curContours[i] = curObjects[i]->contour;
		cv::drawContours(img, curContours, -1, cv::Scalar(255,0,0), 2);
		img.copyTo(oldImg);

		/////////////////// draw principal axes ///////////////////////
		cv::Mat axisImg;
		img.copyTo(axisImg);
		for(int i=0; i<curObjects.size(); i++)
		{
			shared_ptr<ActiveObject> obj = curObjects[i];
			cv::Point2f cen = obj->lastCenter;
			cv::Point2f p1, p2;
			p1.x = cen.x + 10*obj->principalAxes[0][0] * obj->principalAxesEigVal[0];
			p1.y = cen.y + 10*obj->principalAxes[1][0] * obj->principalAxesEigVal[0];
			p2.x = cen.x + 10*obj->principalAxes[0][1] * obj->principalAxesEigVal[1];
			p2.y = cen.y + 10*obj->principalAxes[1][1] * obj->principalAxesEigVal[1];

			line(axisImg,cen,p1,cv::Scalar(0,255,255),1);
			line(axisImg,cen,p2,cv::Scalar(255,0,255),1);
		}

		imshow("bob",axisImg);
		vector<vector<cv::Point>> repeatContours(repeatObjects.size());
		
		for(int i=0; i<repeatObjects.size(); i++)
			repeatContours[i] = repeatObjects[i]->contour;
		cv::drawContours(img, repeatContours, -1, cv::Scalar(0,0,255), 2);

//		stringstream name;
//		name << imgDir << "/annotated/img_" << imgCnt << ".bmp";
//		imwrite(name.str().c_str(),img);

		img.copyTo(dblImg(cv::Rect(oldImg.cols,0,img.cols,img.rows)));
		cv::Point offset(321,0);
		for(int i=0; i<goodMatches.size(); i++)
			line(dblImg,goodMatches[i].aoPrev->lastCenter, goodMatches[i].aoCur->lastCenter+offset, cv::Scalar(0,255,0), 2);
		imshow("tom",dblImg);

		keypress = cv::waitKey(1) % 256;

		imgIter++;
		imgCnt++;
	}

	for(int i=0; i<TimeKeeper::times.size(); i++)
		if(TimeKeeper::times[i] != 0)
			Log::alert(String()+"t" + i +":\t" + (TimeKeeper::times[i]/imgCnt));
	Log::alert(String()+"avg algo time: " + TimeKeeper::sum(0,4)/imgCnt);
	Log::alert(String()+"num objects:\t" + activeCnt/imgCnt);

    return 0;
}

