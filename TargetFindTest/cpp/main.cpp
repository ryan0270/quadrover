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
//#include "mser.h"

int main(int argv, char* argc[])
{
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
//	using namespace ICSL::Constants;
	using namespace TNT;
	using namespace std;
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
//		cout << "Loading image " << i << ": " << imgId << endl;
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
//		cout << "\t" << "Focal length: " << mCameraMatrix_640x480->at<double>(0,0) << endl;
//		cout << "\t" << "centerX: " << mCameraMatrix_640x480->at<double>(0,2) << endl;
//		cout << "\t" << "centerY: " << mCameraMatrix_640x480->at<double>(1,2) << endl;

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

	// make the mser detector
	int delta = 5*2;
	int minArea = 1000;
	int maxArea = 0.5*320*240;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.4; // smaller increase the number of regions
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);

	vector<shared_ptr<ActiveObject>> activeObjects;

	Array2D<double> Sn = 2*10*10*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
	double varxi = pow(500,2);
	float probNoCorr = 0.0000001;

	int keypress = 0;
	list<pair<int, shared_ptr<cv::Mat>>>::const_iterator imgIter = imgList.begin();
	cv::Mat img(240,320, CV_8UC3), imgGray, oldImg(240,320,CV_8UC3);
	vector<vector<cv::Point>> regions;
	float t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
	t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = 0;
	int activeCnt = 0;
	int imgCnt = 0;
	Time curTime;
	cv::Mat mask(240,320,CV_8UC1, cv::Scalar(0));
	while(keypress != (int)'q' && imgIter != imgList.end())
	{
//cout << "imgCnt: " << imgCnt <<  endl;
		Time chadTime;
		curTime.addTimeMS(33);

		img = *(imgIter->second);
Time start;
		cvtColor(img, imgGray, CV_BGR2GRAY);

t1 += start.getElapsedTimeMS(); start.setTime();
		mserDetector(imgGray, regions);

t2 += start.getElapsedTimeMS(); start.setTime();
		/////////////////// Find contours ///////////////////////
		vector<vector<cv::Point>> allContours;
		int border = 2;
		for(int i=0; i<regions.size(); i++)
		{
Time inTime;
			cv::Rect boundRect = boundingRect( cv::Mat(regions[i]) );
			boundRect.x = max(0, boundRect.x-border);
			boundRect.y = max(0, boundRect.y-border);
			boundRect.width = min(img.cols, boundRect.x+boundRect.width+2*border)-boundRect.x;
			boundRect.height= min(img.rows, boundRect.y+boundRect.height+2*border)-boundRect.y;;
			cv::Point corner(boundRect.x, boundRect.y);
			mask(boundRect) = cv::Scalar(0);
			uchar *row;
			int step = mask.step;
			for(int j=0; j<regions[i].size(); j++)
			{
				int x = regions[i][j].x;
				int y = regions[i][j].y;
				mask.at<uchar>(y,x) = 255;
			}

t7 += inTime.getElapsedTimeNS()/1.0e6; inTime.setTime();

			cv::Canny(mask(boundRect), mask(boundRect), 50, 100, 5, true);

t8 += inTime.getElapsedTimeNS()/1.0e6; inTime.setTime();
			vector<vector<cv::Point>> contours;
			cv::findContours(mask(boundRect), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, corner);
//			cv::Mat mask2(img.size(), CV_8UC1, cv::Scalar(0));
//			drawContours(mask2, contours, -1, cv::Scalar(255));

t9 += inTime.getElapsedTimeNS()/1.0e6; inTime.setTime();

			for(int i=0; i<contours.size(); i++)
			{
				if(cv::contourArea(contours[i]) < minArea)
						continue;
//				approxPolyDP(cv::Mat(contours[i]), contours[i], 2, true);
				allContours.push_back(contours[i]);
			}
t10 += inTime.getElapsedTimeNS()/1.0e6; inTime.setTime();
		}

t3 += start.getElapsedTimeMS(); start.setTime();
		/////////////////// make objects of the new contours ///////////////////////
		vector<shared_ptr<ActiveObject>> curObjects;
		for(int i=0; i<allContours.size(); i++)
		{
			shared_ptr<ActiveObject> ao1(new ActiveObject(allContours[i]));
			ao1->lastFoundTime.setTime(curTime);
			ao1->posCov.inject(0.5*Sn);
			if(ao1->mom.m00 > minArea)
				curObjects.push_back(ao1);
		}

		/////////////////// self-similarity check ///////////////////////
		// Ideally, I would do this on all of the active objects at the end of the loop
		// but I still need to work out the math for that. Doing it here, I can take
		// advantage of everything having the same covariance
		// First reset all expected positions and covariances
		// now calculate the correspondence matrix
		Array2D<double> ssC = ActiveObject::calcCorrespondence(curObjects, curObjects, 0.5*Sn, 2.0*SnInv, varxi, 0);

		// Now keep only things that don't have confusion
		// if there is confusion, only keep one
		vector<shared_ptr<ActiveObject>> tempList;
		tempList.swap(curObjects);
		vector<bool> isStillGood(tempList.size(), true);
		for(int i=0; i<tempList.size(); i++)
		{
			if(!isStillGood[i])
				continue;
			vector<int> confusionList;
			for(int j=i+1; j<tempList.size(); j++)
				if(ssC[i][j] > 0.2)
					confusionList.push_back(j);

			for(int j=0; j<confusionList.size(); j++)
				isStillGood[confusionList[j]] = false;

			curObjects.push_back(tempList[i]);
		}

		/////////////////// Get location priors for active objects ///////////////////////
		Array2D<double> mv(3,1,0.0);
		Array2D<double> Sv = 0.2*0.2*createIdentity((double)3);
		double mz = 1;
		double sz = 0.05;
		double f = mCameraMatrix_640x480->at<double>(0,0);
		Array2D<double> omega(3,1,0.0);

//cout << "---------------- priors --------------------------" << endl;
		for(int i=0; i<activeObjects.size(); i++)
		{
			shared_ptr<ActiveObject> ao = activeObjects[i];
			ao->updatePosition(mv, Sv, mz, sz*sz, f, center, omega, curTime);
//cout << ao->id << "\t\t";
//cout << (int)(ao->expectedPos[0][0]+0.5) << " x " << (int)(ao->expectedPos[1][0]+0.5) << "\t\t";
//cout << (int)(ao->posCov[0][0]+0.5) << " x " << (int)(ao->posCov[1][1]+0.5) << "\t\t";
//cout << ao->mom.m00 << "\t\t";
//cout << endl;
		}

//cout << "---------------- current objects --------------------------" << endl;
//for(int i=0; i<curObjects.size(); i++)
//{
//	cout << curObjects[i]->id << "\t\t";
//	cout << curObjects[i]->lastCenter.x << " x " << curObjects[i]->lastCenter.y << "\t\t";
//	cout << curObjects[i]->mom.m00 << "\t\t";
//	cout << endl;
//}

		/////////////////// Establish correspondence based on postiion ///////////////////////
		Array2D<double> C = ActiveObject::calcCorrespondence(activeObjects, curObjects, Sn, SnInv, varxi, probNoCorr);
//cout << "---------------- correspondence --------------------------" << endl;
//printArray("\n",C);

		vector<Match> goodMatches;
		shared_ptr<ActiveObject> aoPrev, aoCur;
		vector<shared_ptr<ActiveObject>> repeatObjects, newObjects;
		int N1 = activeObjects.size();
		int N2 = curObjects.size();
		vector<bool> matched(N2, false);
		for(int i=0; i<N1; i++)
		{
			if(N2 == 0 || C[i][N2] > 0.5)
				continue; // this object probably doesn't have a partner

			aoPrev = activeObjects[i];

			int maxIndex = 0;
			float maxScore = 0;
			for(int j=0; j<N2; j++)
			{
				if(C[i][j] > maxScore && !matched[j])
				{
					maxScore = C[i][j];
					maxIndex =j;
				}
//				else if(C[i][j] > 0.3)
//				{
//					// I think this object is probably a duplicate (i.e.
//					// I found two very similar contours for the same
//					// thing) There may be some cases where this is not
//					// the best handling, though, but I'll dealing with
//					// those when they become an issue
//					matched[j] = true;
//				}
			}

			matched[maxIndex] = true;
			aoCur = curObjects[maxIndex];
//cout << "matching " << aoPrev->id << " and " << aoCur->id << endl;

			Match m;
			m.aoPrev = aoPrev;
			m.aoCur = aoCur;
			m.score = C[i][maxIndex];
			goodMatches.push_back(m);

			// copy data over to update the shape and position
			aoPrev->contour.swap(aoCur->contour);
			aoPrev->mom = aoCur->mom;
			aoPrev->lastCenter = aoCur->lastCenter;
			aoPrev->lastFoundTime.setTime(curTime);
			memcpy(aoPrev->huMom, aoCur->huMom, 7*sizeof(double));
			memcpy(aoPrev->centralMoms, aoCur->centralMoms, 7*sizeof(double));
			aoPrev->life= min((float)21, aoPrev->life+2);
			repeatObjects.push_back(aoPrev);
		}
		
		for(int j=0; j<curObjects.size(); j++)
			if(!matched[j])
				newObjects.push_back(curObjects[j]);

t4 += start.getElapsedTimeMS(); start.setTime();
		for(int i=0; i<activeObjects.size(); i++)
			activeObjects[i]->life -= 1;

		sort(activeObjects.begin(), activeObjects.end(), ActiveObject::sortPredicate);
		while(activeObjects.size() > 0 && activeObjects.back()->life <= 0)
			activeObjects.pop_back();

//		cout << " =======  LIFE ======== " << endl;
//		for(int i=0; i<activeObjects.size(); i++)
//			cout << activeObjects[i]->id << ": " <<  activeObjects[i]->life << endl;
//		cout << endl;

		// TODO
		for(int i=0; i<newObjects.size(); i++)
			activeObjects.push_back(newObjects[i]);

		activeCnt += activeObjects.size();

t5 += start.getElapsedTimeMS(); start.setTime();
		imshow("chad",oldImg);

		cv::Mat dblImg(img.rows, 2*img.cols, img.type());
		oldImg.copyTo(dblImg(cv::Rect(0,0,oldImg.cols,oldImg.rows)));

		cv::drawContours(img, allContours, -1, cv::Scalar(255,0,0), 2);
		img.copyTo(oldImg);

		imshow("bob",img);
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

//t6 += start.getElapsedTimeMS(); start.setTime();
		keypress = cv::waitKey(1) % 256;

		imgIter++;
		imgCnt++;
	}

	cout << "t1:\t" << t1/imgCnt << endl;
	cout << "t2:\t" << t2/imgCnt << endl;
	cout << "t3:\t" << t3/imgCnt << endl;
	cout << "t4:\t" << t4/imgCnt << endl;
	cout << "t5:\t" << t5/imgCnt << endl;
	cout << "t6:\t" << t6/imgCnt << endl;
	cout << "t7:\t" << t7/imgCnt << endl;
	cout << "t8:\t" << t8/imgCnt << endl;
	cout << "t9:\t" << t9/imgCnt << endl;
	cout << "t10:\t" << t10/imgCnt << endl;
	cout << "avg algo time: " << (t1+t2+t3+t4+t5)/imgCnt << endl;
	cout << "num objects:\t" << activeCnt/imgCnt << endl;

    return 0;
}

