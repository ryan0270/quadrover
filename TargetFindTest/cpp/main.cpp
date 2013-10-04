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

	vector<shared_ptr<ActiveObject>> activeObjects;

	Array2D<double> Sn = 2*10*10*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
	double varxi = pow(500,2);
	float probNoCorr = 0.0000001;

	int keypress = 0;
	list<pair<int, shared_ptr<cv::Mat>>>::const_iterator imgIter = imgList.begin();
	cv::Mat img(240,320, CV_8UC3), oldImg(240,320,CV_8UC3);
	float t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
	t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = 0;
	int activeCnt = 0;
	int imgCnt = 0;
	Time curTime;
	while(keypress != (int)'q' && imgIter != imgList.end())
	{
//cout << "imgCnt: " << imgCnt <<  endl;
Time start;
		curTime.addTimeMS(33);

		img = *(imgIter->second);
		vector<vector<cv::Point>> allContours = findContours(img);

t3 += start.getElapsedTimeNS()/1.0e6; start.setTime();
		/////////////////// make objects of the new contours ///////////////////////
		vector<shared_ptr<ActiveObject>> curObjects;
		for(int i=0; i<allContours.size(); i++)
		{
			shared_ptr<ActiveObject> ao1(new ActiveObject(allContours[i]));
			ao1->lastFoundTime.setTime(curTime);
			ao1->posCov.inject(0.5*Sn);
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
			aoPrev->copyData(*aoCur);
			aoPrev->life= min((float)21, aoPrev->life+2);
			aoPrev->lastFoundTime.setTime(curTime);
			repeatObjects.push_back(aoPrev);
		}
		
		for(int j=0; j<curObjects.size(); j++)
			if(!matched[j])
				newObjects.push_back(curObjects[j]);

t4 += start.getElapsedTimeNS()/1.0e6; start.setTime();
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

t5 += start.getElapsedTimeNS()/1.0e6; start.setTime();
		imshow("chad",oldImg);

		cv::Mat dblImg(img.rows, 2*img.cols, img.type());
		oldImg.copyTo(dblImg(cv::Rect(0,0,oldImg.cols,oldImg.rows)));

		cv::drawContours(img, allContours, -1, cv::Scalar(255,0,0), 2);
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

//t6 += start.getElapsedTimeNS()/1.0e6; start.setTime();
		keypress = cv::waitKey(0) % 256;

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

