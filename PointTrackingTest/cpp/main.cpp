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

void matchify(const std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &curPoints,
			  std::vector<ICSL::Quadrotor::PointMatch> &goodMatches,
			  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &repeatPoints,
			  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &newPoints,
			  const TNT::Array2D<double> Sn,
			  const TNT::Array2D<double> SnInv,
			  double probNoCorr,
			  const ICSL::Quadrotor::Time &imageTime,
			  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &mTrackedPoints);

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
			startImg += 500;
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

	vector<shared_ptr<TrackedPoint>> trackedPoints;

	Array2D<double> Sn = 5*5*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
	double probNoCorr = 1e-6;

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
	int sepDist = 50;
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

		// make objects
		vector<shared_ptr<TrackedPoint>> curPoints(points.size());
		for(int i=0; i<points.size(); i++)
		{
			curPoints[i] = shared_ptr<TrackedPoint>(new TrackedPoint(curTime, points[i]));
//			curPoints[i]->markFound(curTime, points[i]);
			curPoints[i]->setPosCov(Sn);
		}

//		/////////////////// Get location priors for active regions ///////////////////////
		Array2D<double> mv(3,1,0.0);
		Array2D<double> Sv = 0.1*0.1*createIdentity((double)3);
		double mz = 1;
		double sz = 0.05;
		Array2D<double> omega(3,1,0.0);

		for(int i=0; i<trackedPoints.size(); i++)
		{
			shared_ptr<TrackedPoint> tp = trackedPoints[i];
			tp->updatePositionDistribution(mv, Sv, mz, sz*sz, f, center, omega, curTime);
		}

		/////////////////// make matches ///////////////////////
		vector<PointMatch> goodMatches;
		vector<shared_ptr<TrackedPoint>> repeatPoints, newPoints;
		matchify(curPoints, goodMatches, repeatPoints, newPoints, Sn, SnInv, probNoCorr, curTime, trackedPoints);
		activeCnt += trackedPoints.size();


		imshow("chad",oldImg);

		cv::Mat dblImg(img.rows, 2*img.cols, img.type());
		oldImg.copyTo(dblImg(cv::Rect(0,0,oldImg.cols,oldImg.rows)));

		for(int i=0; i<points.size(); i++)
			circle(img, points[i], 4, cv::Scalar(255,0,0), -1);

		img.copyTo(oldImg);
		imshow("bob",img);

		vector<vector<cv::Point2f>> repeatPts(repeatPoints.size());
		for(int i=0; i<repeatPoints.size(); i++)
			circle(img, repeatPoints[i]->getPos(), 4, cv::Scalar(0,0,255), -1);

//		stringstream name;
//		name << imgDir << "/annotated/img_" << imgCnt << ".bmp";
//		imwrite(name.str().c_str(),img);

		img.copyTo(dblImg(cv::Rect(oldImg.cols,0,img.cols,img.rows)));
		cv::Point2f offset(321,0);
		for(int i=0; i<goodMatches.size(); i++)
		{
			const vector<pair<Time, cv::Point2f>> history = goodMatches[i].tpPrev->getHistory();
			vector<pair<Time, cv::Point2f>>::const_iterator iter = history.end();
			iter--;
			cv::Point2f p2 = iter->second;
			iter--;
			cv::Point2f p1 = iter->second;
			
			line(dblImg,p1, p2+offset, cv::Scalar(0,255,0), 2);
		}
		imshow("tom",dblImg);

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

void matchify(const std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &curPoints,
			  std::vector<ICSL::Quadrotor::PointMatch> &goodMatches,
			  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &repeatPoints,
			  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &newPoints,
			  const TNT::Array2D<double> Sn,
			  const TNT::Array2D<double> SnInv,
			  double probNoCorr,
			  const ICSL::Quadrotor::Time &imageTime,
			  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedPoint>> &mTrackedPoints)
{
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
	using namespace TNT;
	using namespace std;
	/////////////////// Establish correspondence based on postiion ///////////////////////

	Array2D<double> C = TrackedPoint::calcCorrespondence(mTrackedPoints, curPoints, Sn, SnInv, probNoCorr);
printArray("C:\n",C);

	///////////////////  make matches ///////////////////////
	shared_ptr<TrackedPoint> tpPrev, tpCur;
	int N1 = mTrackedPoints.size();
	int N2 = curPoints.size();
	vector<bool> prevMatched(N1, false);
	vector<bool> curMatched(N2, false);
	vector<cv::Point2f> offsets;
	for(int i=0; i<N1; i++)
	{
		if(N2 == 0 || C[i][N2] > 0.4)
			continue; // this object probably doesn't have a partner

		tpPrev = mTrackedPoints[i];

		int maxIndex = 0;
		float maxScore = 0;
		for(int j=0; j<N2; j++)
		{
			if(C[i][j] > maxScore && !curMatched[j])
			{
				maxScore = C[i][j];
				maxIndex =j;
			}
		}

		if(maxScore < 0.5)
			continue;

		curMatched[maxIndex] = true;
		tpCur = curPoints[maxIndex];

		cv::Point offset = tpCur->getPos()-tpPrev->getPos();
		offsets.push_back(offset);

		PointMatch m;
		m.tpPrev = tpPrev;
		m.tpCur = tpCur;
		m.score = C[i][maxIndex];
		goodMatches.push_back(m);
//		tpPrev->copyData(*tpCur);
		tpPrev->markFound(imageTime,tpCur->getPos());
//		tpPrev->addLife(2);
		repeatPoints.push_back(tpPrev);
		tpCur->kill();
	}

	vector<int> dupIndices;
	for(int j=0; j<curPoints.size(); j++)
		if(!curMatched[j])
		{
			// first check to see if we didn't match because there
			// were too many similar regions
			bool addMe = true;
			if(C.dim1() > 0 && C.dim2() > 0 && C[N1][j] < 0.5)
			{
				// yup, now we should clear out the riff raff
				for(int i=0; i<N1; i++)
					if(C[i][j] > 0.1)
					{
						if(prevMatched[i]) // the dupe already found a good match so we shouldn't delete him
							addMe = false;
						else
							dupIndices.push_back(i);
					}
			}
			// Now add the new region which will be the 
			// only remaining one
			if(addMe)
				newPoints.push_back(curPoints[j]);
			else
				curPoints[j]->kill();
		}

	// sort and remove repeats 
	sort(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator endIter = unique(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator iter = dupIndices.begin();
	while(iter != endIter)
	{
		mTrackedPoints[(*iter)]->kill();
		iter++;
	}

	for(int i=0; i<mTrackedPoints.size(); i++)
		mTrackedPoints[i]->takeLife(1);

	sort(mTrackedPoints.begin(), mTrackedPoints.end(), TrackedPoint::sortPredicate);
	while(mTrackedPoints.size() > 0 && !mTrackedPoints.back()->isAlive() )
		mTrackedPoints.pop_back();

	for(int i=0; i<newPoints.size(); i++)
		mTrackedPoints.push_back(newPoints[i]);
}
