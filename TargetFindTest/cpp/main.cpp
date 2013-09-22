#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Time.h"

//#include "mser.h"


class ActiveObject
{
	public:
	ActiveObject(){life = 10; this->id = sNextID()++;}
	ActiveObject(std::vector<cv::Point> points) : ActiveObject()
	{
		contour = points;
		mom = cv::moments(points);
		lastCenter.x = mom.m10/mom.m00;
		lastCenter.y = mom.m01/mom.m00;
		cv::HuMoments(mom, huMom);
		centralMoms[0] = mom.mu20;
		centralMoms[1] = mom.mu11;
		centralMoms[2] = mom.mu02;
		centralMoms[3] = mom.mu30;
		centralMoms[4] = mom.mu21;
		centralMoms[5] = mom.mu12;
//		centralMoms[6] = mom.mu03;
		centralMoms[6] = mom.m00;
	}

	cv::Point lastCenter;
	float life;
	std::vector<cv::Point> contour;
	cv::Moments mom;
	double huMom[7];
	double centralMoms[7];
	int id;

	static inline unsigned long &sNextID(){ static unsigned long id = 0; return id;}

	static bool sortPredicate(const std::shared_ptr<ActiveObject> &ao1, const std::shared_ptr<ActiveObject> &ao2)
	{ return ao1->life> ao2->life; }
};

class Match
{
	public:
	std::shared_ptr<ActiveObject> ao1, ao2;
	float score;
};

int main(int argv, char* argc[])
{
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
//	using namespace ICSL::Constants;
//	using namespace TNT;
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
		cout << "\n\t" << "Focal length: " << mCameraMatrix_640x480->at<double>(0,0) << endl;
		cout << "\n\t" << "centerX: " << mCameraMatrix_640x480->at<double>(0,2) << endl;
		cout << "\n\t" << "centerY: " << mCameraMatrix_640x480->at<double>(1,2) << endl;

		mCameraMatrix_320x240 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraMatrix_640x480->copyTo( *mCameraMatrix_320x240 );
		(*mCameraMatrix_320x240) = (*mCameraMatrix_320x240)*0.5;
	}
	else
	cout << "Failed to open " <<  filename.c_str();
	fs.release();

	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);
	cv::namedWindow("bob",1);
	cv::moveWindow("bob",321,0);
	cv::namedWindow("dick",1);
	cv::moveWindow("dick",0,261);

	// make the mser detector
	int delta = 5;
	int minArea = 1000;
	int maxArea = 0.5*320*240;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.5; // smaller increase the number of regions
	// for color only
	int maxEvolution = 200;
	double areaThreshold = 1.01;
	double minMargin = 0.003;
	int edgeBlurSize = 5;
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);

	vector<shared_ptr<ActiveObject>> activeObjects;

	int keypress = 0;
	list<pair<int, shared_ptr<cv::Mat>>>::const_iterator imgIter = imgList.begin();
	cv::Mat img(240,320, CV_8UC3), imgGray, oldImg(240,320,CV_8UC3);
	vector<vector<cv::Point>> regions;
	int t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;
	t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = 0;
	int maxCalcTime = 0;
	while(keypress != (int)'q' && imgIter != imgList.end())
	{
//		oldImg = img;
Time start;
		img = *(imgIter->second);
		cvtColor(img, imgGray, CV_BGR2GRAY);

t1 = start.getElapsedTimeMS(); start.setTime();
		mserDetector(imgGray, regions);

t2 = start.getElapsedTimeMS(); start.setTime();
		vector<vector<cv::Point>> allContours;
		int border = 2;
		for(int i=0; i<regions.size(); i++)
		{
			cv::Rect boundRect = boundingRect( cv::Mat(regions[i]) );
			boundRect.x = max(0, boundRect.x-border);
			boundRect.y = max(0, boundRect.y-border);
			boundRect.width = min(img.cols, boundRect.x+boundRect.width+2*border)-boundRect.x;
			boundRect.height= min(img.rows, boundRect.y+boundRect.height+2*border)-boundRect.y;;
			cv::Point corner(boundRect.x, boundRect.y);
			cv::Mat mask(boundRect.height, boundRect.width, CV_8UC1, cv::Scalar(0));
///			vector<vector<cv::Point>> chad(1);
///			for(int j=0; j<regions[i].size(); j++)
///				chad[0].push_back(regions[i][j]-corner);
///			cv::fillPoly(mask, chad, cv::Scalar(255));

			for(int j=0; j<regions[i].size(); j++)
			{
				int x = regions[i][j].x-corner.x;
				int y = regions[i][j].y-corner.y;
				mask.at<uchar>(y,x) = 255;
			}
//			imshow("mask",mask);
//			cv::waitKey(0);

			cv::dilate(mask, mask, cv::Mat());
			cv::erode(mask, mask, cv::Mat());

			cv::Canny(mask, mask, 0, 10, 3);

			vector<vector<cv::Point>> contours;
			cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, corner);
			cv::Mat mask2(img.size(), CV_8UC1, cv::Scalar(0));
			drawContours(mask2, contours, -1, cv::Scalar(255));

			for(int i=0; i<contours.size(); i++)
			{
				approxPolyDP(cv::Mat(contours[i]), contours[i], 2, true);
				if(contours[i].size() < 3 || cv::contourArea(contours[i]) < minArea)
					continue;
				allContours.push_back(contours[i]);
			}
		}

t3 = start.getElapsedTimeMS(); start.setTime();
		vector<shared_ptr<ActiveObject>> newObjects;
		vector<Match> possibleMatches;
		cv::Moments mom;
		double ma[7];
		for(int i=0; i<allContours.size(); i++)
		{
			shared_ptr<ActiveObject> ao1(new ActiveObject(allContours[i]));
			if(ao1->mom.m00 < minArea)
				continue;

			memcpy(ma, ao1->centralMoms, 7*sizeof(double));
			float lowScore = 999;
			float minDist = 999;
			int lowIdx = -1;
			bool noMatch = true;
			for(int j=0; j<activeObjects.size(); j++)
			{
				// based on opencv matchShapes
				shared_ptr<ActiveObject> ao2 = activeObjects[j];
				double c1, c2, c3;
				c1 = c2 = c3 = 0;
				int sma, smb;
				bool noAdd = true;
				for(int k=0; k<7; k++)
				{
					double ama = abs( ma[k] );
//					double amb = abs( ao2->huMom[k] );
					double amb = abs( ao2->centralMoms[k] );
					if(ama < 1.e-5 || amb < 1.e-5)
						continue;
					if(ma[k] > 0)
						sma = 1;
					else if(ma[k] < 0)
						sma = -1;
					else
						sma = 0;
//					if(ao2->huMom[k] > 0)
					if(ao2->centralMoms[k] > 0)
						smb = 1;
//					else if(ao2->huMom[k] < 0)
					else if(ao2->centralMoms[k] < 0)
						smb = -1;
					else
						smb = 0;

					noAdd = false;
					ama = sma*log10(ama);
					amb = smb*log10(amb);
					c1 += abs(-1./ama+1./amb);
					c2 += abs(-ama+amb);
					c3 = max(c3, abs((ama-amb)/ama));
				}

				float score = c1;
				float dist = cv::norm(cv::Mat(ao1->lastCenter), cv::Mat(ao2->lastCenter));

				minDist = min(dist,minDist);

				if(dist < 20)
				{
					lowScore = min(score, lowScore);

					if(score < 1)
					{
						noMatch = false;
						Match m;
						m.ao1 = ao1;
						m.ao2 = ao2;
						m.score = score;
						possibleMatches.push_back(m);
					}
				}
			}

			if(noMatch)
				newObjects.push_back(ao1);
		}

t4 = start.getElapsedTimeMS(); start.setTime();
		// now find the best possible matches and ensure unique matches
		sort(possibleMatches.begin(), possibleMatches.end(), [&](const Match &m1, const Match &m2){return m1.score < m2.score;});
		vector<shared_ptr<ActiveObject>> repeatObjects;
		vector<Match> goodMatches;
		for(int i=0; i<possibleMatches.size(); i++)
		{
			shared_ptr<ActiveObject> ao1 = possibleMatches[i].ao1;
			shared_ptr<ActiveObject> ao2 = possibleMatches[i].ao2;
			if(ao1->life< 0)
				continue;
			if(find(repeatObjects.begin(), repeatObjects.end(), ao2) != repeatObjects.end())
			{
				// this active object has already been used by someone else
				if(find(newObjects.begin(), newObjects.end(), ao1) != newObjects.end())
					newObjects.push_back(ao1);
			}
			else
			{
				goodMatches.push_back(possibleMatches[i]);
				ao2->contour.swap(ao1->contour);
				ao2->mom = ao1->mom;
				ao2->lastCenter = ao1->lastCenter;
				memcpy(ao2->huMom, ao1->huMom, 7*sizeof(double));
				memcpy(ao2->centralMoms, ao1->centralMoms, 7*sizeof(double));
				ao2->life= min((float)21, ao2->life+2);
				repeatObjects.push_back(ao2);
				ao1->life= -1; // to mark that this is no longer valid
			}
		}

t5 = start.getElapsedTimeMS(); start.setTime();
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

t6 = start.getElapsedTimeMS(); start.setTime();
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

		img.copyTo(dblImg(cv::Rect(oldImg.cols,0,img.cols,img.rows)));
		cv::Point offset(321,0);
		for(int i=0; i<goodMatches.size(); i++)
			line(dblImg,goodMatches[i].ao1->lastCenter, goodMatches[i].ao2->lastCenter+offset, cv::Scalar(0,255,0), 2);
		imshow("dick",dblImg);

t7 = start.getElapsedTimeMS(); start.setTime();
		keypress = cv::waitKey(33) % 256;

		maxCalcTime = max(maxCalcTime, t1+t2+t3+t4+t5+t6+t7+t8+t9+t10);

		imgIter++;
	}

	cout << "maxCalcTime: " << maxCalcTime << endl;

    return 0;
}

