#include "funcs.h"

namespace ICSL {
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

vector<double> TimeKeeper::times;

vector<vector<cv::Point>> findContours(const cv::Mat &img)
{
	cv::Mat imgGray;
	cvtColor(img, imgGray, CV_BGR2GRAY);

	int delta = 5*2;
	int minArea = 1000;
	int maxArea = 0.5*240*320;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.4; // smaller increase the number of regions
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);
	vector<vector<cv::Point>> regions;
	mserDetector(imgGray, regions);

	// preallocate
	cv::Mat mask(img.rows,img.cols,CV_8UC1, cv::Scalar(0));

	/////////////////// Find contours ///////////////////////
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
		mask(boundRect) = cv::Scalar(0);
		uchar *row;
		int step = mask.step;
		for(int j=0; j<regions[i].size(); j++)
		{
			int x = regions[i][j].x;
			int y = regions[i][j].y;
			mask.at<uchar>(y,x) = 255;
		}

		cv::Canny(mask(boundRect), mask(boundRect), 50, 100, 5, true);

		vector<vector<cv::Point>> contours;
		cv::findContours(mask(boundRect), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, corner);

		for(int i=0; i<contours.size(); i++)
		{
			if(cv::contourArea(contours[i]) < minArea)
				continue;
			allContours.push_back(contours[i]);
		}
	}

	return allContours;
}

vector<shared_ptr<ActiveObject>> objectify(const vector<vector<cv::Point>> &contours,
										   const TNT::Array2D<double> Sn,
										   const TNT::Array2D<double> SnInv,
										   double varxi, double probNoCorr,
										   const Time &curTime)
{
	/////////////////// make objects of the new contours ///////////////////////
	vector<shared_ptr<ActiveObject>> curObjects;
	for(int i=0; i<contours.size(); i++)
	{
		shared_ptr<ActiveObject> ao1(new ActiveObject(contours[i]));
		ao1->lastFoundTime.setTime(curTime);
		ao1->posCov.inject(Sn);
		curObjects.push_back(ao1);
	}

	/////////////////// similarity check ///////////////////////
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

	return curObjects;
}

void matchify(vector<shared_ptr<ActiveObject>> &activeObjects,
			  const vector<shared_ptr<ActiveObject>> &curObjects,
			  vector<Match> &goodMatches,
			  vector<shared_ptr<ActiveObject>> &repeatObjects,
			  const TNT::Array2D<double> Sn,
			  const TNT::Array2D<double> SnInv,
			  double varxi, double probNoCorr,
			  const Time &curTime)
{
	/////////////////// Establish correspondence based on postiion ///////////////////////
	Array2D<double> C = ActiveObject::calcCorrespondence(activeObjects, curObjects, Sn, SnInv, varxi, probNoCorr);


	///////////////////  make matches ///////////////////////
	shared_ptr<ActiveObject> aoPrev, aoCur;
	vector<shared_ptr<ActiveObject>> newObjects;
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
		}

		matched[maxIndex] = true;
		aoCur = curObjects[maxIndex];

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

	for(int i=0; i<activeObjects.size(); i++)
		activeObjects[i]->life -= 1;

	sort(activeObjects.begin(), activeObjects.end(), ActiveObject::sortPredicate);
	while(activeObjects.size() > 0 && activeObjects.back()->life <= 0)
		activeObjects.pop_back();

	// TODO
	for(int i=0; i<newObjects.size(); i++)
		activeObjects.push_back(newObjects[i]);
}

}
