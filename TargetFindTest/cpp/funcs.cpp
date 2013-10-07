#include "funcs.h"

namespace ICSL {
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;
using toadlet::egg::Log;
using toadlet::egg::String;

vector<double> TimeKeeper::times;

vector<vector<cv::Point>> findContours(const cv::Mat &image)
{
	cv::Mat imageGray;
	if(image.channels() == 3)
		cvtColor(image, imageGray, CV_BGR2GRAY);
	else
		imageGray = image;
	int delta = 5*2;
	int minArea = 1.0/pow(10,2)*240*320;
	int maxArea = 1.0/pow(2,2)*240*320;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.4; // smaller increase the number of regions
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);
	vector<vector<cv::Point>> regions;
	mserDetector(imageGray, regions);

	// preallocate
	cv::Mat mask(imageGray.rows,imageGray.cols,CV_8UC1, cv::Scalar(0));

	/////////////////// Find contours ///////////////////////
	vector<vector<cv::Point>> allContours;
	int border = 2;
	for(int i=0; i<regions.size(); i++)
	{
		cv::Rect boundRect = boundingRect( cv::Mat(regions[i]) );
		boundRect.x = max(0, boundRect.x-border);
		boundRect.y = max(0, boundRect.y-border);
		boundRect.width = min(imageGray.cols, boundRect.x+boundRect.width+2*border)-boundRect.x;
		boundRect.height= min(imageGray.rows, boundRect.y+boundRect.height+2*border)-boundRect.y;;
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

		vector<vector<cv::Point>> contours;
		cv::findContours(mask(boundRect), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, corner);

		for(int i=0; i<contours.size(); i++)
			if(cv::contourArea(contours[i]) >= minArea)
				allContours.push_back(contours[i]);
	}

	return allContours;
}

vector<shared_ptr<ActiveRegion>> objectify(const vector<vector<cv::Point>> &contours,
										   const TNT::Array2D<double> Sn,
										   const TNT::Array2D<double> SnInv,
										   double varxi, double probNoCorr,
										   const Time &imageTime)
{
	/////////////////// make objects of the new contours ///////////////////////
	vector<shared_ptr<ActiveRegion>> curRegions;
	for(int i=0; i<contours.size(); i++)
	{
		shared_ptr<ActiveRegion> ao1(new ActiveRegion(contours[i]));
		ao1->markFound(imageTime);
		ao1->setPosCov(Sn);
		curRegions.push_back(ao1);
	}

	/////////////////// similarity check ///////////////////////
	// Ideally, I would do this on all of the active objects at the end of the loop
	// but I still need to work out the math for that. Doing it here, I can take
	// advantage of everything having the same covariance
	Array2D<double> ssC = ActiveRegion::calcCorrespondence(curRegions, curRegions, 0.5*Sn, 2.0*SnInv, varxi, 0);

	// Now keep only things that don't have confusion
	// if there is confusion, only keep one
	vector<shared_ptr<ActiveRegion>> tempList;
	tempList.swap(curRegions);
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

		curRegions.push_back(tempList[i]);
	}

	return curRegions;
}

void matchify(vector<shared_ptr<ActiveRegion>> &activeRegions,
			  const vector<shared_ptr<ActiveRegion>> &curRegions,
			  vector<RegionMatch> &goodMatches,
			  vector<shared_ptr<ActiveRegion>> &repeatRegions,
			  const TNT::Array2D<double> Sn,
			  const TNT::Array2D<double> SnInv,
			  double varxi, double probNoCorr,
			  const Time &imageTime)
{
	/////////////////// Establish correspondence based on postiion ///////////////////////
	Array2D<double> C = ActiveRegion::calcCorrespondence(activeRegions, curRegions, Sn, SnInv, varxi, probNoCorr);

cout << "------------------ activeRegions --------------------------------" << endl;;
for(int i=0; i<activeRegions.size(); i++)
{
	cout << activeRegions[i]->getId() << ": " << activeRegions[i]->getLife() << "\t";
	cout << activeRegions[i]->getLastFoundPos().x << "x" << activeRegions[i]->getLastFoundPos().y << "\t";
	cout << activeRegions[i]->getArea() << endl;
}

cout << "------------------ curRegions --------------------------------" << endl;;
for(int i=0; i<curRegions.size(); i++)
{
	cout << curRegions[i]->getId() << ": " << curRegions[i]->getLife() << "\t";
	cout << curRegions[i]->getLastFoundPos().x << "x" << curRegions[i]->getLastFoundPos().y << "\t";
	cout << curRegions[i]->getArea() << endl;
}

	///////////////////  make matches ///////////////////////
	shared_ptr<ActiveRegion> aoPrev, aoCur;
	vector<shared_ptr<ActiveRegion>> newObjects;
	int N1 = activeRegions.size();
	int N2 = curRegions.size();
	vector<bool> prevMatched(N1, false);
	vector<bool> curMatched(N2, false);
	for(int i=0; i<N1; i++)
	{
		if(N2 == 0 || C[i][N2] > 0.5)
			continue; // this object probably doesn't have a partner

		aoPrev = activeRegions[i];

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

		prevMatched[i] = true;
		curMatched[maxIndex] = true;
		aoCur = curRegions[maxIndex];

		RegionMatch m;
		m.aoPrev = aoPrev;
		m.aoCur = aoCur;
		m.score = C[i][maxIndex];
		goodMatches.push_back(m);
		aoPrev->copyData(*aoCur);
		aoPrev->markFound(imageTime);
		aoPrev->addLife(2);
		repeatRegions.push_back(aoPrev);
	}

	vector<int> dupIndices;
	for(int j=0; j<curRegions.size(); j++)
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
						Log::alert(String()+"found dupe at " + i + " -- id " + activeRegions[i]->getId());
					}
			}

			// Now add the new region which will be the 
			// only remaining one
			if(addMe)
				newObjects.push_back(curRegions[j]);
		}

	// sort and remove repeats 
	sort(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator endIter = unique(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator iter = dupIndices.begin();
	while(iter != endIter)
	{
		activeRegions[(*iter)]->kill();
		iter++;
	}

	for(int i=0; i<activeRegions.size(); i++)
		activeRegions[i]->takeLife(1);

printArray("C:\n",C);
	sort(activeRegions.begin(), activeRegions.end(), ActiveRegion::sortPredicate);
	while(activeRegions.size() > 0 && !activeRegions.back()->isAlive() )
		activeRegions.pop_back();

	// TODO
	for(int i=0; i<newObjects.size(); i++)
		activeRegions.push_back(newObjects[i]);
}

}
