#include "TargetFinder2.h"

namespace ICSL {
namespace Quadrotor{
using namespace TNT;

TargetFinder2::TargetFinder2()
{
	mRunning = false;
	mFinished = true;
	mUseIbvs = false;
	mFirstImageProcessed = false;
	mHaveUpdatedSettings = true;
	mIsMotorOn = false;

	mImageProcTimeUS = 0;

	mNewImageReady = false;

	mImageDataNext = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

	mObsvTranslational = NULL;
}

void TargetFinder2::shutdown()
{
	Log::alert("-------------------------- Target finder shutdown started ----------------------");
	mRunning = false;
	while(!mFinished)
		System::msleep(10);

	mImageDataNext = NULL;

	Log::alert("-------------------------- Target finder done ----------------------");
}

void TargetFinder2::initialize()
{
}

void TargetFinder2::run()
{
#ifndef ICSL_TARGETFIND_SIMULATION
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	Array2D<double> Sn = pow(5,2)*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
//	double varxi = pow(300,2);
	double varxi_ratio = 0.2;
	double probNoCorr = 0.0000001;

	shared_ptr<RectGroup> target;
	shared_ptr<DataImage> imageData;
	cv::Mat curImage, curImageGray, pyr1ImageGray, imageAnnotated;
	float targetRatios[] = {0, 0, 0, 0, 0, 0};
	String logString;
	Time procStart;
	Time imageTime;
	while(mRunning)
	{
		if(mNewImageReady
			&& mIsMotorOn
			)
		{
			procStart.setTime();
			mNewImageReady = false;

			imageData = mImageDataNext;
//			imageData->image->copyTo(curImage);
//			imageData->imageGray->copyTo(curImageGray);
//			if(curImageGray.cols == 640)
//				cv::pyrDown(curImageGray, pyr1ImageGray);
//			else
//				pyr1ImageGray = curImageGray;
//			cvtColor(pyr1Image, pyr1ImageGray, CV_BGR2GRAY);

			if(mHaveUpdatedSettings)
			{
				mMutex_params.lock();
//				qualityLevel = mQualityLevel;
//				sepDist = mSepDist;
//				fastThresh = mFASTThreshold;
//				pointCntTarget = mPointCntTarget;
//				fastAdaptRate = mFASTAdaptRate;
				mMutex_params.unlock();

				mHaveUpdatedSettings = false;
			}

			imageTime.setTime(imageData->timestamp);

			vector<vector<cv::Point>> allContours = findContours(*imageData->imageGray);
			vector<shared_ptr<ActiveRegion>> curRegions = objectify(allContours,Sn,SnInv,varxi_ratio,probNoCorr,imageTime);
			{ // special announcement of region centroids only
				shared_ptr<ImageRegionLocData> regionData(new ImageRegionLocData());
				regionData->imageData = imageData;
				regionData->timestamp.setTime(imageTime);
				regionData->regionLocs.resize(curRegions.size());
				for(int i=0; i<curRegions.size(); i++)
					regionData->regionLocs[i] = curRegions[i]->getFoundPos();
				for(int i=0; i<mRegionListeners.size(); i++)
					mRegionListeners[i]->onRegionsFound(regionData);
			}

			double f = imageData->focalLength;
			cv::Point2f center = imageData->center;

			Array2D<double> oldState(9,1), oldErrCov(9,9);
			Array2D<double> mv(3,1), Sv(3,3);
			double mz, varz;
			const Array2D<double> curState = mObsvTranslational->estimateStateAtTime(imageTime);
			const Array2D<double> curErrCov = mObsvTranslational->estimateErrCovAtTime(imageTime);
			Array2D<double> omega(3,1);
			SO3 curAtt = imageData->att;
			SO3 oldAtt, attChange;
			double theta;
			Array2D<double> axis;
			double dt;
			for(int i=0; i<mActiveRegions.size(); i++)
			{
				shared_ptr<ActiveRegion> ao = mActiveRegions[i];

				dt = Time::calcDiffNS(ao->getLastFoundTime(), imageTime)/1.0e6;
				oldState.inject(mObsvTranslational->estimateStateAtTime(ao->getLastFoundTime()));
				oldErrCov.inject(mObsvTranslational->estimateErrCovAtTime(ao->getLastFoundTime()));
				oldAtt = mObsvAngular->estimateAttAtTime(ao->getLastFoundTime());

				attChange = curAtt*oldAtt.inv();
				attChange.getAngleAxis(theta, axis);
				omega.inject(theta/dt*axis);

				mv.inject(0.5*(submat(oldState,3,5,0,0)+submat(curState,3,5,0,0)));
				// HACK HACK HACK
				Sv.inject(0.5*(submat(oldErrCov,3,5,3,5)+submat(curErrCov,3,5,3,5)));
				mz = 0.5*(oldState[2][0]+curState[2][0]);
				varz = 0.5*(oldErrCov[2][2]+curErrCov[2][2]);

				ao->updatePositionDistribution(mv, Sv, mz, varz, f, center, omega, imageTime);
			}

			vector<RegionMatch> goodMatches;
			vector<shared_ptr<ActiveRegion>> repeatRegions, newRegions;
			matchify(curRegions, goodMatches, repeatRegions, newRegions, Sn, SnInv, varxi_ratio, probNoCorr, imageTime);

			double procTime = procStart.getElapsedTimeNS()/1.0e9;
//			if(repeatRegions.size() > 0 || newRegions.size() > 0)
			{
				shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
				imageData->image->copyTo(*imageAnnotated);
				drawTarget(*imageAnnotated, curRegions, repeatRegions);

				shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
				imageAnnotatedData->imageAnnotated = imageAnnotated;
				imageAnnotatedData->imageDataSource = imageData;
				imageAnnotatedData->timestamp.setTime(imageData->timestamp);

				shared_ptr<ImageTargetFind2Data> data(new ImageTargetFind2Data());
				data->type = DATA_TYPE_TARGET_FIND;
				data->repeatRegions= repeatRegions;
				data->newRegions= newRegions;
				data->imageData = imageData;
				data->imageAnnotatedData = imageAnnotatedData;
				data->timestamp.setTime(imageTime);

				for(int i=0; i<mListeners.size(); i++)
					mListeners[i]->onTargetFound2(data);

				logString = String()+procTime;
				mQuadLogger->addEntry(LOG_ID_TARGET_FIND_PROC_TIME,logString, LOG_FLAG_CAM_RESULTS);
			}
		}

		System::msleep(1);
	}

	mFinished = true;
#endif
}

void TargetFinder2::drawTarget(cv::Mat &image,
							   const vector<shared_ptr<ActiveRegion>> &curRegions,
							   const vector<shared_ptr<ActiveRegion>> &repeatRegions)
{
	if(curRegions.size() == 0 && repeatRegions.size() == 0)
		return;

	vector<vector<cv::Point>> curContours(curRegions.size()), repeatContours(repeatRegions.size());
	for(int i=0; i<curRegions.size(); i++)
		curContours[i] = curRegions[i]->getContour();
	for(int i=0; i<repeatRegions.size(); i++)
		repeatContours[i] = repeatRegions[i]->getContour();

	cv::drawContours(image, curContours, -1, cv::Scalar(255,0,0), 2);
	cv::drawContours(image, repeatContours, -1, cv::Scalar(0,0,255), 2);

//	cv::Point p1, p2;
//	for(int i=0; i<curRegions.size(); i++)
//	{
//		const vector<shared_ptr<ActiveRegion>> neighbors = curRegions[i]->getNeighbors();
//		for(int j=0; j<neighbors.size(); j++)
//			if(curRegions[i]->getId() < neighbors[j]->getId())
//			{
//				p1.x = curRegions[i]->getExpectedPos()[0][0];
//				p1.y = curRegions[i]->getExpectedPos()[1][0];
//				p2.x = neighbors[j]->getExpectedPos()[0][0];
//				p2.y = neighbors[j]->getExpectedPos()[1][0];
//
//				line(image, p1, p2, cv::Scalar(0,255,255), 1);
//			}
//	}
//	for(int i=0; i<repeatRegions.size(); i++)
//	{
//		const vector<shared_ptr<ActiveRegion>> neighbors = repeatRegions[i]->getNeighbors();
//		for(int j=0; j<neighbors.size(); j++)
//			if(repeatRegions[i]->getId() < neighbors[j]->getId())
//			{
//				p1.x = repeatRegions[i]->getExpectedPos()[0][0];
//				p1.y = repeatRegions[i]->getExpectedPos()[1][0];
//				p2.x = neighbors[j]->getExpectedPos()[0][0];
//				p2.y = neighbors[j]->getExpectedPos()[1][0];
//
//				line(image, p1, p2, cv::Scalar(0,255,255), 1);
//			}
//	}
}


void TargetFinder2::onNewSensorUpdate(const shared_ptr<IData> &data)
{
	if(data->type == DATA_TYPE_IMAGE)
	{
		mImageDataNext = static_pointer_cast<DataImage>(data);
		mNewImageReady = mNewImageReady_targetFind = true;
	}
}

vector<vector<cv::Point>> TargetFinder2::findContours(const cv::Mat &image)
{
	cv::Mat pyrImage;
	double pyrScale = 0.5;
	resize(image, pyrImage, cv::Size(), pyrScale, pyrScale, cv::INTER_AREA);

	int delta = 5*2;
	int minArea = 1.0/pow(15,2)*pyrImage.rows*pyrImage.cols;
	int maxArea = 1.0/pow(2,2)*pyrImage.rows*pyrImage.cols;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.4; // smaller increase the number of regions
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);
	vector<vector<cv::Point>> regions;
	mserDetector(pyrImage, regions);

	// preallocate
	cv::Mat mask(pyrImage.rows,pyrImage.cols,CV_8UC1, cv::Scalar(0));

	/////////////////// Find contours ///////////////////////
	vector<vector<cv::Point>> allContours;
	vector<vector<cv::Point>> contours;
	cv::Rect boundRect;
	int border = 2;
	for(int i=0; i<regions.size(); i++)
	{
		boundRect = boundingRect( cv::Mat(regions[i]) );
		// reject regions on the border since they will change, but perhaps not enough to 
		// prevent matches
		if(boundRect.x == 0 || boundRect.y == 0 ||
			boundRect.x+boundRect.width == pyrImage.cols || boundRect.y+boundRect.height == pyrImage.rows )
			continue;

		boundRect.x = max(0, boundRect.x-border);
		boundRect.y = max(0, boundRect.y-border);
		boundRect.width = min(pyrImage.cols, boundRect.x+boundRect.width+2*border)-boundRect.x;
		boundRect.height= min(pyrImage.rows, boundRect.y+boundRect.height+2*border)-boundRect.y;;
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

		cv::findContours(mask(boundRect), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, corner);

		for(int i=0; i<contours.size(); i++)
		{
			if(cv::contourArea(contours[i]) >= minArea)
				allContours.push_back(contours[i]);
		}
	}


	// scale the contours back to the original image size
	for(int i=0; i<allContours.size(); i++)
		for(int j=0; j<allContours[i].size(); j++)
			allContours[i][j] *= 1.0/pyrScale;

	return allContours;
}

vector<shared_ptr<ActiveRegion>> TargetFinder2::objectify(const vector<vector<cv::Point>> &contours,
										   const TNT::Array2D<double> Sn,
										   const TNT::Array2D<double> SnInv,
										   double varxi_ratio, double probNoCorr,
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
	Array2D<double> ssC = ActiveRegion::calcCorrespondence(curRegions, curRegions, 0.5*Sn, 2.0*SnInv, varxi_ratio, 0);

	// Now keep only things that don't have confusion
	// if there is confusion, only keep one
	vector<shared_ptr<ActiveRegion>> tempList;
	tempList.swap(curRegions);
	vector<bool> isStillGood(tempList.size(), true);
	for(int i=0; i<tempList.size(); i++)
	{
		if(!isStillGood[i])
		{
			tempList[i]->kill();
			continue;
		}
		vector<int> confusionList;
		for(int j=i+1; j<tempList.size(); j++)
			if(ssC[i][j] > 0.2)
				confusionList.push_back(j);

		for(int j=0; j<confusionList.size(); j++)
			isStillGood[confusionList[j]] = false;

		curRegions.push_back(tempList[i]);
	}

	for(int i=0; i<curRegions.size(); i++)
		for(int j=i+1; j<curRegions.size(); j++)
			curRegions[i]->addNeighbor(curRegions[j], true);

	return curRegions;
}

void TargetFinder2::matchify(const vector<shared_ptr<ActiveRegion>> &curRegions,
			  vector<RegionMatch> &goodMatches,
			  vector<shared_ptr<ActiveRegion>> &repeatRegions,
			  vector<shared_ptr<ActiveRegion>> &newRegions,
			  const TNT::Array2D<double> Sn,
			  const TNT::Array2D<double> SnInv,
			  double varxi_ratio, double probNoCorr,
			  const Time &imageTime)
{
	/////////////////// Establish correspondence based on postiion ///////////////////////
	Array2D<double> C = ActiveRegion::calcCorrespondence(mActiveRegions, curRegions, Sn, SnInv, varxi_ratio, probNoCorr);

	///////////////////  make matches ///////////////////////
	shared_ptr<ActiveRegion> aoPrev, aoCur;
	int N1 = mActiveRegions.size();
	int N2 = curRegions.size();
	vector<bool> prevMatched(N1, false);
	vector<bool> curMatched(N2, false);
	vector<cv::Point2f> offsets;
	for(int i=0; i<N1; i++)
	{
		if(N2 == 0 || C[i][N2] > 0.4)
			continue; // this object probably doesn't have a partner

		aoPrev = mActiveRegions[i];

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

		curMatched[maxIndex] = true;
		aoCur = curRegions[maxIndex];

		cv::Point offset = aoCur->getFoundPos()-aoPrev->getFoundPos();
		offsets.push_back(offset);

		RegionMatch m;
		m.aoPrev = aoPrev;
		m.aoCur = aoCur;
		m.score = C[i][maxIndex];
		goodMatches.push_back(m);
		aoPrev->copyData(*aoCur);
		aoPrev->markFound(imageTime);
		aoPrev->addLife(2);
		repeatRegions.push_back(aoPrev);
		aoCur->kill();
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
					}
			}
			// Now add the new region which will be the 
			// only remaining one
			if(addMe)
				newRegions.push_back(curRegions[j]);
			else
				curRegions[j]->kill();
		}

	// sort and remove repeats 
	sort(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator endIter = unique(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator iter = dupIndices.begin();
	while(iter != endIter)
	{
		mActiveRegions[(*iter)]->kill();
		iter++;
	}

	for(int i=0; i<mActiveRegions.size(); i++)
		mActiveRegions[i]->takeLife(1);

	sort(mActiveRegions.begin(), mActiveRegions.end(), ActiveRegion::sortPredicate);
	while(mActiveRegions.size() > 0 && !mActiveRegions.back()->isAlive() )
		mActiveRegions.pop_back();

	// TODO
	for(int i=0; i<newRegions.size(); i++)
		mActiveRegions.push_back(newRegions[i]);
}

} // namespace Quadrotor
} // namespace ICSL
