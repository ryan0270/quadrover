#include "TargetFinder2.h"

namespace ICSL {
namespace Quadrotor{
using namespace TNT;
//using namespace ICSL::Constants;

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
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	Array2D<double> Sn = 10*10*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
	double varxi = pow(500,2);
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
//			&& mIsMotorOn
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
			vector<shared_ptr<ActiveRegion>> curRegions = objectify(allContours,Sn,SnInv,varxi,probNoCorr,imageTime);

			Array2D<double> mv(3,1,0.0);
			Array2D<double> Sv = 0.2*0.2*createIdentity((double)3);
			double mz = 1;
			double sz = 0.05;
			double f = imageData->focalLength;
			cv::Point2f center = imageData->center;
			Array2D<double> omega(3,1,0.0);

			for(int i=0; i<mActiveRegions.size(); i++)
			{
				shared_ptr<ActiveRegion> ao = mActiveRegions[i];
				ao->updatePositionDistribution(mv, Sv, mz, sz*sz, f, center, omega, imageTime);
			}

			vector<RegionMatch> goodMatches;
			vector<shared_ptr<ActiveRegion>> repeatRegions;
			matchify(curRegions, goodMatches, repeatRegions, Sn, SnInv, varxi, probNoCorr, imageTime);

			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
			imageData->image->copyTo(*imageAnnotated);
			drawTarget(*imageAnnotated, curRegions, repeatRegions);

			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
			imageAnnotatedData->imageAnnotated = imageAnnotated;
			imageAnnotatedData->imageDataSource = imageData;
			imageAnnotatedData->timestamp.setTime(imageData->timestamp);

			shared_ptr<ImageTargetFind2Data> data(new ImageTargetFind2Data());
			data->type = DATA_TYPE_TARGET_FIND;
			data->regions= repeatRegions;
			data->imageData = imageData;
			data->imageAnnotatedData = imageAnnotatedData;
			data->timestamp.setTime(imageTime);
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onTargetFound2(data);

//			target = findTarget(pyr1ImageGray, *imageData->cameraMatrix, *imageData->distCoeffs);
//			// TODO: Compensate for current attitdue
//			if(curImage.cols == 640)
//				Log::alert("TargetFinder2 doesn't handle 640x480 images at this time");
//
//			double procTime = procStart.getElapsedTimeNS()/1.0e9;
//
//			if(target != NULL)
//			{
//				shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
//				curImage.copyTo(*imageAnnotated);
//				drawTarget(*imageAnnotated, target);
//
//				shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
//				imageAnnotatedData->imageAnnotated = imageAnnotated;
//				imageAnnotatedData->imageDataSource = imageData;
//				imageAnnotatedData->timestamp.setTime(imageData->timestamp);
//				mImageAnnotatedLast = imageAnnotatedData;
//
//				shared_ptr<ImageTargetFindData> data(new ImageTargetFindData());
//				data->type = DATA_TYPE_CAMERA_POS;
//				data->target = target;
//				data->imageData = imageData;
//				data->imageAnnotatedData = imageAnnotatedData;
//				data->timestamp.setTime(imageData->timestamp);
//				for(int i=0; i<mListeners.size(); i++)
//					mListeners[i]->onTargetFound2(data);
//
//				logString = String();
//				for(int i=0; i<target->squareData.size(); i++)
//					logString = logString+target->squareData[i]->center.x+"\t"+target->squareData[i]->center.y+"\t";
//				mQuadLogger->addEntry(LOG_ID_TARGET_FIND_CENTERS, logString, LOG_FLAG_CAM_RESULTS);
//
//				logString = String();
//				for(int i=0; i<target->squareData.size(); i++)
//					logString = logString+target->squareData[i]->area+"\t";
//				mQuadLogger->addEntry(LOG_ID_TARGET_FIND_AREAS,logString, LOG_FLAG_CAM_RESULTS);
//
//				logString = String()+procTime;
//				mQuadLogger->addEntry(LOG_ID_TARGET_FIND_PROC_TIME,logString, LOG_FLAG_CAM_RESULTS);
//			}
		}

		System::msleep(1);
	}

	mFinished = true;
}

void TargetFinder2::drawTarget(cv::Mat &image,
							   const vector<shared_ptr<ActiveRegion>> &curRegions,
							   const vector<shared_ptr<ActiveRegion>> &repeatRegions)
{
	vector<vector<cv::Point>> curContours(curRegions.size()), repeatContours(repeatRegions.size());
	for(int i=0; i<curRegions.size(); i++)
		curContours[i] = curRegions[i]->getContour();
	for(int i=0; i<repeatRegions.size(); i++)
		repeatContours[i] = repeatRegions[i]->getContour();

	cv::drawContours(image, curContours, -1, cv::Scalar(255,0,0), 2);
	cv::drawContours(image, repeatContours, -1, cv::Scalar(0,0,255), 2);

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

			line(image,cen,p1,cv::Scalar(0,255,255),1);
			line(image,cen,p2,cv::Scalar(255,0,255),1);
		}
	}
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
	int delta = 5*2;
	int minArea = 1.0/pow(10,2)*240*320;
	int maxArea = 1.0/pow(2,2)*240*320;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.4; // smaller increase the number of regions
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);
	vector<vector<cv::Point>> regions;
	mserDetector(image, regions);

	// preallocate
	cv::Mat mask(image.rows,image.cols,CV_8UC1, cv::Scalar(0));

	/////////////////// Find contours ///////////////////////
	vector<vector<cv::Point>> allContours;
	int border = 2;
	for(int i=0; i<regions.size(); i++)
	{
		cv::Rect boundRect = boundingRect( cv::Mat(regions[i]) );
		boundRect.x = max(0, boundRect.x-border);
		boundRect.y = max(0, boundRect.y-border);
		boundRect.width = min(image.cols, boundRect.x+boundRect.width+2*border)-boundRect.x;
		boundRect.height= min(image.rows, boundRect.y+boundRect.height+2*border)-boundRect.y;;
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

vector<shared_ptr<ActiveRegion>> TargetFinder2::objectify(const vector<vector<cv::Point>> &contours,
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

void TargetFinder2::matchify(const vector<shared_ptr<ActiveRegion>> &curRegions,
			  vector<RegionMatch> &goodMatches,
			  vector<shared_ptr<ActiveRegion>> &repeatRegions,
			  const TNT::Array2D<double> Sn,
			  const TNT::Array2D<double> SnInv,
			  double varxi, double probNoCorr,
			  const Time &imageTime)
{
	/////////////////// Establish correspondence based on postiion ///////////////////////
	Array2D<double> C = ActiveRegion::calcCorrespondence(mActiveRegions, curRegions, Sn, SnInv, varxi, probNoCorr);

	///////////////////  make matches ///////////////////////
	shared_ptr<ActiveRegion> aoPrev, aoCur;
	vector<shared_ptr<ActiveRegion>> newObjects;
	int N1 = mActiveRegions.size();
	int N2 = curRegions.size();
	vector<bool> matched(N2, false);
	for(int i=0; i<N1; i++)
	{
		if(N2 == 0 || C[i][N2] > 0.5)
			continue; // this object probably doesn't have a partner

		aoPrev = mActiveRegions[i];

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

	for(int j=0; j<curRegions.size(); j++)
		if(!matched[j])
			newObjects.push_back(curRegions[j]);

	for(int i=0; i<mActiveRegions.size(); i++)
		mActiveRegions[i]->takeLife(1);

	sort(mActiveRegions.begin(), mActiveRegions.end(), ActiveRegion::sortPredicate);
	while(mActiveRegions.size() > 0 && !mActiveRegions.back()->isAlive() )
		mActiveRegions.pop_back();

	// TODO
	for(int i=0; i<newObjects.size(); i++)
		mActiveRegions.push_back(newObjects[i]);
}

} // namespace Quadrotor
} // namespace ICSL