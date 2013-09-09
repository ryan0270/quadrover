#include "TargetFinder.h"

namespace ICSL {
namespace Quadrotor{
//using namespace TNT;
//using namespace ICSL::Constants;

TargetFinder::TargetFinder()
{
	mRunning = false;
	mFinished = true;
	mUseIbvs = false;
	mFirstImageProcessed = false;
	mHaveUpdatedSettings = true;
//	mCurImageAnnotated = NULL;
	mIsMotorOn = false;

	mImageProcTimeUS = 0;

//	mLastProcessTime.setTimeMS(0);

	mNewImageReady = false;

	mImageDataNext = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

//	mQualityLevel = 0.05;
//	mSepDist = 10;
//	mFASTThreshold = 30;
//	mPointCntTarget = 30;
//	mFASTAdaptRate = 0.01;
}

void TargetFinder::shutdown()
{
	Log::alert("-------------------------- Target finder shutdown started ----------------------");
	mRunning = false;
	while(!mFinished)
		System::msleep(10);

	mImageDataNext = NULL;

	Log::alert("-------------------------- Target finder done ----------------------");
}

void TargetFinder::initialize()
{
}

void TargetFinder::run()
{
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	shared_ptr<RectGroup> target;
	shared_ptr<DataImage> imageData;
	cv::Mat curImage, curImageGray, pyr1ImageGray, imageAnnotated;
	float targetRatios[] = {0, 0, 0, 0, 0, 0};
	String logString;
	Time procStart;
	while(mRunning)
	{
		if(mNewImageReady
			&& mIsMotorOn
			)
		{
			procStart.setTime();
			mNewImageReady = false;

			imageData = mImageDataNext;
//			imageData->lock();
			try
			{
				imageData->image->copyTo(curImage);
				imageData->imageGray->copyTo(curImageGray);
			}
			catch(...) {Log::alert("copyTo error in TargetFinder 1");}
//			imageData->unlock();
			if(curImageGray.cols == 640)
				cv::pyrDown(curImageGray, pyr1ImageGray);
			else
				pyr1ImageGray = curImageGray;
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

			target = findTarget(pyr1ImageGray, *imageData->cameraMatrix, *imageData->distCoeffs);
//			target = findTarget(curImage);
			// TODO: Compensate for current attitdue
			if(curImage.cols == 640)
				Log::alert("TargetFinder doesn't handle 640x480 images at this time");

			double procTime = procStart.getElapsedTimeNS()/1.0e9;

			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
			curImage.copyTo(*imageAnnotated);
			if(target != NULL)
				drawTarget(*imageAnnotated, target);

			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
			imageAnnotatedData->imageAnnotated = imageAnnotated;
			imageAnnotatedData->imageDataSource = imageData;
			imageAnnotatedData->timestamp.setTime(imageData->timestamp);
			mImageAnnotatedLast = imageAnnotatedData;

			shared_ptr<ImageTargetFindData> data(new ImageTargetFindData());
			data->type = DATA_TYPE_CAMERA_POS;
			data->target = target;
			data->imageData = imageData;
			data->imageAnnotatedData = imageAnnotatedData;
			data->timestamp.setTime(imageData->timestamp);
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onTargetFound(data);

			if(target != NULL)
			{
				logString = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_TARGET_FIND_CENTERS+"\t";
				for(int i=0; i<target->squareData.size(); i++)
					logString = logString+target->squareData[i]->center.x+"\t"+target->squareData[i]->center.y+"\t";
				mQuadLogger->addLine(logString, LOG_FLAG_CAM_RESULTS);

				logString = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_TARGET_FIND_AREAS+"\t";
				for(int i=0; i<target->squareData.size(); i++)
					logString = logString+target->squareData[i]->area+"\t";
				mQuadLogger->addLine(logString, LOG_FLAG_CAM_RESULTS);

				logString = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_TARGET_FIND_PROC_TIME+"\t"+procTime;
				mQuadLogger->addLine(logString, LOG_FLAG_CAM_RESULTS);
			}
		}

		System::msleep(1);
	}

	mFinished = true;
}

//shared_ptr<RectGroup> TargetFinder::findTarget(cv::Mat &image)
shared_ptr<RectGroup> TargetFinder::findTarget(cv::Mat &image, cv::Mat const &cameraMatrix, cv::Mat const &distCoeffs)
{
	vector<vector<cv::Point> > squares;
	vector<vector<cv::Point> > contours;

	cv::Mat gray0, gray;
	if(image.channels() == 3)
		cvtColor(image, gray0, CV_BGR2GRAY);
	else
	{
		try{ image.copyTo(gray0); }
		catch(...) {Log::alert("copyTo error in TargetFinder 2"); return NULL;}
	}
//	medianBlur(image, gray0, 5);


	Canny(gray0, gray, 50, 150, 3);

	// find contours and store them all as a list
	findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	vector<cv::Point> approx;
	// test each contour for rectangle-ishness
	for( size_t i = 0; i < contours.size(); i++ )
	{
		if( fabs(contourArea(cv::Mat(contours[i]))) < 30)
				continue;

		// approximate contour with accuracy proportional
		// to the contour perimeter
		approxPolyDP(cv::Mat(contours[i]), approx, arcLength(cv::Mat(contours[i]), true)*0.04, true);

		// square contours should have 4 vertices after approximation
		// relatively large area (to filter out noisy contours)
		// and be convex.
		// Note: absolute value of an area is used because
		// area may be positive or negative - in accordance with the
		// contour orientation
		if( approx.size() == 4 )
		{
			double maxCosine = 0;

			for( int j = 2; j < 5; j++ )
			{
				// find the maximum cosine of the angle between joint edges
				double cosine = abs(angle(approx[j%4], approx[j-2], approx[j-1]));
				maxCosine = MAX(maxCosine, cosine);
			}

			// if cosines of all angles are small
			// (all angles are ~90 degree) then write quandrange
			// vertices to resultant sequence
			if( maxCosine < 0.3 )
				squares.push_back(approx);
		}
	}

	// Make square object
	double f = cameraMatrix.at<double>(0,0);
	double cx = cameraMatrix.at<double>(0,2);
	double cy = cameraMatrix.at<double>(1,2);
	cv::Point2f center(cx, cy);
	vector<shared_ptr<Rect> > squareData(squares.size());
	for(int i=0; i<squares.size(); i++)
	{
		vector<cv::Point2f> squares2(squares[i].size());
		for(int j=0; j<squares[i].size(); j++)
			squares2[j] = squares[i][j];
		cv::undistortPoints(squares2, squares2, cameraMatrix, distCoeffs);
		for(int j=0; j<squares2.size(); j++)
			squares2[j] = squares2[j]*f+center;
		shared_ptr<Rect> data(new Rect(squares2));
		squareData[i] = data;
	}

	// Group squares that seem to be similar
	vector< shared_ptr<RectGroup> > groups;
	for(int i=0; i<squareData.size(); i++)
	{
		shared_ptr<Rect> data = squareData[i];
		// See if we belong to an existing group
		bool foundGroup = false;
		for(int g=0; g<groups.size(); g++)
		{
			if( norm(data->center-groups[g]->meanCenter) < 20 &&
				abs(data->area - groups[g]->meanArea) < 0.10*groups[g]->meanArea )
			{
				groups[g]->add(data);
				foundGroup = true;
				break;
			}
		}

		if(!foundGroup)
		{
			shared_ptr<RectGroup> newGroup(new RectGroup(data));
			groups.push_back(newGroup);
		}
	}

	// Sort area from largest to smallest
	sort(groups.begin(), groups.end(), 
				[&](shared_ptr<RectGroup> const &g1, shared_ptr<RectGroup> const &g2){return g1->meanArea  > g2->meanArea;});

	// Find groups that seem to have the correct inter-group relationship
	vector<shared_ptr<RectGroup> > candidateSets;
	vector<double> candidateSetScores;
//	float idealRatios[] = {1.411, 2.664, 38.968, 1.888, 27.621, 14.627};
	float idealRatios[] = {3.4, 30, 9};
	for(int i=0; i<groups.size(); i++)
	{
		vector<double> ratios(groups.size(),0.0);

		if(ratios.size() == 0)
			continue;

		// find ratios to all other groups
		shared_ptr<RectGroup> data = groups[i];
		cv::Point2f center = groups[i]->meanCenter;
		double a1 = groups[i]->meanArea;
		for(int j=i+1; j<groups.size(); j++)
		{
			if(norm(center-groups[j]->meanCenter) < 20)
				ratios[j] = a1/groups[j]->meanArea;
		}

		if(ratios.size() == 0)
			continue;

		// Find a set of groups that looks good (based only on the ratios relative to the largest rect)
		vector<shared_ptr<RectGroup> > groupSet;
		vector<int> usedRatios;
		groupSet.push_back(groups[i]);
		for(int j=0; j<2; j++)
		{
			int minIndex = 0;
			int minErr= abs(ratios[0]-idealRatios[j]);
			for(int r=1; r<ratios.size(); r++)
			{
				float err = abs(ratios[r]-idealRatios[j]);
				if(err < minErr)
				{
					minIndex = r;
					minErr = err;
				}
			}

//			if(ratios[minIndex] != 0 && find(usedRatios.begin(), usedRatios.end(), minIndex) == usedRatios.end())
			if(minErr < 0.3*idealRatios[j] &&
				find(usedRatios.begin(), usedRatios.end(), minIndex) == usedRatios.end())
			{
				usedRatios.push_back(minIndex);
				groupSet.push_back(groups[minIndex]);
			}
		}

		if(groupSet.size() < 3)
			continue;

		// Find the best combination of specific squares
		shared_ptr<Rect> b0, b1, b2, b3;
		shared_ptr<Rect> d0, d1, d2;
		float bestScore = 0x7FFFFFFF;
		for(int j0=0; j0<groupSet[0]->squareData.size(); j0++)
		{
			d0 = groupSet[0]->squareData[j0];
			for(int j1=0; j1<groupSet[1]->squareData.size(); j1++)
			{
				d1 = groupSet[1]->squareData[j1];
				for(int j2=0; j2<groupSet[2]->squareData.size(); j2++)
				{
					d2 = groupSet[2]->squareData[j2];
					float ratios[3];
					ratios[0] = d0->area/d1->area;
					ratios[1] = d0->area/d2->area;
					ratios[2] = d1->area/d2->area;

					float score = 0;
					for(int r=0; r<2; r++)
						score += abs(ratios[i]-idealRatios[i])/idealRatios[i];

					if(score < bestScore)
					{
						b0 = d0;
						b1 = d1;
						b2 = d2;
						bestScore = score;
					}
				}
			}
		}
		shared_ptr<RectGroup> set(new RectGroup);
		set->add(d0);
		set->add(d1);
		set->add(d2);
		candidateSets.push_back(set);
		candidateSetScores.push_back(bestScore);
	}

	shared_ptr<RectGroup> bestSet = NULL;
	if(candidateSets.size() > 0)
	{
		int bestScore = candidateSetScores[0];
		bestSet = candidateSets[0];
		for(int c=1; c<candidateSetScores.size(); c++)
		{
			if(candidateSetScores[c] < bestScore)
			{
				bestScore = candidateSetScores[c];
				bestSet = candidateSets[c];
			}
		}
	}

	return bestSet;
}

void TargetFinder::drawTarget(cv::Mat &image, shared_ptr<RectGroup> const &target)
{
	for( int i = 0; i < target->squareData.size(); i++ )
	{
		const cv::Point *p = &target->squareData[i]->contourInt[0];
		int n = (int)target->squareData[i]->contourInt.size();
		polylines(image, &p, &n, 1, true, cv::Scalar(255,0,0), 1, CV_AA);
	}
}

//void TargetFinder::drawPoints(vector<cv::Point2f> const &points, cv::Mat &image)
//{
//	if(points.size() == 0)
//		return;
//	cv::Point2f p1;
//	for(int i=0; i<points.size(); i++)
//	{
//		p1 = points[i];
// 		circle(image,p1,3,cv::Scalar(0,255,0),-1);
//	}
//}

//void TargetFinder::enableIbvs(bool enable)
//{
//	mUseIbvs = enable;
//	if(mUseIbvs)
//	{
//		Log::alert("Turning IBVS on");
//		String str = String()+ mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IBVS_ENABLED + "\t";
//		mMutex_logger.lock();
//		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
//		mMutex_logger.unlock();
//		mFirstImageProcessed = false;
//	}
//	else
//	{
//		Log::alert("Turning IBVS off");
//		String str = String() + mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IBVS_DISABLED + "\t";
//		mMutex_logger.lock();
//		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
//		mMutex_logger.unlock();
//		mFirstImageProcessed = false;
//	}
//}

void TargetFinder::onNewSensorUpdate(shared_ptr<IData> const &data)
{
	if(data->type == DATA_TYPE_IMAGE)
	{
		mImageDataNext = static_pointer_cast<DataImage>(data);
		mNewImageReady = mNewImageReady_targetFind = true;
	}
}
} // namespace Quadrotor
} // namespace ICSL