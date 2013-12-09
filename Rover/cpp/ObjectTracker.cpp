#include "ObjectTracker.h"

namespace ICSL {
namespace Quadrotor {
using namespace std;
using namespace TNT;

ObjectTracker::ObjectTracker()
{
	mRunning = false;
	mFinished = true;
	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
	mThreadNiceValue = 0;

	mFeatureData = NULL;
	mRegionData = NULL;

	mNewFeatureDataAvailable = false;
	mNewRegionDataAvailable = false;

	mObsvTranslation = NULL;
}

ObjectTracker::~ObjectTracker()
{
	if(mRunning)
		shutdown();
}

void ObjectTracker::initialize()
{
}

void ObjectTracker::shutdown()
{
	Log::alert("-------------------------- Object tracker shutdown started ----------------------");
	mRunning = false;
	while(!mFinished)
		System::msleep(10);

	Log::alert("-------------------------- Object tracker shutdown done ----------------------");
}

void ObjectTracker::run()
{
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);
	setpriority(PRIO_PROCESS, 0, mThreadNiceValue);
	int nice = getpriority(PRIO_PROCESS, 0);
	Log::alert(String()+"ObjectTracker nice value: "+nice);

	Array2D<double> SnPoint = 5*5*createIdentity((double)2);
	Array2D<double> SnInvPoint(2,2,0.0);
	SnInvPoint[0][0] = 1.0/SnPoint[0][0];
	SnInvPoint[1][1] = 1.0/SnPoint[1][1];

	Array2D<double> SnRegion = 10*10*createIdentity((double)2);
	Array2D<double> SnInvRegion(2,2,0.0);
	SnInvRegion[0][0] = 1.0/SnRegion[0][0];
	SnInvRegion[1][1] = 1.0/SnRegion[1][1];
	
	Array2D<double> Sn(2,2), SnInv(2,2);

	double probNoCorrPoints = 2e-4;
	double probNoCorrRegions = 2e-5;
	double probNoCorr;

	shared_ptr<ImageFeatureData> featureData = NULL;
	shared_ptr<ImageRegionData> regionData = NULL;

	double f;
	cv::Point2f center;

	// Keep in memory to avoid reallocation
	Array2D<double> C(50,50);
	vector<Array2D<double>> SdInvmdList, SaInvList, SaList;

	mNewFeatureDataAvailable = false;
	mNewRegionDataAvailable = false;

	vector<ObjectMatch> goodMatches;
	vector<shared_ptr<TrackedObject>> repeatObjects, newObjects;

	vector<shared_ptr<TrackedObject>> curObjects;
	shared_ptr<DataImage> imageData = NULL;

	bool processedImage;
	while(mRunning)
	{
		processedImage = false;
		Time curTime;
		if(mNewFeatureDataAvailable)
		{
			mMutex_featureData.lock();
			featureData = mFeatureData;
			mMutex_featureData.unlock();
			mNewFeatureDataAvailable = false;

			curTime = featureData->imageData->timestamp;
			f = featureData->imageData->focalLength;
			center = featureData->imageData->center;
			imageData = featureData->imageData;
			probNoCorr = probNoCorrPoints;
			Sn.inject(SnPoint);
			SnInv.inject(SnInvPoint);

			// make objects
			if(featureData != NULL)
			{
				curObjects.clear();
				curObjects.resize(featureData->featurePoints.size());
				for(int i=0; i<curObjects.size(); i++)
				{
					curObjects[i] = shared_ptr<TrackedObject>(new TrackedPoint(curTime, featureData->featurePoints[i]));
					curObjects[i]->setPosCov(Sn);
				}
			}

			processedImage = true;
		}
		else if(mNewRegionDataAvailable)
		{
			mMutex_featureData.lock();
			regionData = mRegionData;;
			mMutex_featureData.unlock();
			mNewRegionDataAvailable = false;

			curTime = regionData->imageData->timestamp;
			f = regionData->imageData->focalLength;
			center = regionData->imageData->center;
			imageData = regionData->imageData;
			probNoCorr = probNoCorrRegions;
			Sn.inject(SnRegion);
			SnInv.inject(SnInvRegion);

			curObjects.clear();
			curObjects.resize(regionData->regionCentroids.size());
			for(int i=0; i<curObjects.size(); i++)
			{
				curObjects[i] = shared_ptr<TrackedObject>(new TrackedRegion(curTime,
							regionData->regionContours[i],
							regionData->regionCentroids[i],
							regionData->regionMoments[i]));
				curObjects[i]->setPosCov(Sn);
			}

			doSimilarityCheck(curObjects);

			processedImage = true;
		}

		if(processedImage)//curObjects.size() != 0)
		{
			/////////////////// Get location priors for active objects ///////////////////////
			for(int i=0; i<mTrackedObjects.size(); i++)
			{
				shared_ptr<TrackedObject> to = mTrackedObjects[i];
				to->updatePositionDistribution(f, center, curTime);
			}

			// make matches
			matchify(curObjects, goodMatches, repeatObjects, newObjects, Sn, SnInv, probNoCorr, curTime,
					C, SdInvmdList, SaInvList, SaList );

			// draw
			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
			imageData->image->copyTo(*imageAnnotated);
			drawResults(*imageAnnotated, goodMatches, repeatObjects, newObjects);

			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
			imageAnnotatedData->imageAnnotated = imageAnnotated;
			imageAnnotatedData->imageDataSource = imageData;

			// Sort by age for stats
			sort(repeatObjects.begin(), repeatObjects.end(), TrackedObject::sortAgePredicate);
			vector<double> stats(4);
			if(repeatObjects.size() > 0)
			{
				stats[0] = repeatObjects[0]->getAge(); // oldest
				int medIdx = repeatObjects.size()/2;
				stats[1] = repeatObjects[medIdx]->getAge(); // median
			}
			else
			{ stats[0] = stats[1] = 0; }
			stats[2] = repeatObjects.size();
			stats[3] = newObjects.size();

			// Tell the world
			shared_ptr<ObjectTrackerData> data(new ObjectTrackerData());
			data->timestamp.setTime(imageData->timestamp);
			data->repeatObjects.swap(repeatObjects);
			data->newObjects.swap(newObjects);
			data->imageData = imageData;
			data->imageAnnotatedData = imageAnnotatedData;
			data->repeatObjectLocs.resize(repeatObjects.size());
			data->newObjectLocs.resize(newObjects.size());
			for(int i=0; i<repeatObjects.size(); i++)
				data->repeatObjectLocs[i] = repeatObjects[i]->getLocation();
			for(int i=0; i<newObjects.size(); i++)
				data->newObjectLocs[i] = newObjects[i]->getLocation();
			data->matches.swap(goodMatches);

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onObjectsTracked(data);

			if(mDataLogger != NULL)
				mDataLogger->addEntry(LOG_ID_OBJECT_TRACKING_STATS, stats, LOG_FLAG_CAM_RESULTS);

			curObjects.clear();
		}

		System::msleep(1);
	}

	mFinished = true;
}

void ObjectTracker::drawResults(cv::Mat &img,
								const vector<ObjectMatch> &matches,
								const vector<shared_ptr<TrackedObject>> &repeatObjects,
								const vector<shared_ptr<TrackedObject>> &newObjects)
{
//	for(int i=0; i<matches.size(); i++)
//		line(img, matches[i].prevPos, matches[i].curPos, cv::Scalar(0,255,0), 2);

	cv::Point2f p1, p2;
	vector<pair<Time, cv::Point2f>>::const_iterator iter;
	for(int i=0; i<repeatObjects.size(); i++)
	{
		const vector<pair<Time, cv::Point2f>> history = repeatObjects[i]->getHistory();
		iter = history.begin();
		p1 = iter->second;
		iter++;
		while(iter != history.end())
		{
			p2 = iter->second;
			line(img, p1, p2, cv::Scalar(0,255,0),2);
			p1 = p2;
			iter++;
		}
	}

	if( (repeatObjects.size() > 0 && repeatObjects[0]->getType() == TrackedObjectType::REGION) ||
		(newObjects.size() > 0 && newObjects[0]->getType() == TrackedObjectType::REGION) )
	{
		shared_ptr<TrackedRegion> region;
		vector<vector<cv::Point>> contours(newObjects.size());
		for(int i=0; i<newObjects.size(); i++)
		{
			region = static_pointer_cast<TrackedRegion>(newObjects[i]);
			const vector<cv::Point2f> *contour = &region->getContour();;
			contours[i].resize(contour->size());
			for(int j=0; j<contours[i].size(); j++)
				contours[i][j] = (*contour)[j];
		}
		cv::drawContours(img, contours, -1, cv::Scalar(255,0,0), 2);

		contours.clear();
		contours.resize(repeatObjects.size());
		for(int i=0; i<repeatObjects.size(); i++)
		{
			region = static_pointer_cast<TrackedRegion>(repeatObjects[i]);
			const vector<cv::Point2f> *contour = &region->getContour();;
			contours[i].resize(contour->size());
			for(int j=0; j<contours[i].size(); j++)
				contours[i][j] = (*contour)[j];
		}
		cv::drawContours(img, contours, -1, cv::Scalar(0,0,255), 2);
	}
	else
	{
		for(int i=0; i<newObjects.size(); i++)
			circle(img, newObjects[i]->getLocation(), 4, cv::Scalar(255,0,0), -1);

		for(int i=0; i<repeatObjects.size(); i++)
			circle(img, repeatObjects[i]->getLocation(), 4, cv::Scalar(0,0,255), -1);
	}
}

void ObjectTracker::onFeaturesFound(const shared_ptr<ImageFeatureData> &data)
{
	mMutex_featureData.lock();
	mFeatureData = data;
	mMutex_featureData.unlock();
	mNewFeatureDataAvailable = true;
}

void ObjectTracker::onRegionsFound(const shared_ptr<ImageRegionData> &data)
{
	mMutex_featureData.lock();
	mRegionData = data;
	mMutex_featureData.unlock();
	mNewRegionDataAvailable = true;
}

void ObjectTracker::matchify(const std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedObject>> &curObjects,
						     std::vector<ICSL::Quadrotor::ObjectMatch> &goodMatches,
						     std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedObject>> &repeatPoints,
						     std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedObject>> &newPoints,
						     const TNT::Array2D<double> Sn,
						     const TNT::Array2D<double> SnInv,
						     double probNoCorr,
						     const ICSL::Quadrotor::Time &imageTime,
							 Array2D<double> &C,
							 vector<Array2D<double>> &SdInvmdList,
							 vector<Array2D<double>> &SaInvList,
							 vector<Array2D<double>> &SaList)
{
	goodMatches.clear();
	repeatPoints.clear();
	newPoints.clear();
	/////////////////// Establish correspondence based on postiion ///////////////////////
//	Array2D<double> C = TrackedObject::calcCorrespondence(mTrackedObjects, curObjects, Sn, SnInv, probNoCorr);
	TrackedObject::calcCorrespondence(C, mTrackedObjects, curObjects, Sn, SnInv, SdInvmdList, SaInvList, SaList, probNoCorr);

	///////////////////  make matches ///////////////////////
	shared_ptr<TrackedObject> toPrev, toCur;
	int N1 = mTrackedObjects.size();
	int N2 = curObjects.size();
	vector<bool> prevMatched(N1, false);
	vector<bool> curMatched(N2, false);
//	vector<cv::Point2f> offsets;
	float matchThreshold = 0.6;
	for(int i=0; i<N1; i++)
	{
		if(N2 == 0 || C[i][N2] > 1.0-matchThreshold)
			continue; // this object can't pass the match threshold

		toPrev = mTrackedObjects[i];

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

		if(maxScore < matchThreshold)
			continue;

		curMatched[maxIndex] = true;
		toCur = curObjects[maxIndex];

//		cv::Point offset = toCur->getLocation()-toPrev->getLocation();
//		offsets.push_back(offset);

		ObjectMatch m;
		m.prevPos= toPrev->getLocation();
		m.curPos= toCur->getLocation();
		m.score = C[i][maxIndex];
		goodMatches.push_back(m);
//		toPrev->markFound(imageTime,toCur->getLocation());
		toPrev->markFound(toCur);
		repeatPoints.push_back(toPrev);
		toCur->kill();
	}

	vector<int> dupIndices;
	for(int j=0; j<curObjects.size(); j++)
		if(!curMatched[j])
		{
			// first check to see if we didn't match because there
			// were too many similar objects
			bool addMe = true;
			if(C.dim1() >= N1 && C.dim2() >= j && C[N1][j] < 0.5)
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
				newPoints.push_back(curObjects[j]);
			else
				curObjects[j]->kill();
		}

	// sort and remove repeats 
	sort(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator endIter = unique(dupIndices.begin(), dupIndices.end());
	vector<int>::const_iterator iter = dupIndices.begin();
	while(iter != endIter)
	{
		mTrackedObjects[(*iter)]->kill();
		iter++;
	}

	sort(mTrackedObjects.begin(), mTrackedObjects.end(), TrackedObject::sortAlivePredicate);
	while(mTrackedObjects.size() > 0 && !mTrackedObjects.back()->isAlive() )
		mTrackedObjects.pop_back();

	for(int i=0; i<newPoints.size(); i++)
		mTrackedObjects.push_back(newPoints[i]);
}

void ObjectTracker::doSimilarityCheck(vector<shared_ptr<TrackedObject>> &objects)
{
	double probNoCorr = 0;
	Array2D<double> C = TrackedObject::calcCorrespondence2(objects, objects, probNoCorr);

	// Now keep only things that don't have confusion
	// if there is confusion, only keep one
	vector<shared_ptr<TrackedObject>> tempList;
	tempList.swap(objects);
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
			if(C[i][j] > 0.2)
				confusionList.push_back(j);

		for(int j=0; j<confusionList.size(); j++)
			isStillGood[confusionList[j]] = false;

		objects.push_back(tempList[i]);
	}
}

} // namespace Quadrotor 
} // namespace ICSL 
