#include "VisionProcessor.h"

//#include "tbb/parallel_for.h"

using namespace toadlet;
//using namespace cv;
//using toadlet::egg::String;
using namespace TNT;
using namespace ICSL::Constants;

namespace ICSL {
namespace Quadrotor{

VisionProcessor::VisionProcessor() :
	mFeatureMatcher("FAST_GRID","BRIEF")
{
	mRunning = false;
	mFinished = true;
	mUseIbvs = false;
	mFirstImageProcessed = false;
	int width = 640;
	int height = 480;
	mCurImage.create(height, width, CV_8UC3); mCurImage = cv::Scalar(0);
	mCurImageGray.create(height, width, CV_8UC1); mCurImageGray = cv::Scalar(0);
	mCurImageAnnotated = NULL;

	mImageProcTimeUS = 0;

	mLastProcessTime.setTimeMS(0);

	mNewImageReady_featureMatch = mNewImageReady_targetFind = false;

	mImageBufferMaxSize = 10;
	
	mLogImages = false;

	mImageDataPrev = mImageDataCur = mImageDataNext = NULL;

	mMotorOn = false;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
}

void VisionProcessor::shutdown()
{
	Log::alert("-------------------------- Vision processor shutdown started ----------------------");
	mRunning = false;
	while(!mFinished)
		System::msleep(10);

	mImageDataCur = NULL;
	mImageDataPrev = NULL;
	mImageDataNext = NULL;

	mMutex_image.lock();
	mCurImage.release();
	mCurImageGray.release();
	mMutex_image.unlock();

	Log::alert("-------------------------- Vision processor done ----------------------");
}

void VisionProcessor::initialize()
{
}

void VisionProcessor::getLastImage(cv::Mat *outImage)
{
	mMutex_image.lock();
	outImage->create(mCurImage.rows,mCurImage.cols,mCurImage.type());
	mCurImage.copyTo(*outImage);
	mMutex_image.unlock();
}

void VisionProcessor::getLastImageAnnotated(cv::Mat *outImage)
{
	mMutex_image.lock();
	outImage->create(mCurImage.rows,mCurImage.cols,mCurImage.type());
	if(mCurImageAnnotated != NULL)
		mCurImageAnnotated->copyTo(*outImage);
	else
		(*outImage) = cv::Scalar(0);
	mMutex_image.unlock();
}

void VisionProcessor::run()
{
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	thread targetFinderTh(&VisionProcessor::runTargetFinder, this);

	vector<BlobDetector::Blob> circles;
	TNT::Array2D<double> attPrev(3,1,0.0), attCur(3,1,0.0);
	vector<vector<cv::Point2f> > points;
	while(mRunning)
	{
		if(mNewImageReady_featureMatch && mImageDataCur == NULL)
		{
			mNewImageReady_featureMatch = false;
			mImageDataCur = mImageDataNext;
		}
		else if(mNewImageReady_featureMatch)
		{
			mNewImageReady_featureMatch = false;
			mMutex_imageData.lock();
			mImageDataPrev = mImageDataCur;
			mImageDataCur = mImageDataNext;
			mImageDataPrev->lock(); mImageDataCur->lock();
			double dt = Time::calcDiffUS(mImageDataPrev->timestamp, mImageDataCur->timestamp)/1.0e6;
			attPrev.inject(mImageDataPrev->att);
			attCur.inject(mImageDataCur->att);
			mImageDataPrev->unlock(); 
			mMutex_imageData.unlock();

			mCurImage = *(mImageDataCur->image);
			mCurImageGray.create(mImageDataCur->image->rows, mImageDataCur->image->cols, mImageDataCur->image->type());
			mImageDataCur->unlock();

			Time procStart;
			cvtColor(mCurImage, mCurImageGray, CV_BGR2GRAY);

			points.clear();
			points = getMatchingPoints(mCurImageGray);
//			if(points[0].size() > 0)
//			{
//				mFeatureMatchBuffer.push_back(points);
//				mFeatureMatchTimeBuffer.push_back(mImageDataCur->timestamp);
//				mFeatureMatchDTBuffer.push_back(dt);
//				mFeatureMatchAttPrevBuffer.push_back(attPrev.copy());
//				mFeatureMatchAttCurBuffer.push_back(attCur.copy());
//			}

			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
			mCurImage.copyTo(*imageAnnotated);
			drawMatches(points, *imageAnnotated);

			mMutex_data.lock();
			circles = mLastCircles;
			mMutex_data.unlock();
			drawTarget(circles, *imageAnnotated);

			mCurImageAnnotated = imageAnnotated;
			
			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
			imageAnnotatedData->imageAnnotated = imageAnnotated;
			imageAnnotatedData->imageDataSource = mImageDataCur;

			shared_ptr<ImageMatchData> data(new ImageMatchData());
//			data->dt = dt;
			data->featurePoints = points;
			data->imageData0 = mImageDataPrev;
			data->imageData1 = mImageDataCur;
			data->imageAnnotated = imageAnnotatedData;
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onImageProcessed(data);

			mImageProcTimeUS = procStart.getElapsedTimeUS();
//			if(mQuadLogger != NULL)
//			{
//				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME_FEATURE_MATCH + "\t" + mImageProcTimeUS;
//				mMutex_logger.lock();
//				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
//				mMutex_logger.unlock();
//
//				String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_NUM_FEATURE_POINTS+"\t";
//				str2 = str2+(points.size() > 0 ? points[0].size() : 0);
//				mMutex_logger.lock();
//				mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);
//				mMutex_logger.unlock();
//			}

			if(mLogImages && mMotorOn)
			{
//				mMutex_buffers.lock();
//				mImageDataBuffer.push_back(shared_ptr<DataImage>(mImageDataCur));
//				while(mImageDataBuffer.size() > mImageBufferMaxSize)
//					mImageDataBuffer.pop_front();
//
//				mImageMatchDataBuffer.push_back(shared_ptr<ImageMatchData>(data));
//				while(mImageMatchDataBuffer.size() > mImageBufferMaxSize)
//					mImageMatchDataBuffer.pop_front();
//				mMutex_buffers.unlock();
			}

		}

		System::msleep(1);
	}

	targetFinderTh.join();

//	mQuadLogger->saveImageBuffer(mImageDataBuffer, mImageMatchDataBuffer);
//	mQuadLogger->saveFeatureMatchBuffer(mFeatureMatchBuffer, 
//										mFeatureMatchTimeBuffer, 
//										mFeatureMatchDTBuffer, 
//										mFeatureMatchAttPrevBuffer, 
//										mFeatureMatchAttCurBuffer);

	mImageDataBuffer.clear();
	mImageMatchDataBuffer.clear();

	mFinished = true;
}

void VisionProcessor::runTargetFinder()
{
	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	cv::Mat curImage, curImageGray;
	shared_ptr<DataImage> curImageData;
	vector<BlobDetector::Blob> circles;
	uint32 imageProcTimeUS;
	while(mRunning)
	{
		if(mNewImageReady_targetFind)
		{
			mNewImageReady_targetFind = false;
			curImageData = mImageDataNext;

			curImageData->lock();
			curImage = *(curImageData->image);
			curImageData->unlock();
			curImageGray.create(curImage.rows, curImage.cols, curImage.type());

			Time procStart;
			cvtColor(curImage, curImageGray, CV_BGR2GRAY);

			circles = findCircles(curImageGray);

			mMutex_data.lock();
			mLastCircles = circles;
			mMutex_data.unlock();

			if(circles.size() > 0)
			{
				shared_ptr<ImageTargetFindData> data(new ImageTargetFindData());
				data->circleBlobs = circles;
				data->imageData = curImageData;
				for(int i=0; i<mListeners.size(); i++)
					mListeners[i]->onImageTargetFound(data);
			}

			imageProcTimeUS = procStart.getElapsedTimeUS();
			{
				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME_TARGET_FIND + "\t" + imageProcTimeUS;
				mMutex_logger.lock();
				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();

				if(circles.size() > 0)
				{
					String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_IMG_TARGET_POINTS+"\t";
					for(int i=0; i<circles.size(); i++)
						str2 = str2+circles[i].location.x+"\t"+circles[i].location.y+"\t";
					mMutex_logger.lock();
					mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);
					mMutex_logger.unlock();
				}
			}

		}

		System::msleep(20); // run this a little slower to save computation
	}
}

vector<vector<cv::Point2f> > VisionProcessor::getMatchingPoints(cv::Mat const &image)
{
	mMutex_matcher.lock();
	int result = mFeatureMatcher.Track(image, false);
	vector<vector<cv::Point2f> > points(2);

	Matcher::p_match match;
//	for(int i=0; i<mFeatureMatcher.vMatches.size(); i++)
//	{
//		match = mFeatureMatcher.vMatches[i];
//		points[0].push_back(cv::Point2f(match.u1p, match.v1p));
//		points[1].push_back(cv::Point2f(match.u1c, match.v1c));
//	}
	points[0].resize(mFeatureMatcher.vMatches.size());
	points[1].resize(mFeatureMatcher.vMatches.size());
	for(int i=0; i<mFeatureMatcher.vMatches.size(); i++)
	{
		match = mFeatureMatcher.vMatches[i];
		points[0][i].x = match.u1p;
		points[0][i].y = match.v1p;
		points[1][i].x = match.u1c;
		points[1][i].y = match.v1c;
	}
	mMutex_matcher.unlock();

	if(points[0].size() < 2)
		return points;
	
int prevPointCount = points[0].size();

	// filter out possible bad matches
	vector<double> distances(points[0].size());
	cv::Point2f p1, p2;
	for(int i=0; i<points[0].size(); i++)
	{
		p1 = points[0][i];
		p2 = points[1][i];
		distances[i] = sqrt( pow(p1.x-p2.x,2) + pow(p1.y-p2.y,2) );
	}
	int medIndex = distances.size()/2;
	// this finds the median
	vector<double> distancesCopy(distances);
	nth_element(distancesCopy.begin(), distancesCopy.begin()+medIndex, distancesCopy.end(),
						[&](double const &a, double const &b){return a < b;});
	double medDist = distancesCopy[medIndex];
	double minDelta = 100;
	double lowEnd = min(0.5f*medDist,medDist-minDelta);
	double highEnd = max(1.5f*medDist,medDist+minDelta);
	vector<vector<cv::Point2f> > tempPoints(2);
	points.swap(tempPoints);
	for(int i=0; i<distances.size(); i++)
		if( lowEnd < distances[i] && distances[i] < highEnd)
		{
			points[0].push_back(tempPoints[0][i]);
			points[1].push_back(tempPoints[0][i]);
		}

	return points;
}

vector<BlobDetector::Blob> VisionProcessor::findCircles(cv::Mat const &image)
{
	cv::Mat pyrImage;
	cv::equalizeHist(image, pyrImage);
	cv::pyrDown(pyrImage, pyrImage);
	cv::pyrDown(pyrImage, pyrImage);
 	double scale = (double)pyrImage.rows/image.rows;

	BlobDetector blobby;
//	blobby.minThreshold = 50;
	blobby.minThreshold = 150;
	blobby.maxThreshold = 220;
	blobby.thresholdStep = 10;
	blobby.minRepeatability = 1;
	blobby.minDistBetweenBlobs = 20;
	blobby.filterByColor = true;
	blobby.blobColor = 0;
	blobby.filterByArea = true;
	blobby.minArea = 125;
//	blobby.minArea = 500;
	blobby.maxArea = pyrImage.rows*pyrImage.cols;
	blobby.filterByCircularity = true;
	blobby.minCircularity = 0.85;
//	blobby.maxCircularity = 1.1;
	blobby.filterByInertia = false;
	blobby.filterByConvexity = true;
	blobby.minConvexity = 0.9;
//	blobby.maxConvexity = 1.1;

	vector<BlobDetector::Blob> blobs;
	blobby.detectImpl(pyrImage, blobs);

	// just grab the biggest one
//	sort(keypoints.begin(), keypoints.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2){return kp1.size > kp2.size;});
	sort(blobs.begin(), blobs.end(), [&](BlobDetector::Blob b1, BlobDetector::Blob b2){return b1.radius > b2.radius;});

	vector<BlobDetector::Blob> circs;
	if(blobs.size() > 0)
	{
		// 		base camera
//		double k1 = 0.057821692482247215;
//		double k2 = -0.30298698825546139;
//		double p1 = -0.0049942077937641669;
//		double p2 = 0.0013584538237561394;
//		double k3 = 0.32018957065239517;
		double cx = 318.2;
		double cy = 249.0;
//		//		macro lens
//		double k1 = -1.6684484727208121e-01;
//		double k2 = 2.3371666312763096e-02;
//		double p1 = 5.8454330481278330e-03;
//		double p2 = -1.6372156794818059e-03;
//		double k3 = 2.4911796969280088e-03;
//		double cx = 640/2.0;
//		double cy = 480/2.0;

		cv::Point2f cPt(cx*scale, cy*scale);
		double f = 3.7*640/5.76*scale;

		BlobDetector::Blob b = blobs[0];
		cv::Point2f pt;
		double ptSq;
		b.contourCorrected.clear();
		for(int i=0; i<b.contour.size(); i++)
		{
			// remove distortion and scale back up
			pt = b.contour[i];
//			pt = 1.0/f*(pt-cPt);
//			ptSq = pow(pt.x,2)+pow(pt.y,2);
//			pt = (1+k1*ptSq+k2*pow(ptSq,2)+k3*pow(ptSq,3))*pt;
//			ptSq = pow(pt.x,2)+pow(pt.y,2);
//			pt.x += 2*p1*pt.x*pt.y+p2*(ptSq+2*pow(pt.x,2));
//			pt.y += 2*p2*pt.x*pt.y+p1*(ptSq+2*pow(pt.y,2));
//			pt = f*pt+cPt;
			b.contourCorrected.push_back(1.0/scale*pt);

			b.contour[i] = 1.0/scale*b.contour[i];
		}

		cv::Moments moms = cv::moments(cv::Mat(b.contour));
		cv::Moments momsCorrected = cv::moments(cv::Mat(b.contourCorrected));

		b.location = cv::Point2f(moms.m10 / moms.m00, moms.m01 / moms.m00);
		b.locationCorrected = cv::Point2f(momsCorrected.m10 / momsCorrected.m00, momsCorrected.m01 / momsCorrected.m00);

		b.area = cv::contourArea(b.contour);
		b.areaCorrected = cv::contourArea(b.contourCorrected);

		vector<double> dists, distsCorrected;
		for (size_t pointIdx = 0; pointIdx < b.contour.size(); pointIdx++)
		{
			pt = b.contour[pointIdx];
			dists.push_back(norm(b.location - pt));

			pt = b.contourCorrected[pointIdx];
			distsCorrected.push_back(norm(b.locationCorrected-pt));
		}
		sort(dists.begin(), dists.end());
		sort(distsCorrected.begin(), distsCorrected.end());
		b.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
		b.radiusCorrected = (distsCorrected[(distsCorrected.size() - 1) / 2] + distsCorrected[distsCorrected.size() / 2]) / 2.;

		circs.push_back(b);
	}

	return circs;
}

void VisionProcessor::drawMatches(vector<vector<cv::Point2f> > const &points, cv::Mat &image)
{
	if(points.size() == 0)
		return;
	cv::Point2f p1, p2;
	for(int i=0; i<points[0].size(); i++)
	{
		p1 = points[0][i];
		p2 = points[1][i];
 		circle(image,p1,3,cv::Scalar(0,255,0),-1);
 		circle(image,p2,3,cv::Scalar(255,0,0),-1);
 		line(image,p1,p2,cv::Scalar(255,255,255),1);
	}
}

void VisionProcessor::drawTarget(vector<BlobDetector::Blob> const &circles, cv::Mat &image)
{
	vector<vector<cv::Point> > contours;
	for(int i=0; i<circles.size(); i++)
	{
		contours.push_back(vector<cv::Point>());
		for(int j=0; j<circles[i].contour.size(); j++)
			contours[i].push_back(circles[i].contour[j]);
	}
	cv::drawContours(image, contours, -1, cv::Scalar(0,0,255), 3);
//	for(int i=0; i<circles.size(); i++)
//		circle(image, circles[i].pt, circles[i].size, cv::Scalar(0,0,255), 3);
}

void VisionProcessor::enableIbvs(bool enable)
{
	mUseIbvs = enable;
	if(mUseIbvs)
	{
		Log::alert("Turning IBVS on");
		String str = String()+ mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IBVS_ENABLED + "\t";
		mMutex_logger.lock();
		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
		mMutex_logger.unlock();
		mLastImageFoundTime.setTime();
		mFirstImageProcessed = false;
	}
	else
	{
		Log::alert("Turning IBVS off");
		String str = String() + mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IBVS_DISABLED + "\t";
		mMutex_logger.lock();
		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
		mMutex_logger.unlock();
		mFirstImageProcessed = false;
	}
}

Collection<int> VisionProcessor::getVisionParams()
{
	Collection<int> p;

	return p;
}

void VisionProcessor::setVisionParams(Collection<int> const &p)
{
}

void VisionProcessor::onNewSensorUpdate(shared_ptr<IData> const &data)
{
	if(data->type == DATA_TYPE_IMAGE)
	{
		mMutex_imageData.lock();
		mImageDataNext = static_pointer_cast<DataImage>(data);
		mMutex_imageData.unlock();
		mNewImageReady_featureMatch = mNewImageReady_targetFind = true;
	}
}

void VisionProcessor::onNewCommLogMask(uint32 mask)
{
	if((mask & LOG_FLAG_CAM_IMAGES) > 0)
		mLogImages = true;
	else
	{
		mLogImages = false; 
		mImageDataBuffer.clear();
	}
}

void VisionProcessor::onNewCommImageBufferSize(int size)
{
	mMutex_buffers.lock();
	mImageBufferMaxSize = size;
	mMutex_buffers.unlock();
}

//void VisionProcessor::onNewCommVisionRatioThreshold(float h)
//{
//	Log::alert("onNewCommVisionRatioThreshold not implemented");
//	mMutex_matcher.lock();
//	mFeatureMatcher.params.ratio_threshold = h;
//	mMutex_matcher.unlock();
//	Log::alert(String()+"Matcher ratio threshold set to "+h);
//}

//void VisionProcessor::onNewCommVisionMatchRadius(float r)
//{
//	Log::alert("onNewCommVisionMatchRadius not implemented");
//	mMutex_matcher.lock();
//	mFeatureMatcher.params.match_radius = r;
//	mMutex_matcher.unlock();
//	Log::alert(String()+"Matcher match radius set to "+r);
//}

void VisionProcessor::onNewCommLogClear()
{
	mMutex_buffers.lock();
	mFeatureMatchBuffer.clear();
	mFeatureMatchTimeBuffer.clear();
	mFeatureMatchDTBuffer.clear();
	mFeatureMatchAttPrevBuffer.clear();
	mFeatureMatchAttCurBuffer.clear();
	mImageDataBuffer.clear();
	mImageMatchDataBuffer.clear();
	mMutex_buffers.unlock();
}

void VisionProcessor::onCommConnectionLost()
{
	// stop logging images so I don't loose the ones from the flight
	mLogImages = false;
}

} // namespace Quadrotor
} // namespace ICSL
