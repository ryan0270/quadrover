#include "VisionProcessor.h"

//#include "tbb/parallel_for.h"

using namespace toadlet;
//using namespace cv;
using toadlet::egg::String;
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

	mImgProcTimeUS = 0;

	mLastProcessTime.setTimeMS(0);

	mNewImageReady_featureMatch = mNewImageReady_targetFind = false;

	mImgBufferMaxSize = 10;
	
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
	System sys;
	while(!mFinished)
	{
		Log::alert("VisionProcessor waiting");
		sys.msleep(100); // this can take a while if we are saving a lot of images
	}

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
	if(mCascadeClassifier.load("/sdcard/RoverService/lappyCascade.xml"))
		Log::alert("Cascade classifier loaded");
	else
		Log::alert("Failed to load cascade classifier");
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

	class : public Thread{
		public:
		void run(){parent->runTargetFinder();}
		VisionProcessor *parent;
	} targetFinderThread;
	targetFinderThread.parent = this;
	targetFinderThread.start();

	vector<cv::KeyPoint> circles;
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
			mMutex_imageSensorData.lock();
			mImageDataPrev = mImageDataCur;
			mImageDataCur = mImageDataNext;
			mImageDataPrev->lock(); mImageDataCur->lock();
			double dt = Time::calcDiffUS(mImageDataPrev->timestamp, mImageDataCur->timestamp)/1.0e6;
			mImageDataPrev->unlock(); 
			mMutex_imageSensorData.unlock();

			mCurImage = *(mImageDataCur->img);
			mCurImageGray.create(mImageDataCur->img->rows, mImageDataCur->img->cols, mImageDataCur->img->type());
			mImageDataCur->unlock();

			Time procStart;
			cvtColor(mCurImage, mCurImageGray, CV_BGR2GRAY);

			vector<vector<cv::Point2f> > points = getMatchingPoints(mCurImageGray);
			mFeatureMatchBuffer.push_back(points);

			shared_ptr<cv::Mat> imgAnnotated(new cv::Mat());
			mCurImage.copyTo(*imgAnnotated);
			drawMatches(points, *imgAnnotated);

			mMutex_data.lock();
			circles = mLastCircles;
			mMutex_data.unlock();
			drawTarget(circles, *imgAnnotated);

			mCurImageAnnotated = imgAnnotated;

			shared_ptr<ImageMatchData> data(new ImageMatchData());
			data->dt = dt;
			data->featurePoints = points;
			data->imgData0 = mImageDataPrev;
			data->imgData1 = mImageDataCur;
			data->imgAnnotated = imgAnnotated;
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onImageProcessed(data);

			mImgProcTimeUS = procStart.getElapsedTimeUS();
			{
				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME_FEATURE_MATCH + "\t" + mImgProcTimeUS;
				mMutex_logger.lock();
				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();

				String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_NUM_FEATURE_POINTS+"\t"+points[0].size();
				mMutex_logger.lock();
				mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();
			}

			if(mLogImages && mMotorOn)
			{
//				mMutex_buffers.lock();
//				mImgDataBuffer.push_back(shared_ptr<SensorDataImage>(mImageDataCur));
//				while(mImgDataBuffer.size() > mImgBufferMaxSize)
//					mImgDataBuffer.pop_front();
//
//				mImgMatchDataBuffer.push_back(shared_ptr<ImageMatchData>(data));
//				while(mImgMatchDataBuffer.size() > mImgBufferMaxSize)
//					mImgMatchDataBuffer.pop_front();
//				mMutex_buffers.unlock();
			}

		}

		System::msleep(1);
	}

	targetFinderThread.join();

//	mQuadLogger->saveImageBuffer(mImgDataBuffer, mImgMatchDataBuffer);
	mQuadLogger->saveFeatureMatchBuffer(mFeatureMatchBuffer);

	mImgDataBuffer.clear();
	mImgMatchDataBuffer.clear();

	mFinished = true;
}

void VisionProcessor::runTargetFinder()
{
	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	cv::Mat curImage, curImageGray;
	shared_ptr<SensorDataImage> curImageData;
	vector<cv::KeyPoint> circles;
	uint32 imgProcTimeUS;
	while(mRunning)
	{
		if(mNewImageReady_targetFind)
		{
			mNewImageReady_targetFind = false;
			curImageData = mImageDataNext;

			curImageData->lock();
			curImage = *(curImageData->img);
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
				data->circleLocs = circles;
				data->imgData = curImageData;
				for(int i=0; i<mListeners.size(); i++)
					mListeners[i]->onImageTargetFound(data);
			}

			imgProcTimeUS = procStart.getElapsedTimeUS();
			{
				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME_TARGET_FIND + "\t" + imgProcTimeUS;
				mMutex_logger.lock();
				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();

				if(circles.size() > 0)
				{
					String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_IMG_TARGET_POINTS+"\t";
					for(int i=0; i<circles.size(); i++)
						str2 = str2+circles[i].pt.x+"\t"+circles[i].pt.y+"\t";
					mMutex_logger.lock();
					mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);
					mMutex_logger.unlock();
				}
			}

		}

		System::msleep(1);
	}
}

vector<vector<cv::Point2f> > VisionProcessor::getMatchingPoints(cv::Mat const &img)
{
	mMutex_matcher.lock();
	int result = mFeatureMatcher.Track(img, false);
	vector<vector<cv::Point2f> > points(2);

	for(int i=0; i<mFeatureMatcher.vMatches.size(); i++)
	{
		Matcher::p_match match = mFeatureMatcher.vMatches[i];
		points[0].push_back(cv::Point2f(match.u1p, match.v1p));
		points[1].push_back(cv::Point2f(match.u1c, match.v1c));
	}
	mMutex_matcher.unlock();

	return points;
}

vector<cv::KeyPoint> VisionProcessor::findCircles(cv::Mat const &img)
{
	cv::Mat pyrImg;
	cv::equalizeHist(img, pyrImg);
	cv::pyrDown(pyrImg, pyrImg);
	cv::pyrDown(pyrImg, pyrImg);
 	double scale = (double)pyrImg.rows/img.rows;

	cv::SimpleBlobDetector::Params params;
	params.minThreshold = 150;
	params.maxThreshold = 220;
	params.thresholdStep = 10;
	params.minRepeatability = 1;
	params.minDistBetweenBlobs = 20;
	params.filterByColor = true;
	params.blobColor = 0;
	params.filterByArea = true;
	params.minArea = 100;
	params.maxArea = pyrImg.rows*pyrImg.cols;
	params.filterByCircularity = true;
	params.minCircularity = 0.7;
	params.maxCircularity = 1.1;
	params.filterByInertia = false;
	params.filterByConvexity = true;
	params.minConvexity = 0.9;
	params.maxConvexity = 1.1;

	cv::SimpleBlobDetector blobby(params);
	vector<cv::KeyPoint> keypoints;
	blobby.detect(pyrImg, keypoints);

	// just grab the biggest one
	sort(keypoints.begin(), keypoints.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2){return kp1.size > kp2.size;});

	vector<cv::KeyPoint> circs;
	if(keypoints.size() > 0)
	{
		cv::KeyPoint kp = keypoints[0];
		kp.pt = 1.0/scale*kp.pt;
		kp.size /= scale;
		circs.push_back(kp);
	}

	return circs;

	// Try to get the 4 circles of interest
// 	sort(keypoints.begin(), keypoints.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2){return kp1.size > kp2.size;});
// 	list<cv::KeyPoint> circleKeypoints;
// 	vector<cv::KeyPoint>::const_iterator kpIter = keypoints.begin();
// 	double expectedSize = -1;;
// 	while(circleKeypoints.size() < 4 && kpIter != keypoints.end())
// 	{
// 		cv::KeyPoint kp = *kpIter;
// 		if(circleKeypoints.size() == 0)
// 		{
// 			expectedSize = kp.size;
// 			circleKeypoints.push_back(kp);
// 		}
// 		else
// 		{
// 			if(abs( (double)(kp.size-expectedSize)/expectedSize ) < 0.3)
// 				circleKeypoints.push_back(kp);
// 			else
// 			{
// 				circleKeypoints.push_back(kp);
// 				circleKeypoints.pop_front();
// 				expectedSize = kp.size;
// 			}
// 		}
// 		kpIter++;
// 	}
// 
// 	if(circleKeypoints.size() < 4)
// 	{
// 		circleKeypoints.clear();
// 		return vector<cv::KeyPoint>();
// 	}
// 
// 	// scale results back to original image size and refine the location
// 	list<cv::KeyPoint>::iterator circIter = circleKeypoints.begin();
// 	vector<cv::KeyPoint> localCircles, circs;
// 	while(circIter != circleKeypoints.end())
// 	{
// 		cv::KeyPoint kp = *circIter;
// 		kp.pt = 1.0/scale*kp.pt;
// 		kp.size /= scale;
// 
// // This stuff takes too long
// //		float x = max((double)0,kp.pt.x-2.0*kp.size);
// //		float y = max((double)0,kp.pt.y-2.0*kp.size);
// //		float width = min((double)img.cols-x-1, 4.0*kp.size);
// //		float height = min((double)img.rows-y-1, 4.0*kp.size);
// //		cv::Rect mask(x, y, width, height);
// //		params.minArea = PI*kp.size*kp.size/2;
// //		params.maxArea = width*height;
// //		params.filterByCircularity = false;
// //		localCircles.clear();
// //		cv::SimpleBlobDetector(params).detect(img(mask), localCircles);
// //		if(localCircles.size() > 0)
// //		{
// //			sort(localCircles.begin(), localCircles.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2){return kp1.size > kp2.size;});
// //			int j=0;
// //			bool found = false;
// //			while(!found && j < localCircles.size())
// //			{
// //				if( (localCircles[j].size - kp.size)/kp.size < 0.2 )
// //				{
// //					kp = localCircles[j];
// //					kp.pt = kp.pt+cv::Point2f(x, y);
// //					found = true;
// //				}
// //				j++;
// //			}
// //		}
// 
// 		circs.push_back(kp);
// 		circIter++;
// 	}
// 
// 	// need to get points ordered to calculated square sides
// //	sort(circs.begin(), circs.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2){return norm(kp1.pt) < norm(kp2.pt);});
// 	cv::Point2f p0 = circs[0].pt;
// 	sort(circs.begin()+1, circs.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2){return norm(kp1.pt-p0) < norm(kp2.pt-p0);});
// 	cv::Point2f p1 = circs[1].pt;
// 	sort(circs.begin()+2, circs.end(), [&](cv::KeyPoint kp1, cv::KeyPoint kp2){return norm(kp1.pt-p1) < norm(kp2.pt-p1);});
// 
// 	double l1 = norm(circs[0].pt-circs[1].pt);
// 	double l2 = norm(circs[1].pt-circs[2].pt);
// 	double l3 = norm(circs[2].pt-circs[3].pt);
// 	double l4 = norm(circs[3].pt-circs[0].pt);
// 	double mean = 0.25*(l1+l2+l3+l4);
// 	if( abs(l1-mean)/mean > 0.1 ||
// 		abs(l2-mean)/mean > 0.1 ||
// 		abs(l3-mean)/mean > 0.1 ||
// 		abs(l4-mean)/mean > 0.1)
// 			circs.clear(); // too much variation so something must have been wrong
// 
// 	return circs;
}

void VisionProcessor::drawMatches(vector<vector<cv::Point2f> > const &points, cv::Mat &img)
{
	for(int i=0; i<points[0].size(); i++)
	{
		cv::Point2f p1 = points[0][i];
		cv::Point2f p2 = points[1][i];
 		circle(img,p1,3,cv::Scalar(0,255,0),-1);
 		circle(img,p2,3,cv::Scalar(255,0,0),-1);
 		line(img,p1,p2,cv::Scalar(255,255,255),1);
	}
}

void VisionProcessor::drawTarget(vector<cv::KeyPoint> const &circles, cv::Mat &img)
{
	for(int i=0; i<circles.size(); i++)
		circle(img, circles[i].pt, circles[i].size, cv::Scalar(0,0,255), 3);
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
		mLastImgFoundTime.setTime();
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

void VisionProcessor::onNewSensorUpdate(shared_ptr<SensorData> const &data)
{
	if(data->type == SENSOR_DATA_TYPE_IMAGE)
	{
		mMutex_imageSensorData.lock();
		mImageDataNext = static_pointer_cast<SensorDataImage>(data);
		mMutex_imageSensorData.unlock();
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
		mImgDataBuffer.clear();
	}
}

void VisionProcessor::onNewCommImgBufferSize(int size)
{
	mMutex_buffers.lock();
	mImgBufferMaxSize = size;
	mMutex_buffers.unlock();
}

void VisionProcessor::onNewCommVisionRatioThreshold(float h)
{
	mMutex_matcher.lock();
	mFeatureMatcher.params.ratio_threshold = h;
	mMutex_matcher.unlock();
	Log::alert(String()+"Matcher ratio threshold set to "+h);
}

void VisionProcessor::onNewCommVisionMatchRadius(float r)
{
	mMutex_matcher.lock();
	mFeatureMatcher.params.match_radius = r;
	mMutex_matcher.unlock();
	Log::alert(String()+"Matcher match radius set to "+r);
}

void VisionProcessor::onNewCommLogClear()
{
	mMutex_buffers.lock();
	mFeatureMatchBuffer.clear();
	mImgDataBuffer.clear();
	mImgMatchDataBuffer.clear();
	mMutex_buffers.unlock();
}

void VisionProcessor::onCommConnectionLost()
{
	// stop logging images so I don't loose the ones from the flight
	mLogImages = false;
}

} // namespace Quadrotor
} // namespace ICSL
