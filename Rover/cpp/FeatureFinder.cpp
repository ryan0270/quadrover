#include "FeatureFinder.h"

//#include "tbb/parallel_for.h"

using namespace toadlet;
//using namespace cv;
using toadlet::egg::String;
using namespace TNT;
using namespace ICSL::Constants;

namespace ICSL {
namespace Quadrotor{

FeatureFinder::FeatureFinder()
{
	mRunning = false;
	mFinished = true;
	mUseIbvs = false;
	mFirstImageProcessed = false;
	mCurImageAnnotated = NULL;

	mImageProcTimeUS = 0;

	mLastProcessTime.setTimeMS(0);

	mNewImageReady = mNewImageReady_targetFind = false;

	mLogImages = false;

	mImageDataNext = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
}

void FeatureFinder::shutdown()
{
	Log::alert("-------------------------- Feature finder shutdown started ----------------------");
	mRunning = false;
	while(!mFinished)
		System::msleep(10);

	mImageDataNext = NULL;

	Log::alert("-------------------------- Feature finder done ----------------------");
}

void FeatureFinder::initialize()
{
}

void FeatureFinder::getLastImageAnnotated(cv::Mat *outImage)
{
	if(mImageAnnotatedLast != NULL)
	{
		mImageAnnotatedLast->lock();
		shared_ptr<cv::Mat> image = mImageAnnotatedLast->imageAnnotated;
		mImageAnnotatedLast->unlock();
//		outImage->create(img.rows,img.cols,img.type());
		image->copyTo(*outImage);
	}
	else
		(*outImage) = cv::Scalar(0);
}

void FeatureFinder::run()
{
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	vector<cv::Point2f> points;
	shared_ptr<DataImage> imageData;
	cv::Mat curImage, curImageGray, imageAnnotated;
	while(mRunning)
	{
		if(mNewImageReady)
		{
			Time procStart;
			mNewImageReady = false;

			imageData = mImageDataNext;
			curImage = *(imageData->image);
			curImageGray.create(imageData->image->rows, imageData->image->cols, imageData->image->type());

//			cvtColor(curImage, curImageGray, CV_BGR2GRAY);
//
//			points.clear();
//			points = findFeaturePoints(curImageGray);
//
//			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
//			curImage.copyTo(*imageAnnotated);
//			drawPoints(points, *imageAnnotated);
//			
//			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
//			imageAnnotatedData->imageAnnotated = imageAnnotated;
//			imageAnnotatedData->imageDataSource = imageData;
//			mImageAnnotatedLast = imageAnnotatedData;
//
//			shared_ptr<ImageFeatureData> data(new ImageFeatureData());
//			data->featurePoints = points;
//			data->imageData = imageData;
//			data->imageAnnotated = imageAnnotatedData;
//			for(int i=0; i<mListeners.size(); i++)
//				mListeners[i]->onFeaturesFound(data);
//
//			mImageProcTimeUS = procStart.getElapsedTimeUS();
//			if(mQuadLogger != NULL)
//			{
//				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME_FEATURE_MATCH + "\t" + mImageProcTimeUS;
//				mMutex_logger.lock();
//				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
//				mMutex_logger.unlock();
//
//				String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_NUM_FEATURE_POINTS+"\t";
//				str2 = str2+points.size();
//				mMutex_logger.lock();
//				mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);
//				mMutex_logger.unlock();
//			}
		}

		System::msleep(1);
	}

	mFinished = true;
}

vector<cv::Point2f> FeatureFinder::findFeaturePoints(cv::Mat const &image)
{
	vector<cv::Point2f> points;

	Log::alert("Where are you?");

	return points;
}

void FeatureFinder::drawPoints(vector<cv::Point2f> const &points, cv::Mat &image)
{
	if(points.size() == 0)
		return;
	cv::Point2f p1;
	for(int i=0; i<points.size(); i++)
	{
		p1 = points[i];
 		circle(image,p1,3,cv::Scalar(0,255,0),-1);
	}
}

void FeatureFinder::enableIbvs(bool enable)
{
	mUseIbvs = enable;
	if(mUseIbvs)
	{
		Log::alert("Turning IBVS on");
		String str = String()+ mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IBVS_ENABLED + "\t";
		mMutex_logger.lock();
		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
		mMutex_logger.unlock();
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

void FeatureFinder::onNewSensorUpdate(shared_ptr<IData> const &data)
{
	if(data->type == DATA_TYPE_IMAGE)
	{
		mImageDataNext = static_pointer_cast<DataImage>(data);
		mNewImageReady = mNewImageReady_targetFind = true;
	}
}

void FeatureFinder::onNewCommLogMask(uint32 mask)
{
	if((mask & LOG_FLAG_CAM_IMAGES) > 0)
		mLogImages = true;
	else
		mLogImages = false; 
}

void FeatureFinder::onNewCommImageBufferSize(int size)
{
}

void FeatureFinder::onNewCommLogClear()
{
}

void FeatureFinder::onCommConnectionLost()
{
	// stop logging images so I don't loose the ones from the flight
	mLogImages = false;
}

} // namespace Quadrotor
} // namespace ICSL
