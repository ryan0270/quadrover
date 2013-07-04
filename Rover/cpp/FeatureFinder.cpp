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
	int width = 640;
	int height = 480;
	mCurImage.create(height, width, CV_8UC3); mCurImage = cv::Scalar(0);
	mCurImageGray.create(height, width, CV_8UC1); mCurImageGray = cv::Scalar(0);
	mCurImageAnnotated = NULL;

	mImgProcTimeUS = 0;

	mLastProcessTime.setTimeMS(0);

	mNewImageReady = mNewImageReady_targetFind = false;

	mImgBufferMaxSize = 10;
	
	mLogImages = false;

	mImageDataPrev = mImageDataCur = mImageDataNext = NULL;

	mMotorOn = false;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
}

void FeatureFinder::shutdown()
{
	Log::alert("-------------------------- Feature finder shutdown started ----------------------");
	mRunning = false;
	this->join();
//	while(!mFinished)
//	{
//		Log::alert("FeatureFinder waiting");
//		System::msleep(100); // this can take a while if we are saving a lot of images
//	}

	mImageDataCur = NULL;
	mImageDataPrev = NULL;
	mImageDataNext = NULL;

	mMutex_image.lock();
	mCurImage.release();
	mCurImageGray.release();
	mMutex_image.unlock();

	Log::alert("-------------------------- Feature finder done ----------------------");
}

void FeatureFinder::initialize()
{
}

void FeatureFinder::getLastImage(cv::Mat *outImage)
{
	mMutex_image.lock();
	outImage->create(mCurImage.rows,mCurImage.cols,mCurImage.type());
	mCurImage.copyTo(*outImage);
	mMutex_image.unlock();
}

void FeatureFinder::getLastImageAnnotated(cv::Mat *outImage)
{
	mMutex_image.lock();
	outImage->create(mCurImage.rows,mCurImage.cols,mCurImage.type());
	if(mCurImageAnnotated != NULL)
		mCurImageAnnotated->copyTo(*outImage);
	else
		(*outImage) = cv::Scalar(0);
	mMutex_image.unlock();
}

void FeatureFinder::run()
{
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	TNT::Array2D<double> attPrev(3,1,0.0), attCur(3,1,0.0);
	vector<cv::Point2f> points;
	while(mRunning)
	{
		if(mNewImageReady && mImageDataCur == NULL)
		{
			mNewImageReady = false;
			mImageDataCur = mImageDataNext;
		}
		else if(mNewImageReady)
		{
			mNewImageReady = false;
			mMutex_imageData.lock();
			mImageDataPrev = mImageDataCur;
			mImageDataCur = mImageDataNext;
			mImageDataPrev->lock(); mImageDataCur->lock();
			double dt = Time::calcDiffUS(mImageDataPrev->timestamp, mImageDataCur->timestamp)/1.0e6;
			attPrev.inject(mImageDataPrev->att);
			attCur.inject(mImageDataCur->att);
			mImageDataPrev->unlock(); 
			mMutex_imageData.unlock();

			mCurImage = *(mImageDataCur->img);
			mCurImageGray.create(mImageDataCur->img->rows, mImageDataCur->img->cols, mImageDataCur->img->type());
			mImageDataCur->unlock();

			Time procStart;
//Log::alert("cvtColor 2 start");
			cvtColor(mCurImage, mCurImageGray, CV_BGR2GRAY);
//Log::alert("cvtColor 2 end");

			points.clear();
			points = findFeaturePoints(mCurImageGray);

			shared_ptr<cv::Mat> imgAnnotated(new cv::Mat());
			mCurImage.copyTo(*imgAnnotated);
			drawPoints(points, *imgAnnotated);

			mCurImageAnnotated = imgAnnotated;
			
			shared_ptr<DataAnnotatedImage> imgAnnotatedData(new DataAnnotatedImage());
			imgAnnotatedData->imgAnnotated = imgAnnotated;
			imgAnnotatedData->imgDataSource = mImageDataCur;

			shared_ptr<ImageFeatureData> data(new ImageFeatureData());
			data->featurePoints = points;
			data->imgData = mImageDataCur;
			data->imgAnnotated = imgAnnotatedData;
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onFeaturesFound(data);

			mImgProcTimeUS = procStart.getElapsedTimeUS();
//			if(mQuadLogger != NULL)
//			{
//				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME_FEATURE_MATCH + "\t" + mImgProcTimeUS;
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
		}

		System::msleep(1);
	}

	mFinished = true;
}

vector<cv::Point2f> FeatureFinder::findFeaturePoints(cv::Mat const &img)
{
	vector<cv::Point2f> points;

	Log::alert("Where are you?");

	return points;
}

void FeatureFinder::drawPoints(vector<cv::Point2f> const &points, cv::Mat &img)
{
	if(points.size() == 0)
		return;
	cv::Point2f p1;
	for(int i=0; i<points.size(); i++)
	{
		p1 = points[i];
 		circle(img,p1,3,cv::Scalar(0,255,0),-1);
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
		mMutex_imageData.lock();
		mImageDataNext = static_pointer_cast<DataImage>(data);
		mMutex_imageData.unlock();
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

void FeatureFinder::onNewCommImgBufferSize(int size)
{
	mMutex_buffers.lock();
	mImgBufferMaxSize = size;
	mMutex_buffers.unlock();
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
