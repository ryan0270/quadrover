#include "RegionFinder.h"

namespace ICSL {
namespace Quadrotor{
using namespace toadlet::egg;

RegionFinder::RegionFinder()
{
	mRunning = false;
	mFinished = true;
	mUseIbvs = false;
	mHaveUpdatedSettings = true;
	mIsMotorOn = false;

	mImageProcTimeUS = 0;

	mLastProcessTime.setTimeMS(0);

	mNewImageReady = false;

	mImageDataNext = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
	mThreadNiceValue = 0;

	mQualityLevel = 0.05;
	mSepDist = 10;
	mFASTThreshold = 30;
	mPointCntTarget = 30;
	mFASTAdaptRate = 0.01;
}

void RegionFinder::shutdown()
{
	Log::alert("-------------------------- Feature finder shutdown started ----------------------");
	mRunning = false;
	while(!mFinished)
		System::msleep(10);

	mImageDataNext = NULL;

	Log::alert("-------------------------- Feature finder done ----------------------");
}

void RegionFinder::initialize()
{
}

void RegionFinder::getLastImageAnnotated(cv::Mat *outImage)
{
//	if(mImageAnnotatedLast != NULL)
//	{
//		mImageAnnotatedLast->lock();
//		shared_ptr<cv::Mat> image = mImageAnnotatedLast->imageAnnotated;
//		mImageAnnotatedLast->unlock();
////		outImage->create(image.rows,image.cols,image.type());
//		image->copyTo(*outImage);
//	}
//	else
		(*outImage) = cv::Scalar(0);
}

void RegionFinder::run()
{
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);
	setpriority(PRIO_PROCESS, 0, mThreadNiceValue);
	int nice = getpriority(PRIO_PROCESS, 0);
	Log::alert(String()+"RegionFinder nice value: "+nice);

	vector<vector<cv::Point2f>> regions;
	shared_ptr<DataImage> imageData;
	cv::Mat curImage, curImageGray, imageAnnotated;
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
			imageData->image->copyTo(curImage);
			imageData->imageGray->copyTo(curImageGray);

			regions = findRegions(curImageGray);
			if(regions.size() > 0)
			{
				double f = imageData->focalLength;
				cv::Point2f center = imageData->center;
//				cv::undistortPoints(regions, regions, *imageData->cameraMatrix, *imageData->distCoeffs);
				for(int i=0; i<regions.size(); i++)
				{
					cv::undistortPoints(regions[i], regions[i], *imageData->cameraMatrix, *imageData->distCoeffs);
					for(int j=0; j<regions[i].size(); j++)
						regions[i][j] = regions[i][j]*f+center;
				}
			}
			vector<cv::Point2f> centroids(regions.size());
			vector<cv::Moments> moms(regions.size());
			for(int i=0; i<regions.size(); i++)
			{
				moms[i] = moments(regions[i]);
				centroids[i].x = moms[i].m10/moms[i].m00;
				centroids[i].y = moms[i].m01/moms[i].m00;
			}
			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
			curImage.copyTo(*imageAnnotated);
			drawRegions(*imageAnnotated, regions, centroids);
			
			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
			imageAnnotatedData->imageAnnotated = imageAnnotated;
			imageAnnotatedData->imageDataSource = imageData;
			mImageAnnotatedLast = imageAnnotatedData;

			shared_ptr<ImageRegionData> data(new ImageRegionData());
			data->regionContours.swap(regions);
			data->regionCentroids.swap(centroids);
			data->regionMoments.swap(moms);
			data->imageData = imageData;
			data->imageAnnotatedData = imageAnnotatedData;
			data->timestamp.setTime(imageData->timestamp);
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onRegionsFound(data);

			mImageProcTimeUS = procStart.getElapsedTimeUS();
			if(mDataLogger != NULL)
			{
				mMutex_logger.lock();
				mDataLogger->addEntry(LOG_ID_REGION_FIND_TIME, mImageProcTimeUS/1.0e6, LOG_FLAG_CAM_RESULTS);
				mDataLogger->addEntry(LOG_ID_NUM_REGIONS, regions.size(), LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();
			}
		}

		System::msleep(1);
	}

	mFinished = true;
}

vector<vector<cv::Point2f>> RegionFinder::findRegions(const cv::Mat &image)
{
	cv::Mat pyrImage;
	double pyrScale = 0.5;
	resize(image, pyrImage, cv::Size(), pyrScale, pyrScale, cv::INTER_AREA);

	int delta = 5*2;
	int minArea = 1.0/pow(30,2)*pyrImage.rows*pyrImage.cols;
	int maxArea = 1.0/pow(2,2)*pyrImage.rows*pyrImage.cols;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.4; // smaller increase the number of regions
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);
	vector<vector<cv::Point>> regions;
	mserDetector(pyrImage, regions);

	// now get the outer contours
	vector<vector<cv::Point>> contours = getContours(pyrImage, regions);

	// and return to original size
	double s = 1.0/pyrScale;
	for(int i=0; i<contours.size(); i++)
		for(int j=0; j<contours[i].size(); j++)
			contours[i][j] *= s;


	// and, finally, convert to point2f form
	vector<vector<cv::Point2f>> contours2f(contours.size());
	for(int i=0; i<contours.size(); i++)
	{
		contours2f[i].resize(contours[i].size());
		for(int j=0; j<contours[i].size(); j++)
			contours2f[i][j] = contours[i][j];
	}

	return contours2f;
}

void RegionFinder::drawRegions(cv::Mat &image,
							   const vector<vector<cv::Point2f>> &regions,
							   const vector<cv::Point2f> &centroids)
{
	if(regions.size() == 0)
		return;


	vector<vector<cv::Point>> regionsChad(regions.size());
	for(int i=0; i<regions.size(); i++)
	{
		regionsChad[i].resize(regions[i].size());
		for(int j=0; j<regions[i].size(); j++)
			regionsChad[i][j] = regions[i][j];
	}
//	cv::drawContours(image, regions, -1, cv::Scalar(255,0,0), 2);
	cv::drawContours(image, regionsChad, -1, cv::Scalar(255,0,0), 2);
	for(int i=0; i<centroids.size(); i++)
		cv::circle(image, centroids[i], 3, cv::Scalar(255,0,0), -1);
}

void RegionFinder::onNewSensorUpdate(shared_ptr<IData> const &data)
{
	if(data->type == DATA_TYPE_IMAGE)
	{
		mImageDataNext = static_pointer_cast<DataImage>(data);
		mNewImageReady = true;
	}
}

vector<vector<cv::Point>> RegionFinder::getContours(const cv::Mat &image, const vector<vector<cv::Point>> &regions)
{
	if(regions.size() == 0)
		return vector<vector<cv::Point>>();

	// preallocate
	cv::Mat mask(image.rows,image.cols,CV_8UC1, cv::Scalar(0));

	/////////////////// Find contours ///////////////////////
	vector<vector<cv::Point>> allContours;
	allContours.reserve(regions.size());
	vector<vector<cv::Point>> contours;
	cv::Rect boundRect;
	int border = 2;
	for(int i=0; i<regions.size(); i++)
	{
		boundRect = boundingRect( cv::Mat(regions[i]) );
		// reject regions on the border since they will change, but perhaps not enough to 
		// prevent matches
		if(boundRect.x == 0 || boundRect.y == 0 ||
			boundRect.x+boundRect.width == image.cols || boundRect.y+boundRect.height == image.rows )
			continue;

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

		cv::findContours(mask(boundRect), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, corner);

		double area;
		double maxArea = 0;
		int maxAreaIdx = -1;
		for(int i=0; i<contours.size(); i++)
		{
			area = cv::contourArea(contours[i]);
			if(area > maxArea)
			{
				maxArea = area;
				maxAreaIdx = i;
			}
		}

		if(i != -1)
			allContours.push_back(contours[maxAreaIdx]);
		else
			Log::alert("Why didn't I find any contours in this region?");
	}

	return allContours;
}

} // namespace Quadrotor
} // namespace ICSL
