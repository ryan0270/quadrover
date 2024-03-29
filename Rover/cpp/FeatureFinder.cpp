#include "FeatureFinder.h"

namespace ICSL {
namespace Quadrotor{
using namespace toadlet::egg;

FeatureFinder::FeatureFinder()
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

void FeatureFinder::run()
{
	mFinished = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);
	setpriority(PRIO_PROCESS, 0, mThreadNiceValue);
	int nice = getpriority(PRIO_PROCESS, 0);
	Log::alert(String()+"FeatureFinder nice value: "+nice);

	vector<cv::Point2f> points;
	shared_ptr<DataImage> imageData;
	cv::Mat curImage, curImageGray, imageAnnotated;
	String logString;
	Time procStart;
	float qualityLevel, fastThresh;
	int sepDist;
	float pointCntTarget = mPointCntTarget;
	float fastAdaptRate = mFASTAdaptRate;
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

			if(mHaveUpdatedSettings)
			{
				mMutex_params.lock();
				qualityLevel = mQualityLevel;
				sepDist = mSepDist;
				fastThresh = mFASTThreshold;
				pointCntTarget = mPointCntTarget;
				fastAdaptRate = mFASTAdaptRate;
				mMutex_params.unlock();

				mHaveUpdatedSettings = false;
			}

			points = findFeaturePoints(curImageGray, qualityLevel, sepDist, fastThresh);
			if(points.size() > 0)
			{
				double f = imageData->focalLength;
				double cx = imageData->center.x;
				double cy = imageData->center.y;
				cv::Point2f center(cx, cy);
				cv::undistortPoints(points, points, *imageData->cameraMatrix, *imageData->distCoeffs);
				for(int i=0; i<points.size(); i++)
					points[i] = points[i]*f+center;
			}
			int numPoints = points.size(); // for logging

			// A little adaptation to achieve the target number of points
			fastThresh += min(1.0f, max(-1.0f, fastAdaptRate*((float)points.size()-pointCntTarget)));
			fastThresh = max(5.0f, fastThresh);

			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
			curImage.copyTo(*imageAnnotated);
			drawPoints(points, *imageAnnotated);
			
			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
			imageAnnotatedData->imageAnnotated = imageAnnotated;
			imageAnnotatedData->imageDataSource = imageData;
			mImageAnnotatedLast = imageAnnotatedData;

			shared_ptr<ImageFeatureData> data(new ImageFeatureData());
			data->featurePoints.swap(points);
			data->imageData = imageData;
			data->imageAnnotatedData = imageAnnotatedData;
			data->timestamp.setTime(imageData->timestamp);
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onFeaturesFound(data);

			mImageProcTimeUS = procStart.getElapsedTimeUS();
			if(mDataLogger != NULL)
			{
				mMutex_logger.lock();
				mDataLogger->addEntry(LOG_ID_FEATURE_FIND_TIME, mImageProcTimeUS/1.0e6, LOG_FLAG_CAM_RESULTS);
				mDataLogger->addEntry(LOG_ID_NUM_FEATURE_POINTS, numPoints, LOG_FLAG_CAM_RESULTS);
				mDataLogger->addEntry(LOG_ID_FAST_THRESHOLD, fastThresh, LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();
			}
		}

		System::msleep(1);
	}

	mFinished = true;
}

vector<cv::Point2f> FeatureFinder::findFeaturePoints(const cv::Mat &image, 
													 double qualityLevel,
													 double minDistance,
													 int fastThreshold)
{
	double pyrScale = 0.5;
	cv::Mat pyrImage;
	cv::resize(image, pyrImage, cv::Size(), pyrScale, pyrScale, cv::INTER_AREA);
	minDistance *= pyrScale;

	vector<cv::KeyPoint> tempKp1;
	cv::Ptr<cv::FastFeatureDetector> fastDetector(new cv::FastFeatureDetector(fastThreshold));
	int maxKp = 1000;
	int gridRows = 3;
	int gridCols = 3;
	cv::GridAdaptedFeatureDetector detector(fastDetector, maxKp, gridRows, gridCols);
	detector.detect(pyrImage, tempKp1);
//	FAST(pyrImage, tempKp1, fastThreshold, true);
	int blockSize = 5;
	eigenValResponses(pyrImage, tempKp1, blockSize);

	double maxScore = -0xFFFFFFF;
	for(int i=0; i<tempKp1.size(); i++)
		maxScore = max(maxScore, (double) tempKp1[i].response);

	vector<cv::KeyPoint> tempKp;
	tempKp.reserve(tempKp1.size());
	double threshold = qualityLevel*maxScore;
	for(int i=0; i<tempKp1.size(); i++)
		if(tempKp1[i].response > threshold)
			tempKp.push_back(tempKp1[i]);

	// group keyPoints into grid
	const int cellSize = minDistance+0.5;
	const int nGridX= (pyrImage.cols+cellSize-1)/cellSize;
	const int nGridY= (pyrImage.rows+cellSize-1)/cellSize;

	vector<vector<int> > grids(nGridX*nGridY); // stores index of keykeyPoints in each grid
	vector<int> gridId(tempKp.size());
	for(int kpId=0; kpId < tempKp.size(); kpId++)
	{
		int xGrid = tempKp[kpId].pt.x / cellSize;
		int yGrid = tempKp[kpId].pt.y / cellSize;
		int gid = yGrid*nGridX+xGrid;
		grids[gid].push_back(kpId);
		gridId[kpId] = gid;
	}

	// Now pick the strongest keyPoints 
	// Right now, for reduced computation, it is //using a minDistance x minDistance box for exclusion rather
	// than a minDistance radius circle
	vector<bool> isActive(gridId.size(), true);
	vector<cv::KeyPoint> keyPoints;
	double curResponse;
	int neighborOffsetX[] = {-1, 0, 1, -1, 1, -1, 0, 1};
	int neighborOffsetY[] = {-1, -1, -1, 0, 0, 1, 1, 1};
	for(int i=0; i<gridId.size(); i++)
	{
		if(!isActive[i])
			continue;

		curResponse = tempKp[i].response;
		cv::Point2f *curPt = &tempKp[i].pt;

		// turn off other keyPoints in this grid
		int gid = gridId[i];
		bool isStrongest = true;;
		int j=0;
		while(isStrongest && j < grids[gid].size() )
		{
			int kpId = grids[gid][j];
			if(kpId != i && curResponse >= tempKp[kpId].response)
				isActive[kpId] = false;
			else if(kpId != i && isActive[kpId])
				isStrongest = false;

			j++;
		}

		if(!isStrongest)
		{
			isActive[i] = false;
			continue;
		}

		// need to check neighbor grids too
		int xGrid = gid % nGridX;
		int yGrid = gid / nGridX;

		j = 0;
		while(isStrongest && j < 8)
		{
			int nx = xGrid+neighborOffsetX[j];
			int ny = yGrid+neighborOffsetY[j];
			if(nx < 0 || nx > nGridX-1 || ny < 0 || ny > nGridY-1)
			{
				j++;
				continue;
			}

			int ngid = ny*nGridX+nx;
			int k=0;
			while(isStrongest && k < grids[ngid].size() )
			{
				int kpId = grids[ngid][k];
				if(!isActive[kpId])
				{
					k++;
					continue;
				}

				cv::Point2f *pt = &tempKp[kpId].pt;
				if(abs(pt->x - curPt->x) <= minDistance && abs(pt->y - curPt->y) <= minDistance)
				{
					if(kpId != i && curResponse >= tempKp[kpId].response)
						isActive[kpId] = false;
					else if(kpId != i && isActive[kpId])
						isStrongest = false;
				}

				k++;
			}

			j++;
		}

		if(!isStrongest)
		{
			isActive[i] = false;
			continue;
		}

		keyPoints.push_back(tempKp[i]);
	}

	sort(keyPoints.begin(), keyPoints.end(), [&](const cv::KeyPoint &a, const cv::KeyPoint &b){return a.response > b.response;});

	vector<cv::Point2f> points;
	cv::KeyPoint::convert(keyPoints, points);

	// resize points back to original image
	for(int i=0; i<points.size(); i++)
		points[i] *= 1.0/pyrScale;

	return points;
}

// Adapted from OpenCV 2.4.5
void FeatureFinder::eigenValResponses(const cv::Mat& image, vector<cv::KeyPoint>& points, int blockSize)
{
    CV_Assert( image.type() == CV_8UC1 && blockSize*blockSize <= 2048 );

    size_t ptidx, ptsize = points.size();
    const uchar* ptr00 = image.ptr<uchar>();
    int step = (int)(image.step/image.elemSize1());
    int r = blockSize/2;

    double scale = (1 << 2) * blockSize * 255.0f;
    scale = 1.0f / scale;
    double scale_sq = scale * scale;

	cv::AutoBuffer<int> ofsbuf(blockSize*blockSize);
    int* ofs = ofsbuf;
    for( int i = 0; i < blockSize; i++ )
        for( int j = 0; j < blockSize; j++ )
            ofs[i*blockSize + j] = (int)(i*step + j);

	double m00, m01, m11;
    for( ptidx = 0; ptidx < ptsize; ptidx++ )
    {
        int x0 = cvRound(points[ptidx].pt.x - r);
        int y0 = cvRound(points[ptidx].pt.y - r);

        const uchar* ptr0 = ptr00 + y0*step + x0;
        int a = 0, b = 0, c = 0;

        for( int k = 0; k < blockSize*blockSize; k++ )
        {
            const uchar* ptr = ptr0 + ofs[k];
			// this is sobel
//            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
//            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
			// this is Scharr
            int Ix = (ptr[1] - ptr[-1])*10 + (ptr[-step+1] - ptr[-step-1])*3 + (ptr[step+1] - ptr[step-1])*3;
            int Iy = (ptr[step] - ptr[-step])*10 + (ptr[step-1] - ptr[-step-1])*3 + (ptr[step+1] - ptr[-step+1])*3;
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        }

		m00 = 0.5*a*scale_sq;
		m01 =     c*scale_sq;
		m11 = 0.5*b*scale_sq;
        points[ptidx].response = m00 + m11 - sqrt( (m00-m11)*(m00-m11) + m01*m01);
    }
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

void FeatureFinder::onNewSensorUpdate(shared_ptr<IData> const &data)
{
	if(data->type == DATA_TYPE_IMAGE)
	{
		mImageDataNext = static_pointer_cast<DataImage>(data);
		mNewImageReady = true;
	}
}

void FeatureFinder::onNewCommVisionFeatureFindQualityLevel(float qLevel)
{
	mMutex_params.lock();
	mQualityLevel = qLevel;
	mMutex_params.unlock();

	mHaveUpdatedSettings = true;
	Log::alert(String()+"Feature finder quality level set to " + qLevel);
}

void FeatureFinder::onNewCommVisionFeatureFindSeparationDistance(int sepDist)
{
	mMutex_params.lock();
	mSepDist = sepDist;
	mMutex_params.unlock();

	mHaveUpdatedSettings = true;
	Log::alert(String()+"Feature finder separation distance set to " + sepDist);
}

void FeatureFinder::onNewCommVisionFeatureFindFASTThreshold(int thresh)
{
	mMutex_params.lock();
	mFASTThreshold = thresh;
	mMutex_params.unlock();

	mHaveUpdatedSettings = true;
	Log::alert(String()+"Feature finder FAST threshold set to " + thresh);
}

void FeatureFinder::onNewCommVisionFeatureFindPointCntTarget(int target)
{
	mMutex_params.lock();
	mPointCntTarget = target;
	mMutex_params.unlock();

	mHaveUpdatedSettings = true;
	Log::alert(String()+"Feature finder point count target set to " + target);
}

void FeatureFinder::onNewCommVisionFeatureFindFASTAdaptRate(float r)
{
	mMutex_params.lock();
	mFASTAdaptRate = r;
	mMutex_params.unlock();

	mHaveUpdatedSettings = true;
	Log::alert(String()+"Feature finder FAST adapt rate set to " + r);
}

} // namespace Quadrotor
} // namespace ICSL
