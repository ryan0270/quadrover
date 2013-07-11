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
//		outImage->create(image.rows,image.cols,image.type());
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

			cvtColor(curImage, curImageGray, CV_BGR2GRAY);

			points.clear();
			double qualityLevel = 0.05;
			double minDistance = 10;
			int fastThreshold = 20;
			points = findFeaturePoints(curImageGray, qualityLevel, minDistance, fastThreshold);

			shared_ptr<cv::Mat> imageAnnotated(new cv::Mat());
			curImage.copyTo(*imageAnnotated);
			drawPoints(points, *imageAnnotated);
			
			shared_ptr<DataAnnotatedImage> imageAnnotatedData(new DataAnnotatedImage());
			imageAnnotatedData->imageAnnotated = imageAnnotated;
			imageAnnotatedData->imageDataSource = imageData;
			mImageAnnotatedLast = imageAnnotatedData;

			shared_ptr<ImageFeatureData> data(new ImageFeatureData());
			data->featurePoints = points;
			data->imageData = imageData;
			data->imageAnnotated = imageAnnotatedData;
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onFeaturesFound(data);

			mImageProcTimeUS = procStart.getElapsedTimeUS();
Log::alert(String()+"feature find time: " + mImageProcTimeUS/1.0e3 + "ms");
			if(mQuadLogger != NULL)
			{
				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME_FEATURE_MATCH + "\t" + mImageProcTimeUS;
				mMutex_logger.lock();
				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();

				String str2 = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_NUM_FEATURE_POINTS+"\t";
				str2 = str2+points.size();
				mMutex_logger.lock();
				mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);
				mMutex_logger.unlock();

			}
		}

		System::msleep(1);
	}

	mFinished = true;
}

vector<cv::Point2f> FeatureFinder::findFeaturePoints(cv::Mat const &image, 
													 double const &qualityLevel,
													 double const &minDistance,
													 int const &fastThreshold)
{
	vector<cv::KeyPoint> tempKp1;
	cv::Ptr<cv::FastFeatureDetector> fastDetector(new cv::FastFeatureDetector(fastThreshold));
	int maxKp = 1000;
	int gridRows = 3;
	int gridCols = 3;
	cv::GridAdaptedFeatureDetector detector(fastDetector, maxKp, gridRows, gridCols);
	detector.detect(image, tempKp1);
//	FAST(image, tempKp1, fastFeatureThreshold, true);
	eigenValResponses(image, tempKp1, 5);

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
	const int nGridX= (image.cols+cellSize-1)/cellSize;
	const int nGridY= (image.rows+cellSize-1)/cellSize;

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
	// Right now, for reduced computation, it is using a minDistance x minDistance box for exclusion rather
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
//		for(int j=0; j<grids[gid].size(); j++)
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

//		for(int j=0; j<8; j++)
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
//			for(int k=0; k<grids[ngid].size(); k++)
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

	sort(keyPoints.begin(), keyPoints.end(), [&](cv::KeyPoint const &a, cv::KeyPoint const &b){return a.response > b.response;});

	vector<cv::Point2f> points;
	cv::KeyPoint::convert(keyPoints, points);

	return points;
}

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
		
//cout << points[ptidx].response << endl;
		int chad = 0;
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
