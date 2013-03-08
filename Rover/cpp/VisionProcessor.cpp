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
	mCurImageAnnotated.create(height, width, CV_8UC3); mCurImageAnnotated = cv::Scalar(0);
	mCurImageGray.create(height, width, CV_8UC1); mCurImageGray = cv::Scalar(0);

	mImgProcTimeUS = 0;

	mLastProcessTime.setTimeMS(0);

	mNewImageReady = false;

	mImgBufferMaxSize = 10;
	
	mLogImages = false;
mLogImages = true;

	mImageDataPrev = mImageDataCur = mImageDataNext = NULL;
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
	mCurImageAnnotated.copyTo(*outImage);
	mMutex_image.unlock();
}

void VisionProcessor::run()
{
	System sys;
	mFinished = false;
	mRunning = true;
	while(mRunning)
	{
		if(mNewImageReady && mImageDataCur == NULL)
			mImageDataCur = mImageDataNext;
		else if(mNewImageReady)
		{
			mMutex_imageSensorData.lock();
			mImageDataPrev = mImageDataCur;
			mImageDataCur = mImageDataNext;
			mImageDataPrev->lock(); mImageDataCur->lock();
			double dt = Time::calcDiffUS(mImageDataPrev->timestamp, mImageDataCur->timestamp)/1.0e6;
			mImageDataPrev->unlock(); 
			mMutex_imageSensorData.unlock();

			mCurImage = *(mImageDataCur->img);
			mCurImageGray.create(mImageDataCur->img->rows, mImageDataCur->img->cols, mImageDataCur->img->type());
			mCurImageAnnotated.create(mImageDataCur->img->rows, mImageDataCur->img->cols, mImageDataCur->img->type());
			mImageDataCur->unlock();

			Time procStart;
			cvtColor(mCurImage, mCurImageGray, CV_BGR2GRAY);

			vector<vector<cv::Point2f> > points = getMatchingPoints(mCurImageGray);
			mCurImage.copyTo(mCurImageAnnotated);
			drawMatches(points, mCurImageAnnotated);
//Log::alert(String()+"Found " + points[0].size() + " matches");

			shared_ptr<ImageMatchData> data(new ImageMatchData());
			data->dt = dt;
			data->featurePoints = points;
			data->imgData0 = mImageDataPrev;
			data->imgData1 = mImageDataCur;
			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onImageProcessed(data);
			data = NULL;

			mImgProcTimeUS = procStart.getElapsedTimeUS();
			{
				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IMG_PROC_TIME + "\t" + mImgProcTimeUS;
				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
			}

			if(mLogImages)
			{
				mMutex_imgBuffer.lock();
				mImgDataBuffer.push_back(shared_ptr<SensorDataImage>(mImageDataCur));
				while(mImgDataBuffer.size() > mImgBufferMaxSize)
					mImgDataBuffer.pop_front();
				mMutex_imgBuffer.unlock();
			}

			mNewImageReady = false;
		}

		sys.msleep(1);
	}

	mQuadLogger->saveImageBuffer(mImgDataBuffer);

	mImgDataBuffer.clear();

	mFinished = true;
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

// vector<vector<cv::Point2f> > VisionProcessor::getMatchingPoints()
// {
//  	cv::Mat pyrImg;
// 	cv::pyrDown(mCurImageGray,pyrImg);
// 	cv::pyrDown(pyrImg,pyrImg);
// 	
// 	// manually setting these parameters doesn't seem to work right
// 	int delta=5;
// 	int minArea = 60;
// 	int maxArea = pyrImg.rows*pyrImg.cols/4.0;
// 	double maxVariation=0.2; //! prune the area have simliar size to its children -- smaller number means fewer regions found
// 	double minDiversity=0.1; //! trace back to cut off mser with diversity < min_diversity -- smaller number means more regions found
// 	cv::MSER regionDetector(delta,minArea,maxArea,maxVariation,minDiversity);
// 
// 	vector<vector<cv::Point> > regions;
// 	regionDetector(pyrImg, regions); // the region is actually a point cloud I think
// 	
// 	// get hulls for first image
// 	double scaleX = (double)pyrImg.size().width/mLastImageGray.size().width;
// 	double scaleY = (double)pyrImg.size().height/mLastImageGray.size().height;
// 
// 	// get hulls for second image
// 	scaleX = (double)pyrImg.size().width/mCurImageGray.size().width;
// 	scaleY = (double)pyrImg.size().height/mCurImageGray.size().height;
// 	vector<vector<cv::Point> > hulls;
// 	vector<vector<double> > huMoms;
// 	cv::Moments mom;
// 	vector<cv::Point2f> centroids;
// 	for(int i=0; i<regions.size(); i++)
// 	{
// 		vector<cv::Point> hull;
// 		cv::convexHull(regions[i],hull);
// 		// scale the hull back to the original image size
// 		for(int j=0; j<hull.size(); j++)
// 		{
// 			hull[j].x /= scaleX;
// 			hull[j].y /= scaleY;
// 		}
// 		hulls.push_back(hull);
// 
// //		mom = moments(regions[i]); // regions[i] isn't scaled and I don't want to pay the cost to scale every point
// 		mom = moments(hull);
// 		cv::Point2f centroid;
// 		centroid.x = mom.m10/mom.m00;
// 		centroid.y = mom.m01/mom.m00;
// 		centroids.push_back(centroid);
// 		vector<double> huMom;
// 		cv::HuMoments(mom, huMom);
// 		huMoms.push_back(huMom);
// 	}
// 	
// 	// find matches
// 	vector<int> matchIndex0, matchIndex1;
// 	double matchThreshold = 1;
// 	for(int index1=0; index1<huMoms.size(); index1++)
// 	{
// 		double bestScore = 999999999;
// 		int bestIndex = -1;
// 		for(int index0=0; index0<mMSERHuMoments.size(); index0++)
// 		{
// 			double score = 0;
// 			for(int k=0; k<7; k++)
// 				score += abs(huMoms[index1][k]-mMSERHuMoments[index0][k]);
// //			score = norm(centroids[index1]-mMSERCentroids[index0]);
// 			if(score < bestScore || bestIndex == -1)
// 			{
// 				bestScore = score;
// 				bestIndex = index0;
// 			}
// 		}
// 
// 		if(bestScore < matchThreshold)
// 		{
// 			matchIndex0.push_back(bestIndex);
// 			matchIndex1.push_back(index1);
// 		}
// 	}
// 
// 	for(int i=0; i<matchIndex1.size(); i++)
// 		cv::drawContours(mCurImage, hulls, matchIndex1[i], cv::Scalar(0,0,255));
// 
// 	Log::alert(String()+"MSER: \t" + regions.size() +" found and " + matchIndex1.size() + " matched.");
// 
// 	vector<vector<cv::Point2f> > points(2);
// 	for(int i=0; i<matchIndex1.size(); i++)
// 	{
// 		points[0].push_back(mMSERCentroids[matchIndex0[i]]);
// 		points[1].push_back(centroids[matchIndex1[i]]);
// 		circle(mCurImage,points[0].back(),3,cv::Scalar(0,255,0),-1);
// 		circle(mCurImage,points[1].back(),3,cv::Scalar(255,0,0),-1);
// 		line(mCurImage,points[0].back(),points[1].back(),cv::Scalar(255,255,255),1);
// 	}
// 
// 	// copy the moments over for the next image
// 	mMSERHuMoments = huMoms;
// 	mMSERCentroids = centroids;
// 
// 	return points;
// }

void VisionProcessor::enableIbvs(bool enable)
{
	mUseIbvs = enable;
	if(mUseIbvs)
	{
		Log::alert("Turning IBVS on");
		String str = String()+ mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IBVS_ENABLED + "\t";
		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
		mLastImgFoundTime.setTime();
		mFirstImageProcessed = false;
	}
	else
	{
		Log::alert("Turning IBVS off");
		String str = String() + mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_IBVS_DISABLED + "\t";
		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
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
		mNewImageReady = true;
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
	mMutex_imgBuffer.lock();
	mImgBufferMaxSize = size;
	mMutex_imgBuffer.unlock();
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

void VisionProcessor::onCommConnectionLost()
{
	// stop logging images so I don't loose the ones from the flight
	mLogImages = false;
}

} // namespace Quadrotor
} // namespace ICSL
