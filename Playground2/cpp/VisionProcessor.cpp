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
	mMatcher("FAST_GRID","BRIEF")
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

 	mFocalLength = 3.7*width/5.76; // (focal length mm)*(img width px)/(ccd width mm)

	mNewImageReady = false;

	mImgBufferMaxSize = 10;
	
	mLogImages = false;
}

void VisionProcessor::shutdown()
{
	Log::alert("-------------------------- Vision processor shutdown started ----------------------");
	mRunning = false;
	System sys;
	while(!mFinished)
	{
		Log::alert("VisionProcessor waiting");
		sys.msleep(5);
	}

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
	Array2D<double> imgAtt(3,1,0.0), rotVel(3,1,0.0);
	SensorDataImage data;
	while(mRunning)
	{
		if(mNewImageReady)
		{
			mMutex_imageSensorData.lock();
			mImageData.copyTo(data);
			mMutex_imageSensorData.unlock();

			if(data.img.rows != mCurImage.rows || data.img.cols != mCurImage.cols)
			{
				mCurImage.create(data.img.rows, data.img.cols, data.img.type());
				mCurImageGray.create(data.img.rows, data.img.cols, data.img.type());
				mCurImageAnnotated.create(data.img.rows, data.img.cols, data.img.type());
			}
			data.img.copyTo(mCurImage);
			mNewImageReady = false;

			Time procStart;
			cvtColor(mCurImage, mCurImageGray, CV_BGR2GRAY);

			vector<vector<cv::Point2f> > points = getMatchingPoints(mCurImageGray);
			mCurImage.copyTo(mCurImageAnnotated);
			drawMatches(points, mCurImageAnnotated);
//Log::alert(String()+"Found " + points[0].size() + " matches");

//			if(mUseIbvs)
			{
//				vector<vector<cv::Point2f> > points = getMatchingPoints();
//				if(points.size() > 0)
//					calcOpticalFlow(points);
			}

			mImgProcTimeUS = procStart.getElapsedTimeUS();
			{
				String str = String()+" "+mStartTime.getElapsedTimeMS() + "\t-600\t" + mImgProcTimeUS;
				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
			}

			if(mLogImages)
			{
				mMutex_imgBuffer.lock();
				mImgBuffer.push_back(mCurImage.clone());
				mImgDataBuffer.push_back(data);
				while(mImgBuffer.size() > mImgBufferMaxSize)
					mImgBuffer.pop_front();
				while(mImgDataBuffer.size() > mImgBufferMaxSize)
					mImgDataBuffer.pop_front();
				mMutex_imgBuffer.unlock();
			}
		}

		sys.msleep(1);
	}

	if(mLogImages)
		mQuadLogger->saveImageBuffer(mImgBuffer, mImgDataBuffer);

	mFinished = true;
}

vector<vector<cv::Point2f> > VisionProcessor::getMatchingPoints(cv::Mat const &img)
{
	int result = mMatcher.Track(img, false);
	vector<vector<cv::Point2f> > points(2);

	for(int i=0; i<mMatcher.vMatches.size(); i++)
	{
		Matcher::p_match match = mMatcher.vMatches[i];
		points[0].push_back(cv::Point2f(match.u1p, match.v1p));
		points[1].push_back(cv::Point2f(match.u1c, match.v1c));
	}

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

void VisionProcessor::calcOpticalFlow(vector<vector<cv::Point2f> > const &points)
{
	if(mLastProcessTime.getMS() == 0)
	{
		mLastProcessTime.setTime();
		return;
	}
	double dt = mLastProcessTime.getElapsedTimeUS()/1.0e6;
 	Array2D<double> curRotMat = createIdentity(3);
 	Array2D<double> curAngularVel(3,1,0.0);
 	Array2D<double> normDir(3,1,0.0);
	
	int numPoints = points[0].size();
	Array2D<double> avgFlow(3,1,0.0);
	double maxViewAngle = 0;
	for(int i=0; i<numPoints; i++)
	{
		Array2D<double> p0(3,1), p1(3,1);
		p0[0][0] = points[0][i].x;
		p0[1][0] = points[0][i].y;
		p0[2][0] = mFocalLength;
		p0 = 1.0/norm2(p0)*p0;

		p1[0][0] = points[1][i].x;
		p1[1][0] = points[1][i].y;
		p1[2][0] = mFocalLength;
		p1 = 1.0/norm2(p1)*p1;
		
		avgFlow += 1.0/dt*(p1-p0);
		avgFlow[0][0] = points[0][i].x-points[1][i].x;
		avgFlow[1][0] = points[0][i].y-points[1][i].y;
		avgFlow[2][0] = 0;

		double viewAngle = acos(matmultS(transpose(p1),matmult(transpose(curRotMat),normDir)));
		maxViewAngle = max(maxViewAngle,viewAngle);
	}
	avgFlow = 1.0/numPoints*avgFlow;
printArray("avgFlow: \t\t\t\t\t\t\t\t\t\t",transpose(avgFlow));
 
 	// Flow calc taken from
 	// Herisse et al (2012) - Landing a VTOL Unmanned Aerial Vehicle on a Moving Platform Using Optical Flow
 	double viewAngle = maxViewAngle;
 	double lambda = pow(sin(viewAngle),2)/(4.0-pow(sin(viewAngle),2));
 	Array2D<double> LambdaInv(3,3,0.0);
 	LambdaInv[0][0] = LambdaInv[1][1] = lambda;
 	LambdaInv[2][2] = 0.5;
 	LambdaInv = 4.0/PI/pow(sin(lambda),4)*LambdaInv;
 
 	normDir[2][0] = 1;
 	Array2D<double> transFlow = -1.0*matmult(LambdaInv,
 										matmult(curRotMat,
 											avgFlow + PI*pow(sin(viewAngle),2)*cross(curAngularVel, matmult(transpose(curRotMat),normDir))));
//	printArray("transFlow: \t",transpose(transFlow));
}

void VisionProcessor::enableIbvs(bool enable)
{
	mUseIbvs = enable;
	if(mUseIbvs)
	{
		Log::alert("Turning IBVS on");
		String str = String()+" " + mStartTime.getElapsedTimeMS() + "\t-605\t";
		mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
		mLastImgFoundTime.setTime();
		mFirstImageProcessed = false;
	}
	else
	{
		Log::alert("Turning IBVS off");
		String str = String() + mStartTime.getElapsedTimeMS() + "\t-606\t";
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

void VisionProcessor::onNewSensorUpdate(SensorData const *data)
{
	if(data->type == SENSOR_DATA_TYPE_IMAGE)
	{
		mMutex_imageSensorData.lock();
		((SensorDataImage*)data)->copyTo(mImageData);
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
		mImgBuffer.clear();
		mImgDataBuffer.clear();
	}
}

void VisionProcessor::onNewCommImgBufferSize(int size)
{
	mMutex_imgBuffer.lock();
	mImgBufferMaxSize = size;
	mMutex_imgBuffer.unlock();
}

} // namespace Quadrotor
} // namespace ICSL
