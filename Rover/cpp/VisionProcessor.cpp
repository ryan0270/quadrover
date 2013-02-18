#include "VisionProcessor.h"

#include "tbb/parallel_for.h"

using namespace toadlet;
//using namespace cv;
using toadlet::egg::String;
using namespace TNT;
using namespace ICSL::Constants;

namespace ICSL {
namespace Quadrotor{

ImageGrabber::ImageGrabber() :
	mImgAtt(3,1,0.0),
	mImgRotVel(3,1,0.0)
{
	mRunning = false;
	mFinished = true;
	mIsBottleneck = false;
	mNewImageReady = false;
	mImgConversionDone= false;

	mCurImage.create(240, 320, CV_8UC3); mCurImage = cv::Scalar(0);
	mCurImageGray.create(240, 320, CV_8UC1); mCurImageGray = cv::Scalar(0);

	mAttObserver = NULL;
}

void ImageGrabber::copyImage(cv::Mat *dstImage)
{
	mMutex_image.lock();
	mCurImage.copyTo(*dstImage);
	mNewImageReady = false;
	mMutex_image.unlock();
}

void ImageGrabber::copyImageGray(cv::Mat *dstImage)
{
	mMutex_image.lock();
	mCurImageGray.copyTo(*dstImage);
	mNewImageReady = false;
	mMutex_image.unlock();
}

void ImageGrabber::shutdown()
{
	Log::alert("-------------------------- Image grabber shutdown started ----------------------");
	mRunning = false;
	System sys;
	while(!mFinished)
		sys.msleep(5);

	mCurImage.release();
	
	Log::alert("-------------------------- Image grabber done ----------------------");
}

void ImageGrabber::run()
{
	cv::VideoCapture cap;
	
	cap.open(CV_CAP_ANDROID+0); // back camera
	if(cap.isOpened())
	{
		Log::alert("Successfully opened the camera");
		mRunning = true;

		cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
		cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
//		cap.set(CV_CAP_PROP_ANDROID_FLASH_MODE,CV_CAP_ANDROID_FLASH_MODE_TORCH); // for now just leave this on the whole time
		cap.set(CV_CAP_PROP_ANDROID_FOCUS_MODE,CV_CAP_ANDROID_FOCUS_MODE_CONTINUOUS_VIDEO);
//		cap.set(CV_CAP_PROP_ANDROID_FOCUS_MODE,CV_CAP_ANDROID_FOCUS_MODE_INFINITY);
		cap.set(CV_CAP_PROP_EXPOSURE, -4);
//		cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 5);
		cap.set(CV_CAP_PROP_ANDROID_ANTIBANDING, CV_CAP_ANDROID_ANTIBANDING_OFF);
		cap.set(CV_CAP_PROP_AUTOGRAB, 1); // any nonzero is "on"
	}
	else
	{
		Log::alert("Failed to open the camera");
		mRunning = false;
	}

	System sys;
	cv::Mat newImg;
	mFinished = false;
	Array2D<double> lastAtt(3,1,0.0);
	Time imgAcqTime;
	while(mRunning)
	{          	
		cap.grab();
		double dt = imgAcqTime.getElapsedTimeMS();
		mMutex_data.lock();
		imgAcqTime.setTime();
		if(mAttObserver != NULL && dt > 0 && dt < 1)
		{
			mImgAtt.inject(mAttObserver->getCurAttitude());
			mImgRotVel.inject(1.0/dt*(mImgAtt-lastAtt));
			lastAtt.inject(mImgAtt);
		}
		mMutex_data.unlock();
		cap.retrieve(newImg);
		mMutex_image.lock();
		newImg.copyTo(mCurImage);
		mImgConversionDone = false;
//		if(!mIsBottleneck)
//		{
//			cvtColor(newImg, mCurImageHSV, CV_BGR2HSV);
			cvtColor(newImg, mCurImageGray, CV_BGR2GRAY);
			mImgConversionDone = true;
//		}
		mMutex_image.unlock();
		mNewImageReady = true;

		sys.msleep(1);
	}

	if(cap.isOpened())
	{
		mMutex_image.lock();
		cap.set(CV_CAP_PROP_ANDROID_FLASH_MODE,CV_CAP_ANDROID_FLASH_MODE_OFF);
		cap.release();
		mMutex_image.unlock();
	}

	mFinished = true;
}

Array2D<double> ImageGrabber::getImageAtt()
{
	mMutex_data.lock();
	Array2D<double> temp = mImgAtt.copy();
	mMutex_data.unlock();

	return temp;
}

Array2D<double> ImageGrabber::getRotVel()
{
	mMutex_data.lock();
	Array2D<double> temp = mImgRotVel.copy();
	mMutex_data.unlock();

	return temp;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
VisionProcessor::VisionProcessor()
{
	mRunning = false;
	mFinished = true;
	mUseIbvs = false;
	mFirstImageProcessed = false;
	mCurImage.create(240, 320, CV_8UC3); mCurImage = cv::Scalar(0);
	mCurImageGray.create(240, 320, CV_8UC1); mCurImageGray = cv::Scalar(0);
	mCurImageGray.copyTo(mLastImageGray);

	mImgProcTimeUS = 0;

	mLastProcessTime.setTimeMS(0);

 	mFocalLength = 3.7*320.0/5.76; // (focal length mm)*(img width px)/(ccd width mm)
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
	mLastImageGray.release();
	mMutex_image.unlock();

	mImageGrabber.shutdown();

	Log::alert("-------------------------- Vision processor done ----------------------");
}

void VisionProcessor::getLastImage(cv::Mat *outImage)
{
	mMutex_image.lock();
	mCurImage.copyTo(*outImage);
	mMutex_image.unlock();
}

void VisionProcessor::run()
{
	System sys;
	mFinished = false;
	mRunning = true;
	mImageGrabber.start();
	Array2D<double> imgAtt(3,1,0.0), rotVel(3,1,0.0);
	while(mRunning)
	{
		if(!mImageGrabber.isNewImageReady())
			mImageGrabber.markBottleneck(true); // I don't want this in the polling loop
		while(!mImageGrabber.isNewImageReady())
			sys.msleep(1);
		mMutex_image.lock();
		mImageGrabber.markBottleneck(false);
		imgAtt.inject(mImageGrabber.getImageAtt());
		rotVel.inject(mImageGrabber.getRotVel());

		Time procStart;
		mImageGrabber.copyImage(&mCurImage);
//		if(mImageGrabber.imageConversionDone())
//		{
			mImageGrabber.copyImageGray(&mCurImageGray);
//		}
//		else
//		{
//			mImageGrabber.copyImage(&mCurImage);
//			cvtColor(mCurImage, mCurImageGray, CV_BGR2GRAY);
//		}

//		if(mUseIbvs)
		{
//			vector<vector<cv::Point2f> > points = getMatchingPoints();
//			if(points.size() > 0)
//				calcOpticalFlow(points);
		}

		mMutex_image.unlock();
		mImgProcTimeUS = procStart.getElapsedTimeUS();
		{
			String str = String()+" "+mStartTime.getElapsedTimeMS() + "\t-600\t" + mImgProcTimeUS;
			mQuadLogger->addLine(str,CAM_RESULTS);
		}

		mCurImageGray.copyTo(mLastImageGray);

		sys.msleep(10);
	}

	mFinished = true;
}

vector<vector<cv::Point2f> > VisionProcessor::getMatchingPoints()
{
 	cv::Mat pyrImg;
	cv::pyrDown(mCurImageGray,pyrImg);
	cv::pyrDown(pyrImg,pyrImg);
	
	// manually setting these parameters doesn't seem to work right
	int delta=5;
	int minArea = 60;
	int maxArea = pyrImg.rows*pyrImg.cols/4.0;
	double maxVariation=0.2; //! prune the area have simliar size to its children -- smaller number means fewer regions found
	double minDiversity=0.1; //! trace back to cut off mser with diversity < min_diversity -- smaller number means more regions found
	cv::MSER regionDetector(delta,minArea,maxArea,maxVariation,minDiversity);

	vector<vector<cv::Point> > regions;
	regionDetector(pyrImg, regions); // the region is actually a point cloud I think
	
	// get hulls for first image
	double scaleX = (double)pyrImg.size().width/mLastImageGray.size().width;
	double scaleY = (double)pyrImg.size().height/mLastImageGray.size().height;

	// get hulls for second image
	scaleX = (double)pyrImg.size().width/mCurImageGray.size().width;
	scaleY = (double)pyrImg.size().height/mCurImageGray.size().height;
	vector<vector<cv::Point> > hulls;
	vector<vector<double> > huMoms;
	cv::Moments mom;
	vector<cv::Point2f> centroids;
	for(int i=0; i<regions.size(); i++)
	{
		vector<cv::Point> hull;
		cv::convexHull(regions[i],hull);
		// scale the hull back to the original image size
		for(int j=0; j<hull.size(); j++)
		{
			hull[j].x /= scaleX;
			hull[j].y /= scaleY;
		}
		hulls.push_back(hull);

//		mom = moments(regions[i]); // regions[i] isn't scaled and I don't want to pay the cost to scale every point
		mom = moments(hull);
		cv::Point2f centroid;
		centroid.x = mom.m10/mom.m00;
		centroid.y = mom.m01/mom.m00;
		centroids.push_back(centroid);
		vector<double> huMom;
		cv::HuMoments(mom, huMom);
		huMoms.push_back(huMom);
	}
	
	// find matches
	vector<int> matchIndex0, matchIndex1;
	double matchThreshold = 1;
	for(int index1=0; index1<huMoms.size(); index1++)
	{
		double bestScore = 999999999;
		int bestIndex = -1;
		for(int index0=0; index0<mMSERHuMoments.size(); index0++)
		{
			double score = 0;
			for(int k=0; k<7; k++)
				score += abs(huMoms[index1][k]-mMSERHuMoments[index0][k]);
//			score = norm(centroids[index1]-mMSERCentroids[index0]);
			if(score < bestScore || bestIndex == -1)
			{
				bestScore = score;
				bestIndex = index0;
			}
		}

		if(bestScore < matchThreshold)
		{
			matchIndex0.push_back(bestIndex);
			matchIndex1.push_back(index1);
		}
	}

	for(int i=0; i<matchIndex1.size(); i++)
		cv::drawContours(mCurImage, hulls, matchIndex1[i], cv::Scalar(0,0,255));

	Log::alert(String()+"MSER: \t" + regions.size() +" found and " + matchIndex1.size() + " matched.");

	vector<vector<cv::Point2f> > points(2);
	for(int i=0; i<matchIndex1.size(); i++)
	{
		points[0].push_back(mMSERCentroids[matchIndex0[i]]);
		points[1].push_back(centroids[matchIndex1[i]]);
		circle(mCurImage,points[0].back(),3,cv::Scalar(0,255,0),-1);
		circle(mCurImage,points[1].back(),3,cv::Scalar(255,0,0),-1);
		line(mCurImage,points[0].back(),points[1].back(),cv::Scalar(255,255,255),1);
	}

	// copy the moments over for the next image
	mMSERHuMoments = huMoms;
	mMSERCentroids = centroids;

	return points;
}

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

// void VisionProcessor::processImage(Array2D<double> const &imgAtt, Array2D<double> const &rotVel)
// {
// 	Array2D<double> curRotMat = createIdentity(3);
// 	Array2D<double> curAngularVel(3,1,0.0);
// 	Array2D<double> normDir(3,1,0.0);
// 	double dt = mLastProcessTime.getElapsedTimeUS()/1.0e6;
// 	mLastProcessTime.setTime();
// 
// 	cv::Mat temp1, temp2;
// //	cv::pyrDown(mLastImageGray,temp1);
// //	cv::pyrDown(mCurImageGray,temp2);
// 	mLastImageGray.copyTo(temp1);
// 	mCurImageGray.copyTo(temp2);
// 
// 	double pyrScale = 0.5;
// 	int levels = 3;
// 	int winsize = 5;
// 	int iterations = 3;
// 	int polyN = 5;
// 	double polySigma = 1.1;
// 	int flags = 0;
// 	
// 	cv::Size sz = temp1.size();
// 	vector<cv::Rect> masks;
// 	for(int i=0; i<3; i++)
// 	{
// 		for(int j=0; j<3; j++)
// 		{
// 			cv::Rect mask((int)(i*sz.width/3.0+0.5), (int)(j*sz.height/3.0+0.5), (int)(sz.width/3.0+0.5), (int)(sz.height/3.0+0.5));
// 			masks.push_back(mask);
// 		}
// 	}
// 	cv::Mat flow(temp1.size(),CV_32FC2);
// //	cv::Mat flow;
// //	for(int i=0; i<masks.size(); i++)
// //		cv::calcOpticalFlowFarneback(temp1(masks[i]), temp2(masks[i]), flow(masks[i]), pyrScale, levels, winsize, iterations, polyN, polySigma, flags);
// 	tbb::parallel_for(size_t(0), size_t(masks.size()), [=](size_t i) {
// 			cv::calcOpticalFlowFarneback(temp1(masks[i]), temp2(masks[i]), flow(masks[i]), pyrScale, levels, winsize, iterations, polyN, polySigma, flags);
// 			});
// 
// 	cv::Scalar meanFlow = cv::mean(flow);
// 	Log::alert(String()+"flow: "+meanFlow[0]+" \t"+meanFlow[1]);
// 
// 	for(int j=0; j<flow.rows; j+=30)
// 		for(int i=0; i<flow.cols; i+=30)
// 		{
// 			cv::Point2f p1(i,j);
// 			cv::Vec2b flw= flow.at<float>(j,i);
// 			cv::Point2f p2(i+flw[0],j+flw[1]);
// 			p2.x = max(0,min((int)p2.x, sz.width));
// 			p2.y = max(0,min((int)p2.y, sz.height));
// 
// 			circle(mCurImage,p1,2,cv::Scalar(0,125,125),-1);
// 			line(mCurImage,p1,p2,cv::Scalar(0,180,0));
// 		}
// }

// void VisionProcessor::processImage(Array2D<double> const &imgAtt, Array2D<double> const &rotVel)
// {
// 	Array2D<double> curRotMat = createIdentity(3);
// 	Array2D<double> curAngularVel(3,1,0.0);
// 	Array2D<double> normDir(3,1,0.0);
// 	double dt = mLastProcessTime.getElapsedTimeUS()/1.0e6;
// 	mLastProcessTime.setTime();
// 
// 	cv::Mat temp1, temp2;
// //	cv::pyrDown(mLastImageGray,temp1);
// //	cv::pyrDown(mCurImageGray,temp2);
// 	mLastImageGray.copyTo(temp1);
// 	mCurImageGray.copyTo(temp2);
// 	vector<cv::Point2f> frame1_features;
// 	cv::goodFeaturesToTrack(temp1, frame1_features, 400, .01, .01);
// 
// 	if(frame1_features.size() == 0)
// 	{
// 		Log::alert("Nothing to see here.");
// 		return;
// 	}
// 
// 	vector<cv::Point2f> frame2_features;
// 	vector<bool> featureCorrespondence;
// 	vector<float> err;
// 	cv::calcOpticalFlowPyrLK(temp1, temp2, frame1_features, frame2_features, featureCorrespondence, err);
// 
// 	// get a copy of only the matched points
// 	vector<Array2D<double> > p1List, p2List;
// 	Array2D<double> avgFlow(3,1,0.0);
// 	double focalLength = 3.7*320.0/5.76; // (focal length mm)*(img width px)/(ccd width mm)
// 	double maxViewAngle = 0;
// 	for(int i=0; i<frame1_features.size(); i++)
// 	{
// 		if(featureCorrespondence[i])
// 		{
// 			// project the points onto the unit sphere;
// 			Array2D<double> p1(3,1), p2(3,1);
// 			p1[0][0] = frame1_features[i].x;
// 			p1[1][0] = frame1_features[i].y;
// 			p1[2][0] = focalLength;
// 			p1 = 1.0/norm2(p1)*p1;
// 
// 			p2[0][0] = frame2_features[i].x;
// 			p2[1][0] = frame2_features[i].y;
// 			p2[2][0] = focalLength;
// 			p2 = 1.0/norm2(p2)*p2;
// 
// 			p1List.push_back(p1.copy());
// 			p2List.push_back(p2.copy());
// 			avgFlow += 1.0/dt*(p2-p1);
// 
// 			double viewAngle = acos(matmultS(transpose(p1),matmult(transpose(curRotMat),normDir)));
// 			if(viewAngle > maxViewAngle)
// 				maxViewAngle = viewAngle;
// 		}
// 	}
// 	avgFlow = 1.0/p1List.size()*avgFlow;
// 
// 	// Flow calc taken from
// 	// Herisse et al (2012) - Landing a VTOL Unmanned Aerial Vehicle on a Moving Platform Using Optical Flow
// 	double viewAngle = maxViewAngle;
// 	double lambda = pow(sin(viewAngle),2)/(4.0-pow(sin(viewAngle),2));
// 	Array2D<double> LambdaInv(3,3,0.0);
// 	LambdaInv[0][0] = LambdaInv[1][1] = lambda;
// 	LambdaInv[2][2] = 0.5;
// 	LambdaInv = 4.0/PI/pow(sin(lambda),4)*LambdaInv;
// 
// 	normDir[2][0] = 1;
// 	Array2D<double> transFlow = -1.0*matmult(LambdaInv,
// 										matmult(curRotMat,
// 											avgFlow + PI*pow(sin(viewAngle),2)*cross(curAngularVel, matmult(transpose(curRotMat),normDir))));
// printArray("transFlow: \t",transpose(transFlow));
// 
// 	int numFound = 0;
// 	// draw flow field
// 	for(int i=0; i<frame1_features.size(); i++)
// 	{
// 		if(!featureCorrespondence[i])
// 			continue;
// 
// 		numFound++;
// 
// 		int lineThickness = 1;
// 		cv::Scalar lineColor(60,125,125);
// 		cv::Point2f p,q;
// 		p.x = frame1_features[i].x;
// 		p.y = frame1_features[i].y;
// 		q.x = frame2_features[i].x;
// 		q.y = frame2_features[i].y;
// 
// 		circle(mCurImage,p,5,cv::Scalar(0,125,125),-1);
// 		line(mCurImage, p, q, lineColor, lineThickness, CV_AA, 0);
// 	}
// }

void VisionProcessor::enableIbvs(bool enable)
{
	mUseIbvs = enable;
	if(mUseIbvs)
	{
		Log::alert("Turning IBVS on");
		String str = String()+" " + mStartTime.getElapsedTimeMS() + "\t-605\t";
		mQuadLogger->addLine(str,PC_UPDATES);
		mLastImgFoundTime.setTime();
		mFirstImageProcessed = false;
	}
	else
	{
		Log::alert("Turning IBVS off");
		String str = String() + mStartTime.getElapsedTimeMS() + "\t-606\t";
		mQuadLogger->addLine(str,PC_UPDATES);
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

} // namespace Quadrotor
} // namespace ICSL
