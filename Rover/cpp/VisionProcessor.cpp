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
	mCurImageHSV.create(240, 320, CV_8UC3); mCurImageHSV = cv::Scalar(0);
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

void ImageGrabber::copyImageHSV(cv::Mat *dstImage)
{
	mMutex_image.lock();
	mCurImageHSV.copyTo(*dstImage);
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
		if(!mIsBottleneck)
		{
			cvtColor(newImg, mCurImageHSV, CV_BGR2HSV);
			cvtColor(newImg, mCurImageGray, CV_BGR2GRAY);
			mImgConversionDone = true;
		}
		mMutex_image.unlock();
		mNewImageReady = true;

		sys.msleep(1);
	}

	if(cap.isOpened())
	{
		cap.set(CV_CAP_PROP_ANDROID_FLASH_MODE,CV_CAP_ANDROID_FLASH_MODE_OFF); // for now just leave this on the whole time
		cap.release();
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
	mImgViewType = 0;
	mCurImage.create(240, 320, CV_8UC3); mCurImage = cv::Scalar(0);
	mCurImageGray.create(240, 320, CV_8UC1); mCurImageGray = cv::Scalar(0);
	mCurImageGray.copyTo(mLastImageGray);

	mTempColor.create(240,320,CV_8UC1);
	mTempColor.copyTo(mTempSum);
	mTempColor.copyTo(mChanH);
	mTempColor.copyTo(mChanS);
	mTempColor.copyTo(mChanV);
	mTempColor.copyTo(mTempS);
	mTempColor.copyTo(mTempV);

//	mBlobDetectRegions.resize(4);
//	for(int i=0; i<mBlobDetectRegions.size(); i++)
//	{
//		mBlobDetectRegions[i].x = 0;
//		mBlobDetectRegions[i].y = 0;
//		mBlobDetectRegions[i].width = mCurImage.cols;
//		mBlobDetectRegions[i].height = mCurImage.rows;
//	}

	mBoxCenters.resize(4);
	for(int i=0; i<4; i++)
		mBoxCenters[i] = cv::Point(0,0);
	mFiltBoxColorCenter.resize(4);
	mFiltBoxColorHalfWidth.resize(4);
	mFiltBoxColorCenter[0] = 30; mFiltBoxColorHalfWidth[0] = 10;
	mFiltBoxColorCenter[1] = 75; mFiltBoxColorHalfWidth[1] = 10;
	mFiltBoxColorCenter[2] = 110; mFiltBoxColorHalfWidth[2] = 10;
	mFiltBoxColorCenter[3] = 170; mFiltBoxColorHalfWidth[3] = 10;
	mFiltSatMin = 100; mFiltSatMax = 255;
	mFiltValMin = 50; mFiltValMax = 255;
	mFiltCircMin = 0; mFiltCircMax = 100;
	mFiltConvMin = 75; mFiltConvMax = 125;
	mFiltAreaMin = 50; mFiltAreaMax = 9999999;
	mImgProcTimeUS = 0;

	mFiltBoxColorCenterActive.resize(4);
	for(int i=0; i<4; i++)
		mFiltBoxColorCenterActive[i] = mFiltBoxColorCenter[i];
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
	mTempSum.release();
	mTempColor.release();
	mChanH.release();
	mChanS.release();
	mChanV.release();
	mTempS.release();
	mTempV.release();
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
		if(mImageGrabber.imageConversionDone())
		{
			mImageGrabber.copyImageHSV(&mCurImage);
			mImageGrabber.copyImageGray(&mCurImageGray);
		}
		else
		{
			mImageGrabber.copyImage(&mCurImage);
			cvtColor(mCurImage, mCurImageGray, CV_BGR2GRAY);
			cvtColor(mCurImage, mCurImage, CV_BGR2HSV);
		}

//		if(mUseIbvs || mImgViewType == 1)
			processImage(imgAtt, rotVel);
		mMutex_image.unlock();
		mImgProcTimeUS = procStart.getElapsedTimeUS();
		{
			String str = String()+" "+mStartTime.getElapsedTimeMS() + "\t-600\t" + mImgProcTimeUS;
			mQuadLogger->addLine(str,CAM_RESULTS);
		}

		mCurImageGray.copyTo(mLastImageGray);

		sys.msleep(1);
	}

	mFinished = true;
}

void VisionProcessor::processImage(Array2D<double> const &imgAtt, Array2D<double> const &rotVel)
{
	Array2D<double> curRotMat = createIdentity(3);
	Array2D<double> curAngularVel(3,1,0.0);
	Array2D<double> normDir(3,1,0.0);
	double dt = mLastProcessTime.getElapsedTimeUS()/1.0e6;
	mLastProcessTime.setTime();

	cv::Mat temp1, temp2;
//	cv::pyrDown(mLastImageGray,temp1);
//	cv::pyrDown(mCurImageGray,temp2);
	mLastImageGray.copyTo(temp1);
	mCurImageGray.copyTo(temp2);

	double pyrScale = 0.5;
	int levels = 3;
	int winsize = 5;
	int iterations = 3;
	int polyN = 5;
	double polySigma = 1.1;
	int flags = 0;
	
	cv::Size sz = temp1.size();
	vector<cv::Rect> masks;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			cv::Rect mask((int)(i*sz.width/3.0+0.5), (int)(j*sz.height/3.0+0.5), (int)(sz.width/3.0+0.5), (int)(sz.height/3.0+0.5));
			masks.push_back(mask);
		}
	}
	cv::Mat flow(temp1.size(),CV_32FC2);
//	cv::Mat flow;
//	for(int i=0; i<masks.size(); i++)
//		cv::calcOpticalFlowFarneback(temp1(masks[i]), temp2(masks[i]), flow(masks[i]), pyrScale, levels, winsize, iterations, polyN, polySigma, flags);
	tbb::parallel_for(size_t(0), size_t(masks.size()), [=](size_t i) {
			cv::calcOpticalFlowFarneback(temp1(masks[i]), temp2(masks[i]), flow(masks[i]), pyrScale, levels, winsize, iterations, polyN, polySigma, flags);
			});

	cv::Scalar meanFlow = cv::mean(flow);
	Log::alert(String()+"flow: "+meanFlow[0]+" \t"+meanFlow[1]);

	for(int j=0; j<flow.rows; j+=30)
		for(int i=0; i<flow.cols; i+=30)
		{
			cv::Point2f p1(i,j);
			cv::Vec2b flw= flow.at<float>(j,i);
			cv::Point2f p2(i+flw[0],j+flw[1]);
			p2.x = max(0,min((int)p2.x, sz.width));
			p2.y = max(0,min((int)p2.y, sz.height));

			circle(mCurImage,p1,2,cv::Scalar(0,125,125),-1);
			line(mCurImage,p1,p2,cv::Scalar(0,180,0));
		}
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
// printArray(transpose(transFlow),"transFlow: \t");
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

void VisionProcessor::setBoxColorCenters(Collection<int> const &data)
{
	for(int i=0; i<data.size(); i++)
	{
		mFiltBoxColorCenter[i] = data[i];
		mFiltBoxColorCenterActive[i] = data[i];
	}
}

void VisionProcessor::setBoxColorHalfWidth(Collection<int> const &data)
{
	for(int i=0; i<data.size(); i++)
		mFiltBoxColorHalfWidth[i] = data[i];
}

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
		for(int boxID=0; boxID<mFiltBoxColorCenterActive.size(); boxID++)
		{
//			mBlobDetectRegions[boxID].x = 0;
//			mBlobDetectRegions[boxID].y = 0;
//			mBlobDetectRegions[boxID].width = mCurImage.cols;
//			mBlobDetectRegions[boxID].height = mCurImage.rows;
			mFiltBoxColorCenterActive[boxID] = mFiltBoxColorCenter[boxID];
		}
		mFirstImageProcessed = false;
	}
}

Collection<int> VisionProcessor::getVisionParams()
{
	Collection<int> p;
	p.push_back(mFiltBoxColorCenterActive[0]);
	p.push_back(mFiltBoxColorHalfWidth[0]);
	p.push_back(mFiltBoxColorCenterActive[1]);
	p.push_back(mFiltBoxColorHalfWidth[1]);
	p.push_back(mFiltBoxColorCenterActive[2]);
	p.push_back(mFiltBoxColorHalfWidth[2]);
	p.push_back(mFiltBoxColorCenterActive[3]);
	p.push_back(mFiltBoxColorHalfWidth[3]);
	p.push_back(mFiltSatMin);
	p.push_back(mFiltSatMax);
	p.push_back(mFiltValMin);
	p.push_back(mFiltValMax);
	p.push_back(mFiltCircMin);
	p.push_back(mFiltCircMax);
	p.push_back(mFiltConvMin);
	p.push_back(mFiltConvMax);

	return p;
}

void VisionProcessor::setVisionParams(Collection<int> const &p)
{
	mFiltBoxColorCenter[0] = p[0];
	mFiltBoxColorHalfWidth[0] = p[1];
	mFiltBoxColorCenter[1] = p[2];
	mFiltBoxColorHalfWidth[1] = p[3];
	mFiltBoxColorCenter[2] = p[4];
	mFiltBoxColorHalfWidth[2] = p[5];
	mFiltBoxColorCenter[3] = p[6];
	mFiltBoxColorHalfWidth[3] = p[7];
	mFiltSatMin = p[8];
	mFiltSatMax = p[9];
	mFiltValMin = p[10];
	mFiltValMax = p[11];
	mFiltCircMin = p[12];
	mFiltCircMax = p[13];
	mFiltConvMin = p[14];
	mFiltConvMax = p[15];

	for(int i=0; i<mFiltBoxColorCenter.size(); i++)
		mFiltBoxColorCenterActive[i] = mFiltBoxColorCenter[i];
}

void VisionProcessor::setSatMin(int val)
{
	mFiltSatMin = val;
	Log::alert(String()+"Sat min set to "+mFiltSatMin);
}

void VisionProcessor::setSatMax(int val)
{
	mFiltSatMax = val;
	Log::alert(String()+"Sat max set to "+mFiltSatMax);
}

void VisionProcessor::setValMin(int val)
{
	mFiltValMin = val;
	Log::alert(String()+"Val Min set to "+mFiltValMin);
}

void VisionProcessor::setValMax(int val)
{
	mFiltValMax = val;
	Log::alert(String()+"Val Max set to "+mFiltValMax);
}

void VisionProcessor::setCircMin(int val)
{
	mFiltCircMin = val;
	Log::alert(String()+"Circ Min set to "+mFiltCircMin);
}

void VisionProcessor::setCircMax(int val)
{
	mFiltCircMax = val;
	Log::alert(String()+"Circ Max set to "+mFiltCircMax);
}

void VisionProcessor::setConvMin(int val)
{
	mFiltConvMin = val;
	Log::alert(String()+"Conv Min set to "+mFiltConvMin);
}

void VisionProcessor::setConvMax(int val)
{
	mFiltConvMax = val;
	Log::alert(String()+"Conv Max set to "+mFiltConvMax);
}

void VisionProcessor::setAreaMin(int val)
{
	mFiltAreaMin = val;
	Log::alert(String()+"Area Min set to "+mFiltAreaMin);
}

void VisionProcessor::setAreaMax(int val)
{
	mFiltAreaMax = val;
	Log::alert(String()+"Area Max set to "+mFiltAreaMax);
}

void VisionProcessor::setViewType(int val)
{
	mImgViewType = val;
	Log::alert(String()+"Image view set to "+mImgViewType);
}

void VisionProcessor::onNewCommImgProcBoxColorCenter(Collection<int> const &data)
{
	setBoxColorCenters(data);
}

void VisionProcessor::onNewCommImgProcBoxColorHalfRange(Collection<int> const &data)
{
	setBoxColorHalfWidth(data);
}

void VisionProcessor::onNewCommImgProcSatMin(int val)
{
	setSatMin(val);
}

void VisionProcessor::onNewCommImgProcSatMax(int val)
{
	setSatMax(val);
}

void VisionProcessor::onNewCommImgProcValMin(int val)
{
	setValMin(val);
}

void VisionProcessor::onNewCommImgProcValMax(int val)
{
	setValMax(val);
}

void VisionProcessor::onNewCommImgProcCircMin(int val)
{
	setCircMin(val);
}

void VisionProcessor::onNewCommImgProcCircMax(int val)
{
	setCircMax(val);
}

void VisionProcessor::onNewCommImgProcConvMin(int val)
{
	setConvMin(val);
}

void VisionProcessor::onNewCommImgProcConvMax(int val)
{
	setConvMax(val);
}

void VisionProcessor::onNewCommImgProcAreaMin(int val)
{
	setAreaMin(val);
}

void VisionProcessor::onNewCommImgProcAreaMax(int val)
{
	setAreaMax(val);
}

void VisionProcessor::onNewCommImgViewType(int val)
{
	setViewType(val);
}

} // namespace Quadrotor
} // namespace ICSL
