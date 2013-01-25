#include "VisionProcessor.h"

using namespace toadlet;
//using namespace cv;
using toadlet::egg::String;
using namespace TNT;

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

	mLastImage.create(240, 320, CV_8UC3); mLastImage = cv::Scalar(0);
	mLastImageHSV.create(240, 320, CV_8UC3); mLastImageHSV = cv::Scalar(0);
	mLastImageGray.create(240, 320, CV_8UC1); mLastImageGray = cv::Scalar(0);

	mAttObserver = NULL;
}

void ImageGrabber::copyImage(cv::Mat *dstImage)
{
	mMutex_image.lock();
	mLastImage.copyTo(*dstImage);
	mNewImageReady = false;
	mMutex_image.unlock();
}

void ImageGrabber::copyImageHSV(cv::Mat *dstImage)
{
	mMutex_image.lock();
	mLastImageHSV.copyTo(*dstImage);
	mNewImageReady = false;
	mMutex_image.unlock();
}

void ImageGrabber::copyImageGray(cv::Mat *dstImage)
{
	mMutex_image.lock();
	mLastImageGray.copyTo(*dstImage);
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

	mLastImage.release();
	
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
		cap.set(CV_CAP_PROP_EXPOSURE, -4);
//		cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 5);
//		cap.set(CV_CAP_PROP_ANDROID_ANTIBANDING, CV_CAP_ANDROID_ANTIBANDING_OFF);
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
		newImg.copyTo(mLastImage);
		mImgConversionDone = false;
		if(!mIsBottleneck)
		{
			cvtColor(newImg, mLastImageHSV, CV_BGR2HSV);
			cvtColor(newImg, mLastImageGray, CV_BGR2GRAY);
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
	mLastImage.create(240, 320, CV_8UC3); mLastImage = cv::Scalar(0);
	mLastImageGray.create(240, 320, CV_8UC1); mLastImageGray = cv::Scalar(0);

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
//		mBlobDetectRegions[i].width = mLastImage.cols;
//		mBlobDetectRegions[i].height = mLastImage.rows;
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
	mLastImage.release();
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
	mLastImage.copyTo(*outImage);
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
			mImageGrabber.copyImageHSV(&mLastImage);
			mImageGrabber.copyImageGray(&mLastImageGray);
		}
		else
		{
			mImageGrabber.copyImage(&mLastImage);
			cvtColor(mLastImage, mLastImageGray, CV_BGR2GRAY);
			cvtColor(mLastImage, mLastImage, CV_BGR2HSV);
		}

		if(mUseIbvs || mImgViewType == 1)
			processImage(imgAtt, rotVel);
		mMutex_image.unlock();
		mImgProcTimeUS = Time::calcDiffUS(procStart,Time());
		{
			String str = String()+" "+mStartTime.getElapsedTimeMS() + "\t-600\t" + mImgProcTimeUS;
			mQuadLogger->addLine(str,CAM_RESULTS);
		}

		sys.msleep(1);
	}

	mFinished = true;
}

// imgAtt is only used to pass to the VisionProcessorListeners
void VisionProcessor::processImage(Array2D<double> const &imgAtt, Array2D<double> const &rotVel)
{
	// ensure these are sized correctly
	// create only allocates new space if the size is different
	mChanH.create(mLastImage.size(), CV_8UC1);
	mChanS.create(mLastImage.size(), CV_8UC1);
	mChanV.create(mLastImage.size(), CV_8UC1);

	int fromToH[] = {0, 0};
	int fromToS[] = {1, 0};
	int fromToV[] = {2, 0};
	mixChannels(&mLastImage,1,&mChanH,1,fromToH,1);
	mixChannels(&mLastImage,1,&mChanS,1,fromToS,1);
	mixChannels(&mLastImage,1,&mChanV,1,fromToV,1);

	// get rid of white and black background
	threshold(mChanS,mTempS,mFiltSatMax,0,cv::THRESH_TOZERO_INV);
	threshold(mTempS,mTempS,mFiltSatMin,1,cv::THRESH_BINARY);
	threshold(mChanV,mTempV,mFiltValMax,0,cv::THRESH_TOZERO_INV);
	threshold(mTempV,mTempV,mFiltValMin,1,cv::THRESH_BINARY);

	multiply(mTempS,mChanH,mChanH);
	multiply(mTempV,mChanH,mChanH);

	// Filter to have only the colors we believe to exist
	mTempSum.create(mLastImage.size(), CV_8UC1);
	mTempColor.create(mLastImage.size(), CV_8UC1);
	mTempSum = cv::Scalar(0);
	double alphaHue = 0.05/(1+0.05);
	double alphaPos = 0.2;
	alphaPos = 1;
	int numFound = 0;
	Collection<bool> boxFound(mBoxCenters.size());
	for(int boxID=0; boxID<mFiltBoxColorCenterActive.size(); boxID++)
	{
		threshold(mChanH,mTempColor,mFiltBoxColorCenterActive[boxID]+mFiltBoxColorHalfWidth[boxID],0,cv::THRESH_TOZERO_INV);
		threshold(mTempColor,mTempColor,mFiltBoxColorCenterActive[boxID]-mFiltBoxColorHalfWidth[boxID],1,cv::THRESH_BINARY);

		// use the gray image as the selected pixels
		multiply(mLastImageGray,mTempColor,mTempColor);
		mTempSum += mTempColor;

		if(mUseIbvs)
		{
			vector<vector<cv::Point> > contours;
			cv::findContours(mTempColor, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
			cv::drawContours(mLastImage, contours, -1, cv::Scalar(0, 255, 75), 2);
			double area, maxArea;
			maxArea = 0;
			cv::Point2f center;
			int maxAreaContourIndex = 0;
			for(int i=0; i<contours.size(); i++)
			{
				area = abs(cv::contourArea(contours[i]));
				if(area > maxArea && area < mFiltAreaMax)
				{
					maxArea = area;
					maxAreaContourIndex = i;
					center.x = center.y = 0;
					for(int j=0; j<contours[i].size(); j++)
					{
						center.x += contours[i][j].x;
						center.y += contours[i][j].y;
					}
					center.x /= contours[i].size();
					center.y /= contours[i].size();
//					cv::Moments m = cv::moments(contours[i]);
//					center.x = m.m10;
//					center.y = m.m01;
				}
			}
			double radius = sqrt(maxArea)/2;

			if(maxArea > mFiltAreaMin)
			{
				numFound++;
				cv::circle(mLastImage, center, 3, cv::Scalar(20, 125, 125), CV_FILLED);
//				cv::Point p1, p2, p3, p4;
//				p1 = p2 = p3 = p4 = center;
//				p1.x -= radius; p2.x += radius;
//				p3.y -= radius; p4.y += radius;
//				line(mLastImage,p1,p2,cv::Scalar(125,0,0),3);
//				line(mLastImage,p3,p4,cv::Scalar(125,0,0),3);

				cv::Mat mask(mChanH.size(),mChanH.type(),cv::Scalar(0));
				drawContours(mask, contours, maxAreaContourIndex, cv::Scalar(1,1,1), CV_FILLED);
				cv::Scalar meanHue1 = mean(mChanH,mask);
				double meanHue = meanHue1[0];
				mask.release();

				mFiltBoxColorCenterActive[boxID] = alphaHue*meanHue+(1-alphaHue)*mFiltBoxColorCenterActive[boxID];
				if(mFirstImageProcessed)
					mBoxCenters[boxID] = alphaPos*center+(1-alphaPos)*mBoxCenters[boxID];
				else
					mBoxCenters[boxID] = center;
				boxFound[boxID] = true;
			}
		}
	}

	if(mImgViewType == 1)
		mTempSum.copyTo(mLastImage);
//		mLastImage = mTempSum.clone();

	if(numFound == 4)
		mLastImgFoundTime.setTime();
	else if(mUseIbvs)
	{
		Time now;
		if(now.getMS() - mLastImgFoundTime.getMS() > 2e3)
		{
			mUseIbvs = false;
			mFirstImageProcessed = false;
			Log::alert("Lost the image");
			for(int boxID=0; boxID<mFiltBoxColorCenterActive.size(); boxID++)
			{
//				mBlobDetectRegions[boxID].x = 0;
//				mBlobDetectRegions[boxID].y = 0;
//				mBlobDetectRegions[boxID].width = mLastImage.cols;
//				mBlobDetectRegions[boxID].height = mLastImage.rows;
				mFiltBoxColorCenterActive[boxID] = mFiltBoxColorCenter[boxID];
			}

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onImageLost();
		}
	}

	if(mUseIbvs && (mFirstImageProcessed || numFound == 4))
	{
		mFirstImageProcessed = true;
		for(int i=0; i<mListeners.size(); i++)
			mListeners[i]->onImageProcessed(mBoxCenters, boxFound, imgAtt, rotVel);

		{
			String str = String()+" "+mStartTime.getElapsedTimeMS()+"\t-601\t";
			for(int boxID=0; boxID<mBoxCenters.size(); boxID++)
//			{
//				if(boxFound[boxID])
					str = str+mBoxCenters[boxID].x+"\t"+mBoxCenters[boxID].y+"\t";
//				else
//					str = str+(-1*mBoxCenters[boxID].x)+"\t"+(-1*mBoxCenters[boxID].y)+"\t";
//			}
			mQuadLogger->addLine(str,CAM_RESULTS);
		}
	}
}


// void VisionProcessor::processImage(Array2D<double> const &imgAtt, Array2D<double> const &rotVel)
// {
// //	cvtColor(mLastImage,mLastImage,CV_BGR2HSV);
// 
// 	Time start;
// 	cv::Mat imgGray;
// 	cvtColor(mLastImage,imgGray,CV_HSV2BGR);
// 	cvtColor(imgGray,imgGray,CV_BGR2GRAY);
// 
//  	// ensure these are sized correctly
//  	// create only allocates new space if the size is different
//  	mChanH.create(mLastImage.size(), CV_8UC1);
//  	mChanS.create(mLastImage.size(), CV_8UC1);
//  	mChanV.create(mLastImage.size(), CV_8UC1);
//  
//  	int fromToH[] = {0, 0};
//  	int fromToS[] = {1, 0};
//  	int fromToV[] = {2, 0};
//  	mixChannels(&mLastImage,1,&mChanH,1,fromToH,1);
//  	mixChannels(&mLastImage,1,&mChanS,1,fromToS,1);
//  	mixChannels(&mLastImage,1,&mChanV,1,fromToV,1);
// 
// 	  // get rid of white and black background
//  	threshold(mChanS,mTempS,mFiltSatMax,0,cv::THRESH_TOZERO_INV);
//  	threshold(mTempS,mTempS,mFiltSatMin,1,cv::THRESH_BINARY);
//  	threshold(mChanV,mTempV,mFiltValMax,0,cv::THRESH_TOZERO_INV);
//  	threshold(mTempV,mTempV,mFiltValMin,1,cv::THRESH_BINARY);
//  
//  	multiply(mTempS,mChanH,mChanH);
//  	multiply(mTempV,mChanH,mChanH);
// 
//  	// Filter to have only the colors we believe to exist
//  	mTempSum.create(mLastImage.size(), CV_8UC1);
//  	mTempColor.create(mLastImage.size(), CV_8UC1);
//  	mTempSum = cv::Scalar(0);
// 
// 	vector<cv::Point2f> corners;
//  	Collection<bool> boxFound(mBoxCenters.size());
//  	for(int boxID=0; boxID<mFiltBoxColorCenterActive.size(); boxID++)
//  	{
//  		threshold(mChanH,mTempColor,mFiltBoxColorCenterActive[boxID]+mFiltBoxColorHalfWidth[boxID],0,cv::THRESH_TOZERO_INV);
//  		threshold(mTempColor,mTempColor,mFiltBoxColorCenterActive[boxID]-mFiltBoxColorHalfWidth[boxID],1,cv::THRESH_BINARY);
// 
// 		// mTempColor now contains a mask of pixels that should hold the desired square. 
// 		// Grow this mask a bit and search over it for the corners
// 		mTempSum += mTempColor;
// 	 	cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5));
// 		dilate(mTempColor, mTempColor,kernel,cv::Point(-1,-1),1);
// 
// 		vector<cv::Point2f> newCorners;
// 		cv::goodFeaturesToTrack(imgGray, newCorners, 5, 0.001, 20, mTempColor, 3, true);
// 		for(int c=0; c<newCorners.size(); c++)
// 			corners.push_back(newCorners[c]);
// 	}
// //	mLastImage = imgGray.clone();
// 
// 	for(int i=0; i<corners.size(); i++)
// 	{
// 		int size = 8;
// 		cv::Point2f p1, p2, p3, p4;
// 		p1 = p2 = p3 = p4 = corners[i];
// 		p1.x -= size; p2.x += size;
// 		p3.y -= size; p4.y += size;
// 		line(mLastImage,p1,p2,cv::Scalar(125,0,255),2);
// 		line(mLastImage,p3,p4,cv::Scalar(125,0,255),2);
// 	}
// 
// //	String str = "--------------------------------";
// //	str = str+"Corners:\n";
// //	for(int i=0; i<corners.size(); i++)
// //		str = str+corners[i].x+" \t"+corners[i].y+"\n";
// //	String str = String()+"Time: "+start.getElapsedTimeMS();
// //	Log::alert(str);
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
//			mBlobDetectRegions[boxID].width = mLastImage.cols;
//			mBlobDetectRegions[boxID].height = mLastImage.rows;
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
