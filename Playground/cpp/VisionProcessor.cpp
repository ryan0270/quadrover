#include "VisionProcessor.h"

using namespace toadlet;
//using namespace cv;
using toadlet::egg::String;
using namespace TNT;

namespace ICSL {
namespace Quadrotor{

ImageGrabber::ImageGrabber()
{
	mRunning = false;
	mFinished = true;
	mIsBottleneck = false;
	mNewImageReady = false;

	mLastImage.create(240, 320, CV_8UC3); mLastImage = cv::Scalar(0);
}

void ImageGrabber::copyImage(cv::Mat *dstImage)
{
	mMutex_image.lock();
	mLastImage.copyTo(*dstImage);
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
		cap.retrieve(newImg);
		mMutex_image.lock();
		newImg.copyTo(mLastImage);
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


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
VisionProcessor::VisionProcessor()
{
	mRunning = false;
	mFinished = true;
	mUseIbvs = false;
	mFirstImageProcessed = false;
	mLastImage.create(240, 320, CV_8UC3); mLastImage = cv::Scalar(0);
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
	while(mRunning)
	{
		if(!mImageGrabber.isNewImageReady())
			mImageGrabber.markBottleneck(true); // I don't want this in the polling loop
		while(!mImageGrabber.isNewImageReady())
			sys.msleep(1);
		mMutex_image.lock();
		mImageGrabber.markBottleneck(false);

		Time procStart;
		mImageGrabber.copyImage(&mLastImage);

		processImage();
		mMutex_image.unlock();
		mImgProcTimeUS = procStart.getElapsedTimeUS();
		{
			String str = String()+" "+mStartTime.getElapsedTimeMS() + "\t-600\t" + mImgProcTimeUS;
			mQuadLogger->addLine(str,CAM_RESULTS);
		}

		sys.msleep(1);
	}

	mFinished = true;
}

// imgAtt is only used to pass to the VisionProcessorListeners
void VisionProcessor::processImage()
{
	System sys;
	sys.msleep(100);
	// do image processing here
	Log::alert("Doing image processing");
}

} // namespace Quadrotor
} // namespace ICSL
