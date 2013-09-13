#include "VideoMaker.h"

namespace ICSL {
namespace Quadrotor {
using namespace toadlet::egg;

	VideoMaker::VideoMaker()
	{
		mRunning = false;
		mDone = true;

		mScheduler = SCHED_NORMAL;
		mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

		mMotorOn = false;
	}

	VideoMaker::~VideoMaker()
	{
	}

	void VideoMaker::initialize()
	{
	}

	void VideoMaker::shutdown()
	{
		Log::alert("-------------------------- Video maker shutdown started ----------------------");
		mRunning = false;
		while(!mDone)
		{
			Log::alert("VideoMaker waiting");
			System::msleep(100); // this can take a while if we are saving a lot of images
		}

		Log::alert("-------------------------- Video maker shutdown done ----------------------");
	}

	void VideoMaker::run()
	{
		mRunning = true;

		bool firstImage = true;
		int id = -1;
		cv::Mat image;
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			mMutex_imageQueue.lock();
			while(!mImageQueue.empty())
			{
//				if(mImageQueue.size() > 1)
//					Log::alert(String()+"Backlog: "+mImageQueue.size());

//				shared_ptr<ImageMatchData> data = mImageQueue.front();
				shared_ptr<DataImage> data = mImageQueue.front();

				id = data->imageId;
				String filename = String()+"/sdcard/RoverService/video/image_"+id+".bmp";
				cv::Mat image = *(data->image);
				mImageQueue.pop();
  				mMutex_imageQueue.unlock();
				cv::imwrite(filename.c_str(), image);
				mMutex_imageQueue.lock();
				id++;
			}
			mMutex_imageQueue.unlock();
			System::msleep(1);
		}

		mDone = true;
	}

//	void VideoMaker::onImageProcessed(const shared_ptr<ImageMatchData> &data)
//	{
////		if( Time::calcDiffMS(mLastImageTime, data->imageData1->timestamp) > 100
////				&& mMotorOn
////				)
//		if(mMotorOn)
//		{
//			mMutex_imageQueue.lock();
//			mLastImageTime.setTime(data->imageData1->timestamp);
//			while(mImageQueue.size() > 100) 
//			{// This is to cap the RAM usage. It WILL delay the calling thread.
//				mMutex_imageQueue.unlock();
//				System::msleep(1);
//				mMutex_imageQueue.lock();
//			}
//			mImageQueue.push(data);
//			mMutex_imageQueue.unlock();
//		}
//	}

	void VideoMaker::onNewSensorUpdate(const shared_ptr<IData> &data)
	{
		if(data->type == DATA_TYPE_IMAGE && mMotorOn)
		{
			shared_ptr<DataImage> imgData = static_pointer_cast<DataImage>(data);
			if(mImageQueue.size() < 100)
				mImageQueue.push(imgData);
			else
				Log::alert("Chad can't get it up");
		}
	}

} // namespace Quadrotor
} // namespace ICSL
