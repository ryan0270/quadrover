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

void VideoMaker::onNewSensorUpdate(const shared_ptr<IData> &data)
{
	if(data->type == DATA_TYPE_IMAGE
			&& mMotorOn
			)
	{
		shared_ptr<DataImage> imgData = static_pointer_cast<DataImage>(data);
		mMutex_imageQueue.lock();
		if(mImageQueue.size() < 100)
			mImageQueue.push(imgData);
		else
			Log::alert("Chad can't get it up");
		mMutex_imageQueue.unlock();
	}
}

} // namespace Quadrotor
} // namespace ICSL
