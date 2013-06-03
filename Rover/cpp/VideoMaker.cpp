#include "VideoMaker.h"

namespace ICSL {
namespace Quadrotor {

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
		System sys;
		while(!mDone)
		{
			Log::alert("VideoMaker waiting");
			sys.msleep(100); // this can take a while if we are saving a lot of images
		}

		Log::alert("-------------------------- Video maker shutdown done ----------------------");
	}

	void VideoMaker::run()
	{
		mRunning = true;
		System sys;

		bool firstImage = true;
		int id = -1;
		cv::Mat img;
		sched_param sp;
		sp.sched_priority = mThreadPriority;
		sched_setscheduler(0, mScheduler, &sp);
		while(mRunning)
		{
			mMutex_imgQueue.lock();
			if(mImgQueue.size() > 1)
				Log::alert(String()+"Backlog: "+mImgQueue.size());
			while(!mImgQueue.empty())
			{
				shared_ptr<ImageMatchData> data = mImgQueue.front();

				id = data->imgData1->id;
				String filename = String()+"/sdcard/RoverService/video/img_"+id+".bmp";
				cv::Mat img = *(data->imgData1->img);
				mImgQueue.pop();
  				mMutex_imgQueue.unlock();
				cv::imwrite(filename.c_str(), img);
				mMutex_imgQueue.lock();
				id++;
			}
			mMutex_imgQueue.unlock();
			sys.msleep(1);
		}

		mDone = true;
	}

	void VideoMaker::onImageProcessed(shared_ptr<ImageMatchData> const data)
	{
		if(mLastImgTime.getElapsedTimeMS() > 10 
				&& mMotorOn
				)
		{
			mLastImgTime.setTime();
			mMutex_imgQueue.lock();
			mImgQueue.push(data);
			mMutex_imgQueue.unlock();
		}
	}

} // namespace Quadrotor
} // namespace ICSL
