#include "VideoMaker.h"
#include "sched.h"

namespace ICSL {
namespace Quadrotor {

	VideoMaker::VideoMaker()
	{
		mRunning = false;
		mDone = true;
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

		sched_param schedParam;
		schedParam.sched_priority = sched_get_priority_min(SCHED_NORMAL);;
		sched_setscheduler(0,SCHED_NORMAL,&schedParam);

		bool firstImage = true;
		int id = 0;
		while(mRunning)
		{
			mMutex_imgQueue.lock();
			if(mImgQueue.size() > 1)
				Log::alert(String()+"Backlog: "+mImgQueue.size());
			while(!mImgQueue.empty())
			{
				String filename = String()+"/sdcard/RoverService/video/img_"+id+".bmp";
				cv::Mat img = *(mImgQueue.front());
				mImgQueue.pop();
  				mMutex_imgQueue.unlock();
				cv::imwrite(filename.c_str(), img);
				mMutex_imgQueue.lock();
				id++;
			}
			mMutex_imgQueue.unlock();
			sys.msleep(10);
		}

		mDone = true;
	}

	void VideoMaker::onImageProcessed(shared_ptr<ImageMatchData> const data)
	{
		if(mLastImgTime.getElapsedTimeMS() > 90)
		{
//			mLastImgTime.setTime();
//			mMutex_imgQueue.lock();
//			mImgQueue.push(data->imgAnnotated);
//			mMutex_imgQueue.unlock();
		}
	}

} // namespace Quadrotor
} // namespace ICSL
