#include "VelocityEstimator.h"

namespace ICSL {
namespace Quadrotor {
using namespace std;
using namespace TNT;

VelocityEstimator::VelocityEstimator() : 
	mRotPhoneToCam(3,3,0.0),
	mRotCamToPhone(3,3,0.0)
{
	mDone = true;
	mRunning = false;

	mNewImageDataAvailable = false;
}

VelocityEstimator::~VelocityEstimator()
{
}

void VelocityEstimator::shutdown()
{
	Log::alert("------------------------- VelocityEstimator shutdown started");
	mRunning = false;
	this->join();
	Log::alert("------------------------- VelocityEstimator shutdown done");
}

void VelocityEstimator::initialize()
{
	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
}

void VelocityEstimator::run()
{
	mDone = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	shared_ptr<ImageMatchData> imgMatchData;
	while(mRunning)
	{
		if(mNewImageDataAvailable)
		{
			mMutex_imageData.lock();
			imgMatchData = mImageMatchData;
			mMutex_imageData.unlock();

			doVelocityEstimate(imgMatchData);

			mNewImageDataAvailable = false;
		}

		System::msleep(1);
	}

	mDone = true;
}

void VelocityEstimator::onImageProcessed(shared_ptr<ImageMatchData> const data)
{
	mMutex_imageData.lock();
	mImageMatchData = data;	
	mMutex_imageData.unlock();

	mNewImageDataAvailable = true;
}

// See eqn 98 in the Feb 25, 2013 notes
void VelocityEstimator::doVelocityEstimate(shared_ptr<ImageMatchData> const &matchData)
{
//	Log::alert("skipping velocity estimate");
}

} // namespace Rover
} // namespace ICSL
