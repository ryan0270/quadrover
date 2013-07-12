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
	mLastImageFeatureData = NULL;
}

VelocityEstimator::~VelocityEstimator()
{
}

void VelocityEstimator::shutdown()
{
	Log::alert("------------------------- VelocityEstimator shutdown started");
	mRunning = false;
	while(mDone != true)
		System::msleep(10);
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

	shared_ptr<ImageFeatureData> imgFeatureData;
	Time procTimer;
	double procTime;
	Array2D<double> velEst(3,1);
	double heightEst;
	while(mRunning)
	{
		if(mNewImageDataAvailable)
		{
			procTimer.setTime();
			imgFeatureData = mLastImageFeatureData;
			doVelocityEstimate(imgFeatureData, velEst, heightEst);
			mNewImageDataAvailable = false;

			procTime = procTimer.getElapsedTimeNS()/1.0e9;

			if(mQuadLogger != NULL)
			{
				String str = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_MAP_VEL_CALC_TIME + "\t"+procTime;
				mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
			}
		}

		System::msleep(1);
	}

	mDone = true;
}

// See eqn 98 in the Feb 25, 2013 notes
void VelocityEstimator::doVelocityEstimate(shared_ptr<ImageFeatureData> const &featureData,
										   Array2D<double> &velEst, 
										   double &heightEst) const
{
//	Log::alert("skipping velocity estimate");
}

void VelocityEstimator::onFeaturesFound(shared_ptr<ImageFeatureData> const &data)
{
	mLastImageFeatureData = data;
	mNewImageDataAvailable = true;
}

} // namespace Rover
} // namespace ICSL
