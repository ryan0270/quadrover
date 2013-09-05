#ifndef ICSL_VIDEOMAKER_H
#define ICSL_VIDEOMAKER_H
#include <sched.h>
#include <queue>
#include <thread>
#include <mutex>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "toadlet/egg.h"

#include "CommManager.h"
#include "SensorManager.h"
#include "Time.h"

namespace ICSL{
namespace Quadrotor{

class VideoMaker : 	public SensorManagerListener,
					public CommManagerListener
{
	public:
		VideoMaker();
		virtual ~VideoMaker();

		void initialize();
		void start(){ thread th(&VideoMaker::run, this); th.detach(); }
		void run();
		void shutdown();
		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

		// for SensorManagerListener
		void onNewSensorUpdate(shared_ptr<IData> const &data);

		// from CommManagerListener
		void onNewCommMotorOn(){mMotorOn = true;}
		void onNewCommMotorOff(){mMotorOn = false;}

	protected:
		bool mRunning, mDone;
		bool mMotorOn;
		std::queue<shared_ptr<DataImage> > mImageQueue;
//		std::queue<shared_ptr<ImageMatchData> > mImageQueue;

		Time mLastImageTime;
		std::mutex mMutex_imageQueue;
		int mThreadPriority, mScheduler;
};
} // namespace Quadrotor
} // namespace ICSL

#endif // ICSL_VIDEOMAKER_H
