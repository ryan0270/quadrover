#ifndef ICSL_VIDEOMAKER_H
#define ICSL_VIDEOMAKER_H
#include <sched.h>
#include <queue>

#include "opencv2/highgui/highgui.hpp"

#include "toadlet/egg.h"

#include "VisionProcessor.h"
#include "Time.h"

namespace ICSL{
namespace Quadrotor{

class VideoMaker : public VisionProcessorListener,
				   public toadlet::egg::Thread
{
	public:
		VideoMaker();
		virtual ~VideoMaker();

		void initialize();
		void run();
		void shutdown();
		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

		// for VisionProcessorListener
		void onImageProcessed(shared_ptr<ImageMatchData> const data);
		void onImageTargetFound(shared_ptr<ImageTargetFindData> const data){};
		void onImageLost(){};

	protected:
		bool mRunning, mDone;
		std::queue<shared_ptr<cv::Mat> > mImgQueue;

		Time mLastImgTime;
		toadlet::egg::Mutex mMutex_imgQueue;
		int mThreadPriority, mScheduler;
};
} // namespace Quadrotor
} // namespace ICSL

#endif // ICSL_VIDEOMAKER_H
