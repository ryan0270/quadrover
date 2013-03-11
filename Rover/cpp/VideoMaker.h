#ifndef ICSL_VIDEOMAKER_H
#define ICSL_VIDEOMAKER_H
#include <queue>

#include "opencv2/highgui/highgui.hpp"

#include "toadlet/egg.h"

#include "VisionProcessor.h"

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

		// for VisionProcessorListener
		void onImageProcessed(shared_ptr<ImageMatchData> const data);
		void onImageLost(){};

	protected:
		bool mRunning, mDone;
		std::queue<shared_ptr<cv::Mat> > mImgQueue;

		toadlet::egg::Mutex mMutex_imgQueue;
};
} // namespace Quadrotor
} // namespace ICSL

#endif // ICSL_VIDEOMAKER_H
