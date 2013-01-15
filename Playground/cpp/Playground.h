#ifndef CHADPHONE_H
#define CHADPHONE_H
#include <fstream>
#include <sstream>

#include <toadlet/egg.h>
using toadlet::int64;
using toadlet::uint64;
using toadlet::egg::String;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "TNT/tnt.h"
#include "TNT_Utils.h"
#include "constants.h"
#include "VisionProcessor.h"
#include "Time.h"
#include "QuadLogger.h"

namespace ICSL {
namespace Quadrotor {
class Playground: public toadlet::egg::Thread
{
public:
	Playground();
	virtual ~Playground();

	void initialize();
	void shutdown();

	void setNumCpuCores(int numCores){mNumCpuCores = numCores;}
	void setLogFilename(String name);
	void setLogDir(String dir);
	void startLogging();
	void stopLogging();
	
	// these functions are primarily for the jni interface
	void copyImageData(cv::Mat *m);
	int getImageProcTimeMS(){ return mVisionProcessor.getImageProcTimeMS();}

protected:
	bool mRunning, mDone;

	void run();

	VisionProcessor mVisionProcessor;

	QuadLogger mQuadLogger;

	TNT::Array2D<int> getCpuUsage();

	Time mStartTime;

	int mNumCpuCores;
}; // class Playground

} // namespace Quadrotor
} // namespace ICSL

#endif
