#include "Playground.h"

using namespace std;
using namespace TNT;
using namespace toadlet;
using namespace ICSL::Constants;
using toadlet::uint64;
using toadlet::egg::String;

namespace ICSL {
namespace Quadrotor {
Playground::Playground()
{
	mRunning = false;
	mDone = true;

	mNumCpuCores = 1;
}

Playground::~Playground()
{
}

void Playground::initialize()
{
//	mVisionProcessor.setStartTime(mStartTime);
	mVisionProcessor.setQuadLogger(&mQuadLogger);
	mVisionProcessor.start();

	this->start(); // this should spawn a separate thread running the run() function

	Log::alert("Initialized");
}

void Playground::shutdown()
{
	mRunning = false;
//	this->join(); // join doesn't work correctly in NDK
	toadlet::egg::System sys;
	while(!mDone)
	{
		Log::alert("Main waiting");
		sys.msleep(10);
	}


	mVisionProcessor.shutdown();

	Log::alert(String("----------------- really dead -------------"));
}


void Playground::run()
{
	mRunning= true;
	System sys;
	mDone = false;

	Array2D<int> cpuUsagePrev(1,7,0.0), cpuUsageCur(1,7,0.0);

	while(mRunning) 
	{
		sys.msleep(10);
	}

	Log::alert(String("----------------- Playground runner dead -------------"));
	mDone = true;
}

void Playground::startLogging(){
	mQuadLogger.setFilename("log.txt");
	mQuadLogger.start();
}

void Playground::stopLogging()
{
	mQuadLogger.close();
}

void Playground::setLogFilename(String name)
{
	mQuadLogger.setFilename(name);
}

void Playground::setLogDir(String dir)
{
	mQuadLogger.setDir(dir);
	Log::alert("Log dir set to: " + mQuadLogger.getFullPath());
}

void Playground::copyImageData(cv::Mat *m)
{
	if(!mRunning) // use this as an indicator that we are shutting down
		return;

	mVisionProcessor.getLastImage(m);
}

} // namespace Quadrotor
} // namespace ICSL
