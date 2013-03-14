#ifndef QUADLOGGER
#define QUADLOGGER
#include <list>
#include <queue>
#include <sstream>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mxml.h"

#include <toadlet/toadlet.h>
#include "Common.h"
#include "Time.h"
#define ICSL_SENSOR_DATA_ONLY 
#include "SensorManager.h" 
#undef ICSL_SENSOR_DATA_ONLY
#define ICSL_IMAGEMATCHDATA_ONLY
#include "VisionProcessor.h"
#undef ICSL_IMAGEMATCHDATA_ONLY


namespace ICSL {
namespace Quadrotor {
using namespace std;
class QuadLogger : public toadlet::egg::Thread
{
	public:
		explicit QuadLogger();
		virtual ~QuadLogger();

		String getDir(){return mDir;}
		String getFilename(){return mFilename;}
		String getFullPath(){return mDir+"/"+mFilename;}
		uint32 getMask(){return mTypeMask;}

		void setDir(String dir){mDir = dir;}
		void setFilename(String name){mFilename = name;}
		void setMask(uint32 mask){mTypeMask = mask;}
		void addLine(String const &str, uint16 type);

		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
//		void start();
		void run();
		void close();
		void clearLog(){close(); start();}

		void saveImageBuffer(list<shared_ptr<SensorDataImage> > const &dataBuffer,
							 list<shared_ptr<ImageMatchData> > const &matchDataBuffer);

		void setStartTime(Time time){mStartTime.setTime(time);}

	protected:
		String mDir, mFilename;
		uint32 mTypeMask;
		FileStream::ptr mLogStream;
		Mutex mMutex_file, mMutex_logQueue;
		Time mStartTime;

		void generateMatlabHeader();

		int mThreadPriority, mScheduler;

		queue<String> mLogQueue;

		bool mRunning;
};

}
}
#endif
