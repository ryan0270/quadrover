#ifndef QUADLOGGER
#define QUADLOGGER
#include <list>
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

using namespace std;

namespace ICSL {
namespace Quadrotor {
class QuadLogger
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

		void start();
		void close();
		void clearLog(){close(); start();}

		void pause(){mPaused = true;}
		void unpause(){mPaused = false;}

		void saveImageBuffer(list<shared_ptr<SensorDataImage> > const &dataBuffer);

		void setStartTime(Time time){mStartTime.setTime(time);}
	protected:
		String mDir, mFilename;
		uint32 mTypeMask;
		FileStream::ptr mLogStream;
		Mutex mMutex;
		bool mPaused;
		Time mStartTime;
};

}
}
#endif
