#ifndef QUADLOGGER
#define QUADLOGGER

#include <toadlet/toadlet.h>
#include "Common.h"


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
	protected:
		String mDir, mFilename;
		uint32 mTypeMask;
		FileStream::ptr mLogStream;
		Mutex mMutex;
		bool mPaused;
};

}
}
#endif
