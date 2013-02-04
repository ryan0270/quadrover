#include "QuadLogger.h"

using namespace std;

namespace ICSL{
namespace Quadrotor{
QuadLogger::QuadLogger()
{
	mDir = ".";
	mFilename = "quadLog.txt";
	mLogStream = NULL;
	mTypeMask = 0;
	mTypeMask = PC_UPDATES ;
//	mTypeMask |= STATE;
//	mTypeMask |= STATE_DES;
//	mTypeMask |= MOTORS;
//	mTypeMask |= OBSV_UPDATE;
//	mTypeMask |= OBSV_BIAS;
//	mTypeMask |= MAGNOMETER;
//	mTypeMask |= ACCEL;
//	mTypeMask |= GYRO;
//	mTypeMask |= CAM_RESULTS;
//	mTypeMask |= CAM_IMAGES;
	mPaused = false;
}

QuadLogger::~QuadLogger()
{
	close();
}

void QuadLogger::addLine(String const &str, uint16 type)
{
	if(mPaused)
		return;
	mMutex.lock();
	if(mTypeMask & type)
	{
		String str2= str + "\n";
		if(mLogStream != NULL)
			mLogStream->write((tbyte*)str2.c_str(), str2.length());
	}
	mMutex.unlock();
}
void QuadLogger::start()
{
	{
		String s = String() +"Starting logs at " + mDir+"/"+mFilename;
		Log::alert(s);
	}
	mLogStream = FileStream::ptr(new FileStream(mDir+"/"+mFilename, FileStream::Open_BIT_WRITE));

//	String str = "1\t2\t3\t4\t5\t6\t7\t8\t9\t10\n";
	String str = "";
	for(int i=0; i<20; i++)
		str = str+i+"\t";
	str = str+"\n";
	mLogStream->write((tbyte*)str.c_str(),str.length());

	// for easy indexing while post processing
	str = String() + "0\t-500\n";
	mLogStream->write((tbyte*)str.c_str(),str.length());
}

void QuadLogger::close()
{
	if(mLogStream != NULL)
	{
		mLogStream->close();
		mLogStream = NULL;
	}
}

}
}
