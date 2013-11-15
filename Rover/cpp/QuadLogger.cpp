#include "QuadLogger.h"

#include <unistd.h>

namespace ICSL{
namespace Quadrotor{
//using namespace std;
QuadLogger::QuadLogger()
{
	mDir = ".";
	mFilename = "quadLog.txt";
	mLogStream = NULL;
	mTypeMask = 0;
	mTypeMask |= LOG_FLAG_PC_UPDATES ;
//	mTypeMask |= LOG_FLAG_STATE;
//	mTypeMask |= LOG_FLAG_STATE_DES;
//	mTypeMask |= LOG_FLAG_MOTORS;
//	mTypeMask |= LOG_FLAG_OBSV_UPDATE;
//	mTypeMask |= LOG_FLAG_OBSV_BIAS;
//	mTypeMask |= LOG_FLAG_MAGNOMETER;
	mTypeMask |= LOG_FLAG_ACCEL;
//	mTypeMask |= LOG_FLAG_GYRO;
//	mTypeMask |= LOG_FLAG_PRESSURE;
	mTypeMask |= LOG_FLAG_CAM_RESULTS;
//	mTypeMask |= LOG_FLAG_CAM_IMAGES;
	mTypeMask |= LOG_FLAG_PHONE_TEMP;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

	mPaused = false;
	mRunning = false;;
	mDone = true;
}

QuadLogger::~QuadLogger()
{
}

void QuadLogger::shutdown()
{
	Log::alert("------------------------- QuadLogger shutdown started");
	mRunning = false;
	while(!mDone)
		System::msleep(10);
	Log::alert("------------------------- QuadLogger shutdown done");
}

void QuadLogger::run()
{
	mDone = false;
	mRunning = true;
	{
		String s = String() +"Starting logs at " + mDir+"/"+mFilename;
		Log::alert(s);
	}

	generateMatlabHeader();

	mLogStream = FileStream::ptr(new FileStream(mDir+"/"+mFilename, FileStream::Open_BIT_WRITE));

	String str = "";
	for(int i=0; i<20; i++)
		str = str+i+"\t";
	str = str+"\n";
	mLogStream->write((tbyte*)str.c_str(),str.length());

	// for easy indexing while post processing
	str = String() + "0\t-500\n";
	mLogStream->write((tbyte*)str.c_str(),str.length());

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	while(mLogQueue.size() > 0)
		mLogQueue.pop_front();

	String line;
	shared_ptr<LogEntry> entry;
	while(mRunning)
	{
		Time t;
		mMutex_logQueue.lock();
		int size = mLogQueue.size();
		if(size > 0)
			t.setTime(mLogQueue.front()->timestamp);
		mMutex_logQueue.unlock();
		while(size > 0 && !mPaused && t.getElapsedTimeMS() > 1.0e3 && mRunning)
		{
			mMutex_logQueue.lock();
			entry  = mLogQueue.front();
			mLogQueue.pop_front();
			mMutex_logQueue.unlock();

			if(mStartTime < entry->timestamp)
				line = String()+Time::calcDiffMS(mStartTime,entry->timestamp)+"\t";
			else
				line = String()+Time::calcDiffMS(entry->timestamp,mStartTime)+"\t";
			line = line+entry->id+"\t";
			line = line+entry->str;

			line = line+"\n";
			if(mLogStream != NULL)
				mLogStream->write((tbyte*)line.c_str(), line.length());

			mMutex_logQueue.lock();
			size = mLogQueue.size();
			if(size > 0)
				t.setTime(mLogQueue.front()->timestamp);
			mMutex_logQueue.unlock();
		}

		System::msleep(100);
	}


	// Clear the remainder of the queue
	mMutex_logQueue.lock(); int size = mLogQueue.size(); mMutex_logQueue.unlock();
	while(size > 0)
	{
		mMutex_logQueue.lock();
		entry  = mLogQueue.front();
		mLogQueue.pop_front();
		mMutex_logQueue.unlock();

		if(mStartTime < entry->timestamp)
			line = String()+Time::calcDiffMS(mStartTime,entry->timestamp)+"\t";
		else
			line = String()+Time::calcDiffMS(entry->timestamp,mStartTime)+"\t";
		line = line+entry->id+"\t";
		line = line+entry->str;

		line = line+"\n";
		if(mLogStream != NULL)
			mLogStream->write((tbyte*)line.c_str(), line.length());

		mMutex_logQueue.lock();
		size = mLogQueue.size();
		mMutex_logQueue.unlock();
	}
	
	if(mLogStream != NULL)
	{
		mLogStream->close();
		mLogStream = NULL;
	}

	mDone = true;
}

//void QuadLogger::addLine(const String &str, LogFlags type)
void QuadLogger::addEntry(const LogID &id, const toadlet::egg::String &str, LogFlags type)
{
	addEntry(Time(), id, str, type);
}

void QuadLogger::addEntry(const Time &t, const LogID &id, const toadlet::egg::String &str, LogFlags type)
{
	mMutex_addLine.lock();
	if( (mTypeMask & type) && mRunning)
	{
		shared_ptr<LogEntry> entry(new LogEntry());
		entry->timestamp.setTime(t);
		entry->id = id;
		entry->str = str;

		// Insert such that we maintain a sorted list
		mMutex_logQueue.lock();
//		list<shared_ptr<LogEntry>>::reverse_iterator insertIter;
//		insertIter = lower_bound(mLogQueue.rbegin(), mLogQueue.rend(), entry,
//				[&](const shared_ptr<LogEntry> &e1, const shared_ptr<LogEntry> &e2){return e1->timestamp >= e2->timestamp;});
//		mLogQueue.insert(insertIter.base(), entry);
		mLogQueue.push_back(entry);
		mMutex_logQueue.unlock();
	}
	mMutex_addLine.unlock();
}

void QuadLogger::generateMatlabHeader()
{
	FileStream::ptr logStream = FileStream::ptr(new FileStream(mDir+"/log_ids.m", FileStream::Open_BIT_WRITE));

	if(logStream != NULL)
	{
		String str;
		str = String()+"LOG_ID_ACCEL="+LOG_ID_ACCEL+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_GYRO="+LOG_ID_GYRO+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MAGNOMETER="+LOG_ID_MAGNOMETER+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_PRESSURE="+LOG_ID_PRESSURE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IMAGE="+LOG_ID_IMAGE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_PHONE_TEMP="+LOG_ID_PHONE_TEMP+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CPU_USAGE="+LOG_ID_CPU_USAGE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CPU_FREQ="+LOG_ID_CPU_FREQ+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TIME_SYNC="+LOG_ID_TIME_SYNC+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_GYRO_BIAS="+LOG_ID_GYRO_BIAS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_SET_YAW_ZERO="+LOG_ID_SET_YAW_ZERO+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_ANG_RESET="+LOG_ID_OBSV_ANG_RESET+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_ANG_GAINS_UPDATED="+LOG_ID_OBSV_ANG_GAINS_UPDATED+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OPTIC_FLOW="+LOG_ID_OPTIC_FLOW+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS="+LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OPTIC_FLOW_LS="+LOG_ID_OPTIC_FLOW_LS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_TRANS_ATT_BIAS="+LOG_ID_OBSV_TRANS_ATT_BIAS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_TRANS_FORCE_GAIN="+LOG_ID_OBSV_TRANS_FORCE_GAIN+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_BAROMETER_HEIGHT="+LOG_ID_BAROMETER_HEIGHT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MOTOR_CMDS="+LOG_ID_MOTOR_CMDS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_DES_ATT="+LOG_ID_DES_ATT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CUR_ATT="+LOG_ID_CUR_ATT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_DES_TRANS_STATE="+LOG_ID_DES_TRANS_STATE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CUR_TRANS_STATE="+LOG_ID_CUR_TRANS_STATE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IMG_PROC_TIME_TARGET_FIND="+LOG_ID_IMG_PROC_TIME_TARGET_FIND+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IBVS_ENABLED="+LOG_ID_IBVS_ENABLED+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IBVS_DISABLED="+LOG_ID_IBVS_DISABLED+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_RECEIVE_VICON="+LOG_ID_RECEIVE_VICON+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_KALMAN_ERR_COV="+LOG_ID_KALMAN_ERR_COV+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_ANG_INNOVATION="+LOG_ID_OBSV_ANG_INNOVATION+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IMG_TARGET_POINTS="+LOG_ID_IMG_TARGET_POINTS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CAMERA_POS="+LOG_ID_CAMERA_POS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_TRANS_PROC_TIME="+LOG_ID_OBSV_TRANS_PROC_TIME+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_FEATURE_FIND_TIME="+LOG_ID_FEATURE_FIND_TIME+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_NUM_FEATURE_POINTS="+LOG_ID_NUM_FEATURE_POINTS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_FAST_THRESHOLD="+LOG_ID_FAST_THRESHOLD+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MAP_VEL_CALC_TIME="+LOG_ID_MAP_VEL_CALC_TIME+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OPTIC_FLOW_VELOCITY_DELAY="+LOG_ID_OPTIC_FLOW_VELOCITY_DELAY+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MAP_VEL="+LOG_ID_MAP_VEL+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MAP_HEIGHT="+LOG_ID_MAP_HEIGHT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MAP_NUM_MATCHES="+LOG_ID_MAP_NUM_MATCHES+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MAP_PEAK_PSD_VALUE="+LOG_ID_MAP_PEAK_PSD_VALUE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TARGET_FIND_PROC_TIME="+LOG_ID_TARGET_FIND_PROC_TIME+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TARGET_FIND_CENTERS="+LOG_ID_TARGET_FIND_CENTERS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TARGET_FIND_AREAS="+LOG_ID_TARGET_FIND_AREAS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TARGET_ESTIMATED_POS="+LOG_ID_TARGET_ESTIMATED_POS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TARGET_ACQUIRED="+LOG_ID_TARGET_ACQUIRED+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TARGET_LOST="+LOG_ID_TARGET_LOST+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_ACCEL_CMD="+LOG_ID_ACCEL_CMD+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_VEL_CMD="+LOG_ID_VEL_CMD+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_SONAR_HEIGHT="+LOG_ID_SONAR_HEIGHT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MOTOR_PLANE_BIAS="+LOG_ID_MOTOR_PLANE_BIAS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IMAGE_OFFSET="+LOG_ID_IMAGE_OFFSET+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TORQUE_CMD="+LOG_ID_TORQUE_CMD+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_REF_ATTITUDE_SYSTEM_STATE="+LOG_ID_REF_ATTITUDE_SYSTEM_STATE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_VISION_INNOVATION="+LOG_ID_VISION_INNOVATION+";\n"; logStream->write((tbyte*)str.c_str(),str.length());

		logStream->close();
	}
}


}
}
