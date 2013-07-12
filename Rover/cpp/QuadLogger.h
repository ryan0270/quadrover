#ifndef QUADLOGGER
#define QUADLOGGER
#include <list>
#include <queue>
#include <sstream>
#include <memory>
#include <thread>

#include "mxml.h"

#include <cmath>
#include <toadlet/egg.h>
#include "Common.h"
#include "Time.h"
#include "Data.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ICSL {
namespace Quadrotor {
using namespace std;

enum LogIDs
{
	LOG_ID_ACCEL = 1,
	LOG_ID_GYRO = 2,
	LOG_ID_MAGNOMETER = 3,
	LOG_ID_PRESSURE = 4,
	LOG_ID_IMAGE = 10,
	LOG_ID_PHONE_TEMP = 500,
	LOG_ID_CPU_USAGE = -2000,
	LOG_ID_TIME_SYNC = -500,
	LOG_ID_GYRO_BIAS = -1003,
	LOG_ID_OBSV_ANG_INNOVATION = -1004,
	LOG_ID_SET_YAW_ZERO = -805,
	LOG_ID_OBSV_ANG_RESET = -200,
	LOG_ID_OBSV_ANG_GAINS_UPDATED = -210,
	LOG_ID_OPTIC_FLOW = 12345,
	LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS = 12346,
	LOG_ID_OPTIC_FLOW_LS = 123457,
	LOG_ID_OBSV_TRANS_ATT_BIAS = -710,
	LOG_ID_OBSV_TRANS_FORCE_GAIN = -711,
	LOG_ID_BAROMETER_HEIGHT = 1234,
	LOG_ID_MOTOR_CMDS = -1000,
	LOG_ID_DES_ATT = -1001,
	LOG_ID_CUR_ATT = -1002,
	LOG_ID_DES_TRANS_STATE = -1011,
	LOG_ID_CUR_TRANS_STATE = -1012,
//	LOG_ID_IMG_PROC_TIME_FEATURE_MATCH = -600,
	LOG_ID_IMG_PROC_TIME_TARGET_FIND = -601,
	LOG_ID_IBVS_ENABLED = -605,
	LOG_ID_IBVS_DISABLED = -606,
	LOG_ID_RECEIVE_VICON = 700,
	LOG_ID_CAMERA_POS = 800,
	LOG_ID_KALMAN_ERR_COV = -720,
	LOG_ID_IMG_TARGET_POINTS = 1310,
	LOG_ID_OBSV_TRANS_PROC_TIME = 10000,
	LOG_ID_FEATURE_FIND_TIME = 5000,
	LOG_ID_NUM_FEATURE_POINTS = 5001,
	LOG_ID_MAP_VEL_CALC_TIME = 5010,
	LOG_ID_OPTIC_FLOW_VELOCITY_DELAY = 5020,
};

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
		void addLine(String const &str, LogFlags type);

		void pause(){mPaused = true;}
		void resume(){mPaused = false;}

		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
		void start(){ thread th(&QuadLogger::run, this); th.detach(); }
		void run();
		void shutdown();
		void clearLog(){shutdown(); start();}

		void saveImageBuffer(list<shared_ptr<DataImage> > const &dataBuffer,
							 list<shared_ptr<ImageMatchData> > const &matchDataBuffer);

		void saveFeatureMatchBuffer(list<vector<vector<cv::Point2f> > > const &matchBuffer, 
									list<Time> const &timeBuffer,
									list<double> const &dtBuffer,
									list<TNT::Array2D<double> > const &attPrevBuffer,
									list<TNT::Array2D<double> > const &attCurBuffer);
		void setStartTime(Time time){mStartTime.setTime(time);}

	protected:
		String mDir, mFilename;
		uint32 mTypeMask;
		FileStream::ptr mLogStream;
		Mutex mMutex_file, mMutex_logQueue, mMutex_addLine;
		Time mStartTime;

		bool mPaused;

		void generateMatlabHeader();

		int mThreadPriority, mScheduler;

		queue<String> mLogQueue;

		bool mRunning, mDone;
};

}
}
#endif
