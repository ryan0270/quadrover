#ifndef QUADLOGGER
#define QUADLOGGER
#include <list>
#include <sstream>
#include <memory>
#include <thread>
#include <mutex>

#include "android/sensor.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "mxml.h"

#include <cmath>

#include "TNT/tnt.h"

#include "Data.h"
#include "Common.h"
#include "Time.h"
#include "TNT_Utils.h"

#include <toadlet/egg.h>

namespace ICSL {
namespace Quadrotor {

enum LogID
{
	LOG_ID_UNKNOWN=0,
	LOG_ID_ACCEL = 1,
	LOG_ID_GYRO = 2,
	LOG_ID_MAGNOMETER = 3,
	LOG_ID_PRESSURE = 4,
	LOG_ID_IMAGE = 10,
	LOG_ID_SONAR_HEIGHT=20,
	LOG_ID_PHONE_TEMP = 500,
	LOG_ID_CPU_USAGE = -2000,
	LOG_ID_CPU_FREQ = -2001,
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
	LOG_ID_ACCEL_CMD = -1020,
	LOG_ID_VEL_CMD = -1021,
//	LOG_ID_IMG_PROC_TIME_FEATURE_MATCH = -600,
	LOG_ID_IMG_PROC_TIME_TARGET_FIND = -601,
	LOG_ID_IBVS_ENABLED = -605,
	LOG_ID_IBVS_DISABLED = -606,
	LOG_ID_TARGET_ACQUIRED = -607,
	LOG_ID_TARGET_LOST = -608,
	LOG_ID_RECEIVE_VICON = 700,
	LOG_ID_CAMERA_POS = 800,
	LOG_ID_KALMAN_ERR_COV = -720,
	LOG_ID_IMG_TARGET_POINTS = 1310,
	LOG_ID_OBSV_TRANS_PROC_TIME = 10000,
	LOG_ID_FEATURE_FIND_TIME = 5000,
	LOG_ID_NUM_FEATURE_POINTS,
	LOG_ID_FAST_THRESHOLD,
	LOG_ID_MAP_VEL = 5010,
	LOG_ID_MAP_HEIGHT,
	LOG_ID_MAP_NUM_MATCHES,
	LOG_ID_MAP_PEAK_PSD_VALUE,
	LOG_ID_MAP_VEL_CALC_TIME,
	LOG_ID_OPTIC_FLOW_VELOCITY_DELAY = 5020,
	LOG_ID_REGION_FIND_TIME = 6000,
	LOG_ID_NUM_REGIONS,
	LOG_ID_TARGET_FIND_PROC_TIME = 6000,
	LOG_ID_TARGET_FIND_CENTERS,
	LOG_ID_TARGET_FIND_AREAS,
	LOG_ID_TARGET_ESTIMATED_POS=6010,
	LOG_ID_MOTOR_PLANE_BIAS=7000,
	LOG_ID_IMAGE_OFFSET=8000,
	LOG_ID_TORQUE_CMD=10000,
	LOG_ID_REF_ATTITUDE_SYSTEM_STATE=11000,
	LOG_ID_VISION_INNOVATION=12000,
	LOG_ID_OBJECT_TRACKING_STATS=13000,
	LOG_ID_USE_VICON_YAW=14000,
	LOG_ID_USE_VICON_XY,
};

class LogEntry
{
	public:
	String str;
	Time timestamp;
	LogID id;
};

class QuadLogger
{
	public:
		explicit QuadLogger();
		virtual ~QuadLogger();

		toadlet::egg::String getDir(){return mDir;}
		toadlet::egg::String getFilename(){return mFilename;}
		toadlet::egg::String getFullPath(){return mDir+"/"+mFilename;}
		uint32_t getMask(){return mTypeMask;}

		void setDir(toadlet::egg::String dir){mDir = dir;}
		void setFilename(toadlet::egg::String name){mFilename = name;}
		void setMask(uint32_t mask){mTypeMask = mask;}
		void addEntry(const Time &t, const LogID &id, const toadlet::egg::String &str, LogFlags type);
		void addEntry(const LogID &id, LogFlags type);
		void addEntry(const LogID &id, const toadlet::egg::String &str, LogFlags type);
		void addEntry(const LogID &id, int data, const Time &t, LogFlags type);
		void addEntry(const LogID &id, double data, LogFlags type);
		void addEntry(const LogID &id, double data, const Time &t, LogFlags type);
		void addEntry(const LogID &id, const TNT::Array2D<double> &data, LogFlags type);
		void addEntry(const LogID &id, const shared_ptr<DataVector<double>> &data, LogFlags type);
		void addEntry(const LogID &id, const SO3 &data, const TNT::Array2D<double> &velData, LogFlags type);
		void addEntry(const LogID &id, const shared_ptr<SO3Data<double>> &data, const shared_ptr<DataVector<double>> &velData, LogFlags type);
		void addEntry(const LogID &id, const toadlet::egg::Collection<double> &data, LogFlags type);
		void addEntry(const LogID &id, const toadlet::egg::Collection<float> &data, LogFlags type);
		void addEntry(const LogID &id, const vector<double> &data, LogFlags type);
		void addEntry(const LogID &id, const vector<float> &data, LogFlags type);
		void addEntry(const LogID &id, const cv::Point2f &data, LogFlags type);
		void addEntry(const LogID &id, const cv::Point2f &data, const Time &t, LogFlags type);
		void addEntry(const LogID &id, const ASensorEvent &event, const Time &t, LogFlags type);
		void addEntry(const LogID &id, const shared_ptr<DataImage> &data, LogFlags type);


		void pause(){mPaused = true;}
		void resume(){mPaused = false;}

		void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
		void start(){ thread th(&QuadLogger::run, this); th.detach(); }
		void run();
		void shutdown();
		void clearLog(){shutdown(); start();}

		void setStartTime(Time time){mStartTime.setTime(time);}

	protected:
		toadlet::egg::String mDir, mFilename;
		uint32_t mTypeMask;
		toadlet::egg::FileStream::ptr mLogStream;
		std::mutex mMutex_file, mMutex_logQueue, mMutex_addLine;
		Time mStartTime;

		bool mPaused;

		void generateMatlabHeader();

		int mThreadPriority, mScheduler;

		list<shared_ptr<LogEntry>> mLogQueue;

		bool mRunning, mDone;
};

}
}
#endif
