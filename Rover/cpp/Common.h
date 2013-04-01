#ifndef QUAD_CONSTANTS
#define QUAD_CONSTANTS

#include <toadlet/egg.h>


namespace ICSL {
namespace Quadrotor {
using toadlet::uint64;
enum LogFlags
{
	LOG_FLAG_STATE			= 1 << 0,
	LOG_FLAG_STATE_DES		= 1 << 1,
	LOG_FLAG_MOTORS 		= 1 << 2,
	LOG_FLAG_PC_UPDATES		= 1 << 3,
	LOG_FLAG_OBSV_UPDATE	= 1 << 4,
	LOG_FLAG_OBSV_BIAS		= 1 << 5,
	LOG_FLAG_MAGNOMETER		= 1 << 6,
	LOG_FLAG_ACCEL			= 1 << 7,
	LOG_FLAG_GYRO			= 1 << 8,
	LOG_FLAG_CAM_RESULTS	= 1 << 9,
	LOG_FLAG_CAM_IMAGES		= 1 << 10,
	LOG_FLAG_PRESSURE		= 1 << 11,
	LOG_FLAG_PHONE_TEMP		= 1 << 12,
};

enum LogIDs
{
	LOG_ID_ACCEL = 1,
	LOG_ID_GYRO = 2,
	LOG_ID_MAGNOMETER = 3,
	LOG_ID_PRESSURE = 4,
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
	LOG_ID_IMG_PROC_TIME_FEATURE_MATCH = -600,
	LOG_ID_IMG_PROC_TIME_TARGET_FIND = -601,
	LOG_ID_IBVS_ENABLED = -605,
	LOG_ID_IBVS_DISABLED = -606,
	LOG_ID_RECEIVE_VICON = 700,
	LOG_ID_CAMERA_POS = 800,
	LOG_ID_KALMAN_ERR_COV = -720,
	LOG_ID_NUM_FEATURE_POINTS = 1300,
	LOG_ID_IMG_TARGET_POINTS = 1310,
	LOG_ID_OBSV_TRANS_PROC_TIME = 10000,
};

enum SensorDataType
{
	SENSOR_DATA_TYPE_UNDEFINED=0,
	SENSOR_DATA_TYPE_GYRO,
	SENSOR_DATA_TYPE_ACCEL,
	SENSOR_DATA_TYPE_MAG,
	SENSOR_DATA_TYPE_PRESSURE,
	SENSOR_DATA_TYPE_IMAGE,
	SENSOR_DATA_TYPE_PHONE_TEMP,
};

enum CommID
{
	COMM_PING = 0,
	COMM_MOTOR_ON,
	COMM_MOTOR_OFF,
	COMM_MOTOR_TRIM,
	COMM_OBSV_RESET,
	COMM_ATT_OBSV_GAIN,
	COMM_TIME_SYNC,
	COMM_HOST_TIME_MS,
	COMM_LOG_FILE_REQUEST,
	COMM_LOG_FILE_DATA,
	COMM_IMG_DATA,
	COMM_ARDUINO_STATUS,
	COMM_USE_MOTORS,
	COMM_STATE_PHONE,
	COMM_DESIRED_STATE,
	COMM_SET_DESIRED_STATE,
	COMM_GYRO,
	COMM_ACCEL,
	COMM_MAGNOMETER,
	COMM_OBSV_BIAS,
	COMM_MOTOR_VAL,
	COMM_INT_MEM_POS,
	COMM_INT_MEM_TORQUE,
	COMM_IMGPROC_TIME_US,
	COMM_IMGVIEW_TYPE,
	COMM_LOG_MASK,
	COMM_CLEAR_LOG,
	COMM_STATE_VICON,
	COMM_MOTOR_FORCE_GAIN,
	COMM_MOTOR_TORQUE_GAIN,
	COMM_MASS,
	COMM_USE_IBVS,
	COMM_CNTL_ATT_GAINS,
	COMM_CNTL_TRANS_GAINS,
	COMM_KALMAN_ATT_BIAS,
	COMM_KALMAN_ATT_BIAS_ADAPT_RATE,
	COMM_KALMAN_FORCE_SCALING_ADAPT_RATE,
	COMM_KALMANFILTER_MEAS_VAR,
	COMM_KALMANFILTER_DYN_VAR,
	COMM_CLIENT_EXIT,
	COMM_HOST_EXIT,
	COMM_NOMINAL_MAG,
	COMM_MOTOR_ARM_LENGTH,
	COMM_IMG_BUFFER_SIZE,
	COMM_BAROMETER_ZERO_HEIGHT,
	COMM_BAROMETER_HEIGHT,
	COMM_PRESSURE,
	COMM_PHONE_TEMP,
	COMM_VISION_NUM_FEATURES,
	COMM_VISION_RATIO_THRESHOLD,
	COMM_VISION_MATCH_RADIUS,
	COMM_SET_DESIRED_POS,
};

class Packet
{
	public:
	int32 size, type;
	uint64 time;
	toadlet::egg::Collection<int32> dataInt32;
	toadlet::egg::Collection<float> dataFloat;
	toadlet::egg::Collection<bool> dataBool;

	/*!
	 * \param serial Object to store the serialized packet information in
	 */
	void serialize(Collection<tbyte> &serial)
	{
		size = sizeof(int32); // size
		size += sizeof(int32); // type
		size += sizeof(uint64); //time
		size += sizeof(int32); // number of int32
		size += dataInt32.size()*sizeof(int32); // ints
		size += sizeof(int32); // number of floats
		size += dataFloat.size()*sizeof(float); // floats
		size += sizeof(int32); // number of bools
		size += dataBool.size()*sizeof(bool); // bools
		serial.resize(size);
		tbyte *p = serial.begin();

		memcpy(p, &size, sizeof(int32)); p += sizeof(int32);
		memcpy(p, &type, sizeof(int32)); p += sizeof(int32);
		memcpy(p, &time, sizeof(uint64)); p += sizeof(uint64);

		int32 nInt32 = dataInt32.size();
		memcpy(p, &nInt32, sizeof(int32)); p += sizeof(int32);
		memcpy(p, dataInt32.begin(), dataInt32.size()*sizeof(int32)); p += dataInt32.size()*sizeof(int32);

		int32 nFloat = dataFloat.size();
		memcpy(p, &nFloat, sizeof(int32)); p += sizeof(int32);
		memcpy(p, dataFloat.begin(), dataFloat.size()*sizeof(float)); p += dataFloat.size()*sizeof(float);

		int32 nBool= dataBool.size();
		memcpy(p, &nBool, sizeof(int32)); p += sizeof(int32);
		memcpy(p, dataBool.begin(), dataBool.size()*sizeof(bool)); p += dataBool.size()*sizeof(bool);

//		{
//			String s = String()+"Serialized " + size + " bytes";
//			Logger::alert(s);
//		}
	}

	void deserialize(Collection<tbyte> const &serial)
	{
		tbyte *p = serial.begin();
		memcpy(&size, p, sizeof(int32)); p += sizeof(int32);
		memcpy(&type, p, sizeof(int32)); p += sizeof(int32);
		memcpy(&time, p, sizeof(uint64)); p += sizeof(uint64);

		int32 nInt32;
		memcpy(&nInt32, p, sizeof(int32)); p += sizeof(int32);
		dataInt32.resize(nInt32);
		memcpy(dataInt32.begin(), p, dataInt32.size()*sizeof(int32)); p += dataInt32.size()*sizeof(int32);

		int32 nFloat;
		memcpy(&nFloat, p, sizeof(int32)); p += sizeof(int32);
		dataFloat.resize(nFloat);
		memcpy(dataFloat.begin(), p, dataFloat.size()*sizeof(float)); p += dataFloat.size()*sizeof(float);

		int32 nBool;
		memcpy(&nBool, p, sizeof(int32)); p += sizeof(int32);
		dataBool.resize(nBool);
		memcpy(dataBool.begin(), p, dataBool.size()*sizeof(bool)); p += dataBool.size()*sizeof(bool);
	}
};

} // ICSL
} // Quadrotor

#endif
