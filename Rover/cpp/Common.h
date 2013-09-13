#ifndef QUAD_COMMON
#define QUAD_COMMON

#include <toadlet/egg.h>

namespace ICSL {
namespace Quadrotor {
enum LogFlags
{
	LOG_FLAG_OTHER			= 0,
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
	COMM_SET_DESIRED_POS,
	COMM_FEATURE_FIND_QUALITY_THRESHOLD,
	COMM_FEATURE_FIND_SEPARATION_DISTANCE,
	COMM_FEATURE_FIND_FAST_THRESHOLD,
	COMM_FEATURE_FIND_POINT_COUNT_TARGET,
	COMM_FEATURE_FIND_FAST_ADAPT_RATE,
	COMM_VELOCITY_ESTIMATION_VISION_MEASUREMENT_COV,
	COMM_VELOCITY_ESTIMATION_PROB_NO_CORR,
	COMM_SEND_TRANS_CNTL_SYSTEM,
	COMM_ACCELEROMETER_BIAS,
};

class Packet
{
	public:
	toadlet::int32 size, type;
	toadlet::uint64 time;
	toadlet::egg::Collection<toadlet::int32> dataInt32;
	toadlet::egg::Collection<float> dataFloat;
	toadlet::egg::Collection<bool> dataBool;

	/*!
	 * \param serial Object to store the serialized packet information in
	 */
	void serialize(toadlet::egg::Collection<toadlet::tbyte> &serial)
	{
		size = sizeof(toadlet::int32); // size
		size += sizeof(toadlet::int32); // type
		size += sizeof(toadlet::uint64); //time
		size += sizeof(toadlet::int32); // number of toadlet::int32
		size += dataInt32.size()*sizeof(toadlet::int32); // ints
		size += sizeof(toadlet::int32); // number of floats
		size += dataFloat.size()*sizeof(float); // floats
		size += sizeof(toadlet::int32); // number of bools
		size += dataBool.size()*sizeof(bool); // bools
		serial.resize(size);
		toadlet::tbyte *p = &serial.front();

		memcpy(p, &size, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		memcpy(p, &type, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		memcpy(p, &time, sizeof(toadlet::uint64)); p += sizeof(toadlet::uint64);

		toadlet::int32 nInt32 = dataInt32.size();
		memcpy(p, &nInt32, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		memcpy(p, &dataInt32.front(), dataInt32.size()*sizeof(toadlet::int32)); p += dataInt32.size()*sizeof(toadlet::int32);

		toadlet::int32 nFloat = dataFloat.size();
		memcpy(p, &nFloat, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		memcpy(p, &dataFloat.front(), dataFloat.size()*sizeof(float)); p += dataFloat.size()*sizeof(float);

		toadlet::int32 nBool= dataBool.size();
		memcpy(p, &nBool, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		memcpy(p, &dataBool.front(), dataBool.size()*sizeof(bool)); p += dataBool.size()*sizeof(bool);

//		{
//			String s = String()+"Serialized " + size + " bytes";
//			Logger::alert(s);
//		}
	}

	void deserialize(const toadlet::egg::Collection<toadlet::tbyte> &serial)
	{
		const toadlet::tbyte *p = &serial.front();
		memcpy(&size, p, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		memcpy(&type, p, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		memcpy(&time, p, sizeof(toadlet::uint64)); p += sizeof(toadlet::uint64);

		toadlet::int32 nInt32;
		memcpy(&nInt32, p, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		dataInt32.resize(nInt32);
		memcpy(&dataInt32.front(), p, dataInt32.size()*sizeof(toadlet::int32)); p += dataInt32.size()*sizeof(toadlet::int32);

		toadlet::int32 nFloat;
		memcpy(&nFloat, p, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		dataFloat.resize(nFloat);
		memcpy(&dataFloat.front(), p, dataFloat.size()*sizeof(float)); p += dataFloat.size()*sizeof(float);

		toadlet::int32 nBool;
		memcpy(&nBool, p, sizeof(toadlet::int32)); p += sizeof(toadlet::int32);
		dataBool.resize(nBool);
		memcpy(&dataBool.front(), p, dataBool.size()*sizeof(bool)); p += dataBool.size()*sizeof(bool);
	}
};

} // ICSL
} // Quadrotor

#endif
