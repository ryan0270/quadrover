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
	LOG_FLAG_SONAR			= 1 << 13,
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
	COMM_CNTL_IBVS_GAINS,
};

class Packet
{
	public:
	int32_t size, type;
	uint64_t time;
	toadlet::egg::Collection<int32_t> dataInt32;
	toadlet::egg::Collection<float> dataFloat;
	toadlet::egg::Collection<bool> dataBool;

	/*!
	 * \param serial Object to store the serialized packet information in
	 */
	void serialize(toadlet::egg::Collection<toadlet::tbyte> &serial)
	{
		size = sizeof(int32_t); // size
		size += sizeof(int32_t); // type
		size += sizeof(uint64_t); //time
		size += sizeof(int32_t); // number of int32
		size += dataInt32.size()*sizeof(int32_t); // ints
		size += sizeof(int32_t); // number of floats
		size += dataFloat.size()*sizeof(float); // floats
		size += sizeof(int32_t); // number of bools
		size += dataBool.size()*sizeof(bool); // bools
		serial.resize(size);
		toadlet::tbyte *p = &serial.front();

		memcpy(p, &size, sizeof(int32_t)); p += sizeof(int32_t);
		memcpy(p, &type, sizeof(int32_t)); p += sizeof(int32_t);
		memcpy(p, &time, sizeof(uint64_t)); p += sizeof(uint64_t);

		int32_t nInt32 = dataInt32.size();
		memcpy(p, &nInt32, sizeof(int32_t)); p += sizeof(int32_t);
		memcpy(p, &dataInt32.front(), dataInt32.size()*sizeof(int32_t)); p += dataInt32.size()*sizeof(int32_t);

		int32_t nFloat = dataFloat.size();
		memcpy(p, &nFloat, sizeof(int32_t)); p += sizeof(int32_t);
		memcpy(p, &dataFloat.front(), dataFloat.size()*sizeof(float)); p += dataFloat.size()*sizeof(float);

		int32_t nBool= dataBool.size();
		memcpy(p, &nBool, sizeof(int32_t)); p += sizeof(int32_t);
		memcpy(p, &dataBool.front(), dataBool.size()*sizeof(bool)); p += dataBool.size()*sizeof(bool);
	}

	void deserialize(const toadlet::egg::Collection<toadlet::tbyte> &serial)
	{
		const toadlet::tbyte *p = &serial.front();
		memcpy(&size, p, sizeof(int32_t)); p += sizeof(int32_t);
		memcpy(&type, p, sizeof(int32_t)); p += sizeof(int32_t);
		memcpy(&time, p, sizeof(uint64_t)); p += sizeof(uint64_t);

		int32_t nInt32;
		memcpy(&nInt32, p, sizeof(int32_t)); p += sizeof(int32_t);
		dataInt32.resize(nInt32);
		memcpy(&dataInt32.front(), p, dataInt32.size()*sizeof(int32_t)); p += dataInt32.size()*sizeof(int32_t);

		int32_t nFloat;
		memcpy(&nFloat, p, sizeof(int32_t)); p += sizeof(int32_t);
		dataFloat.resize(nFloat);
		memcpy(&dataFloat.front(), p, dataFloat.size()*sizeof(float)); p += dataFloat.size()*sizeof(float);

		int32_t nBool;
		memcpy(&nBool, p, sizeof(int32_t)); p += sizeof(int32_t);
		dataBool.resize(nBool);
		memcpy(&dataBool.front(), p, dataBool.size()*sizeof(bool)); p += dataBool.size()*sizeof(bool);
	}
};

} // ICSL
} // Quadrotor

#endif
