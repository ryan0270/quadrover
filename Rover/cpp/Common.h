#ifndef QUAD_CONSTANTS
#define QUAD_CONSTANTS

#include <toadlet/toadlet.h>


namespace ICSL {
namespace Quadrotor {
using toadlet::uint64;
enum LogFlags
{
	STATE		= 1 << 0,
	STATE_DES	= 1 << 1,
	MOTORS 		= 1 << 2,
	PC_UPDATES	= 1 << 3,
	OBSV_UPDATE	= 1 << 4,
	OBSV_BIAS	= 1 << 5,
	MAGNOMETER	= 1 << 6,
	ACCEL		= 1 << 7,
	GYRO		= 1 << 8,
	CAM_RESULTS	= 1 << 9,
	CAM_IMAGES	= 1 << 10,
	PRESSURE	= 1 << 11,
	PHONE_TEMP	= 1 << 12,
};

enum LogIDs
{
	LOG_ID_PHONE_TEMP = 500,
};

enum SensorDataType
{
	SENSOR_DATA_TYPE_UNDEFINED=0,
	SENSOR_DATA_TYPE_GYRO,
	SENSOR_DATA_TYPE_ACCEL,
	SENSOR_DATA_TYPE_MAG,
	SENSOR_DATA_TYPE_PRESSURE,
	SENSOR_DATA_TYPE_BATTERY,
	SENSOR_DATA_TYPE_IMAGE,
};

enum CommID
{
	COMM_MOTOR_ON = 1,
	COMM_MOTOR_OFF,
	COMM_MOTOR_TRIM,
	COMM_OBSV_RESET,
	COMM_ATT_OBSV_GAIN,
	COMM_TIME_SYNC,
	COMM_LOG_FILE_REQUEST,
	COMM_LOG_FILE_DATA,
	COMM_IMG_DATA,
	COMM_ARDUINO_STATUS,
	COMM_USE_MOTORS,
	COMM_STATE_PHONE,
	COMM_DESIRED_STATE,
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
	COMM_DES_STATE,
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
