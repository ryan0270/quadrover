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
};

enum CommID
{
	COMM_MOTOR_ON = 1,
	COMM_MOTOR_OFF,
//	COMM_RATE_CMD,
	COMM_CNTL_GAIN_PID,
	COMM_MOTOR_TRIM,
	COMM_OBSV_RESET,
	COMM_OBSV_GAIN,
	COMM_TIME_SYNC,
	COMM_LOG_TRANSFER,
	COMM_SEND_CNTL_SYSTEM,
	COMM_CNTL_SYS_GAINS,
//	COMM_CNTL_TYPE,	
	COMM_ARDUINO_STATUS,
	COMM_USE_MOTORS,
	COMM_STATE_PHONE,
	COMM_DESIRED_STATE,
//	COMM_IMAGE_STATE,
//	COMM_DESIRED_IMAGE_STATE,
//	COMM_DESIRED_IMAGE_MOMENT,
	COMM_GYRO,
	COMM_ACCEL,
	COMM_MAGNOMETER,
	COMM_OBSV_BIAS,
	COMM_MOTOR_VAL,
//	COMM_MOTOR_VAL_IBVS,
	COMM_INT_MEM_POS,
	COMM_INT_MEM_TORQUE,
//	COMM_CNTL_CALC_TIME,
//	COMM_IMGPROC_BOX_COLOR_MIN,
//	COMM_IMGPROC_BOX_COLOR_MAX,
//	COMM_IMGPROC_SAT_MIN,
//	COMM_IMGPROC_SAT_MAX,
//	COMM_IMGPROC_VAL_MIN,
//	COMM_IMGPROC_VAL_MAX,
//	COMM_IMGPROC_CIRC_MIN,
//	COMM_IMGPROC_CIRC_MAX,
//	COMM_IMGPROC_CONV_MIN,
//	COMM_IMGPROC_CONV_MAX,
//	COMM_IMGPROC_AREA_MIN,
//	COMM_IMGPROC_AREA_MAX,
	COMM_IMGPROC_TIME_US,
	COMM_IMGVIEW_TYPE,
	COMM_LOG_MASK,
	COMM_CLEAR_LOG,
	COMM_STATE_VICON,
	COMM_MOTOR_FORCE_SCALING,
	COMM_MOTOR_TORQUE_SCALING,
	COMM_MASS,
	COMM_IBVS_GAINS,
	COMM_USE_IBVS,
	COMM_SET_YAW_ZERO,
	COMM_POS_CNTL_GAINS,
	COMM_DES_STATE,
	COMM_ATT_BIAS,
	COMM_ATT_BIAS_GAIN,
	COMM_FORCE_SCALING_GAIN,
	COMM_ATT_GAINS,
	COMM_KALMANFILTER_POS_MEAS_STD,
	COMM_KALMANFILTER_VEL_MEAS_STD,
};

enum Controllers
{	CNTL_ANGLE_RATE_PID = 0, CNTL_ANGLE_RATE_SYS,
	CNTL_TRANSLATION_PID = 0, CNTL_TRANSLATION_SYS,};

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
