#include <jni.h>
#include <string>
#include <vector>

#include "toadlet/egg.h"

#include "../cpp/Rover.h"
#include <opencv2/core/core.hpp>
#include "../cpp/TNT/tnt.h"

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "ADKTest", __VA_ARGS__))

static ICSL::Quadrotor::Rover *rover = NULL;

extern "C" {
JNIEXPORT void JNICALL Java_com_icsl_Rover_RoverService_onJNIStart(JNIEnv* env, jobject thiz)
{
	if(rover == NULL)
		rover = new ICSL::Quadrotor::Rover();
	else
	{
		rover->shutdown();
		delete rover;
		rover = NULL;
		rover = new ICSL::Quadrotor::Rover();
	}

	rover->initialize();
}

JNIEXPORT void JNICALL Java_com_icsl_Rover_RoverService_setLogDir(JNIEnv* env, jobject thiz, jstring jdir)
{
	if(rover == NULL)
		return;

	const char *str = env->GetStringUTFChars(jdir, NULL);
	toadlet::egg::String dir(str);
	rover->setLogDir(dir);
	env->ReleaseStringUTFChars(jdir, str);
}

JNIEXPORT void JNICALL Java_com_icsl_Rover_RoverService_startLogging(JNIEnv* env, jobject thiz)
{
	if(rover == NULL)
		return;

	rover->startLogging();
}

JNIEXPORT jboolean JNICALL Java_com_icsl_Rover_RoverService_getImage(JNIEnv* env, jobject thiz, jlong addr)
{
	if(rover == NULL)
		return false;

	rover->copyImageData((cv::Mat*)addr);
	return true;
}

JNIEXPORT void JNICALL Java_com_icsl_Rover_RoverService_onNewSonarReading(JNIEnv* env, jobject thiz, jint jval, jlong jtimestamp)
{
	if(rover == NULL)
		return;

	rover->onNewSonarReading(jval, jtimestamp);
}

JNIEXPORT void JNICALL Java_com_icsl_Rover_RoverService_passNewImage(JNIEnv* env, jobject thiz, jlong imgAddr, jlong timestampNS)
{
	if(rover == NULL)
		return;

	rover->passNewImage((cv::Mat*)imgAddr, (int64)timestampNS);
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_Rover_RoverService_getGyroValue(JNIEnv* env, jobject thiz, jlong addr)
{
	if(rover == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> gyro = rover->getGyroValue();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)gyro[0][0];
	elem[1] = (jfloat)gyro[1][0];
	elem[2] = (jfloat)gyro[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_Rover_RoverService_getAccelValue(JNIEnv* env, jobject thiz, jlong addr)
{
	if(rover == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> accel= rover->getAccelValue();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)accel[0][0];
	elem[1] = (jfloat)accel[1][0];
	elem[2] = (jfloat)accel[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_Rover_RoverService_getMagValue(JNIEnv* env, jobject thiz, jlong addr)
{
	if(rover == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> mag = rover->getMagValue();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)mag[0][0];
	elem[1] = (jfloat)mag[1][0];
	elem[2] = (jfloat)mag[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_Rover_RoverService_getAttitude(JNIEnv* env, jobject thiz, jlong addr)
{
	if(rover == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> att= rover->getAttitude();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)att[0][0];
	elem[1] = (jfloat)att[1][0];
	elem[2] = (jfloat)att[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT jint JNICALL Java_com_icsl_Rover_RoverService_getImageProcTimeMS(JNIEnv* env, jobject thiz)
{
	if(rover == NULL)
		return -1;

	return rover->getImageProcTimeMS();
}

JNIEXPORT bool JNICALL Java_com_icsl_Rover_RoverService_pcIsConnected(JNIEnv* env, jobject thiz)
{
	if(rover == NULL)
		return false;

	return rover->pcIsConnected();
}

JNIEXPORT void JNICALL Java_com_icsl_Rover_RoverService_onJNIStop(JNIEnv* env, jobject thiz)
{
	if(rover != NULL)
	{
		rover->shutdown();
		delete rover;
		rover = NULL;
	}
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void *reserved)
{
	if(rover != NULL)
	{
		rover->shutdown();
		delete rover;
		rover = NULL;
	}
}

}
