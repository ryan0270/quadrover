#include <jni.h>
#include <string.h>

#include "../cpp/Playground.h"
// #include <opencv2/core/core.hpp>
// #include "../cpp/TNT/tnt.h"

static ICSL::Quadrotor::Playground *playground = NULL;

extern "C" {

JNIEXPORT void JNICALL Java_com_icsl_Playground_Playground_onJNIStart(JNIEnv* env, jobject thiz)
{
	if(playground == NULL)
		playground = new ICSL::Quadrotor::Playground();
	else
	{
		playground->shutdown();
		delete playground;
		playground = NULL;
		playground = new ICSL::Quadrotor::Playground();
	}

	playground->initialize();
}

JNIEXPORT void JNICALL Java_com_icsl_Playground_Playground_setNumCpuCores(JNIEnv* env, jobject thiz, jint numCores)
{
	if(playground == NULL)
		return;

	playground->setNumCpuCores(numCores);
}

JNIEXPORT void JNICALL Java_com_icsl_Playground_Playground_setLogDir(JNIEnv* env, jobject thiz, jstring jdir)
{
	if(playground == NULL)
		return;

	const char *str = env->GetStringUTFChars(jdir, NULL);
	String dir(str);
	playground->setLogDir(dir);
	env->ReleaseStringUTFChars(jdir, str);
}

JNIEXPORT void JNICALL Java_com_icsl_Playground_Playground_startLogging(JNIEnv* env, jobject thiz)
{
	if(playground == NULL)
		return;

	playground->startLogging();
}

JNIEXPORT void JNICALL Java_com_icsl_Playground_Playground_getImage(JNIEnv* env, jobject thiz, jlong addr)
{
	if(playground == NULL)
		return;

	playground->copyImageData((cv::Mat*)addr);
}

JNIEXPORT jint JNICALL Java_com_icsl_Playground_Playground_getImageProcTimeMS(JNIEnv* env, jobject thiz)
{
	if(playground == NULL)
		return -1;

	return playground->getImageProcTimeMS();
}

JNIEXPORT void JNICALL Java_com_icsl_Playground_Playground_onJNIStop(JNIEnv* env, jobject thiz)
{
	if(playground != NULL)
	{
		playground->shutdown();
		delete playground;
		playground = NULL;
	}
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void *reserved)
{
	if(playground != NULL)
	{
		playground->shutdown();
		delete playground;
		playground = NULL;
	}
}

}
