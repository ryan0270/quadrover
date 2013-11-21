#include <jni.h>
#include <unistd.h>
#include <thread>
#include <android/log.h>

#include "Time.h"
#include "MotorInterface.h"

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "AdbTest", __VA_ARGS__))

bool isShutdown = true;
bool doShutdown = true;
JavaVM *myJVM = NULL;
jobject obj = 0;
jclass cls = 0;
jmethodID mid_updateDisplay = 0;

ICSL::Time startTime;
ICSL::Quadrotor::MotorInterface *motorInterface = NULL;

extern "C" {
jint JNI_OnLoad(JavaVM *jvm, void *reserved)
{
	myJVM = jvm;
	JNIEnv* env;
	int result = myJVM->AttachCurrentThread(&env, NULL);
	if(myJVM->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
	{
		LOGI("Error getting environ");
		return -1;
	}
	jclass tmpcls = env->FindClass("com/icsl/adbtest/AdbTest");
	cls = (jclass)env->NewGlobalRef(tmpcls); // keep cls and, hence, mid_updateDisplay valid even after this function exits
	mid_updateDisplay = env->GetMethodID(cls, "updateDisplay","(F)V");
	if(mid_updateDisplay == 0)
	{
		LOGI("Couldn't find java method updateDisplay");
		return -1;
	}
	else
		LOGI("Found java function updateDisplay");

	return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL Java_com_icsl_adbtest_AdbTest_jniShutdown(JNIEnv* env, jobject thiz)
{
	doShutdown = true;

	while(!isShutdown)
		usleep(10e3);

	if(motorInterface != NULL)
	{
		motorInterface->shutdown();
		delete motorInterface;
	}
	env->DeleteGlobalRef(cls);
	env->DeleteGlobalRef(obj);
}

void run()
{
	isShutdown = false;
	doShutdown = false;
	jint timeoutMS = 500;
	JNIEnv* env;
	int result = myJVM->AttachCurrentThread(&env, NULL);
	if(myJVM->GetEnv(reinterpret_cast<void**>(&env), JNI_VERSION_1_6) != JNI_OK)
	{
		LOGI("Error getting environ");
		return;
	}
    while( !doShutdown )
    {
//		float time = startTime.getElapsedTimeNS()/1.0e9;
		float height = 0;
		if(motorInterface != NULL)
			height = motorInterface->getLastSonarHeight();
		env->CallBooleanMethod(obj, mid_updateDisplay, height);
		usleep(1e6);
    }
	myJVM->DetachCurrentThread();

	LOGI("jni runner dead");

	isShutdown = true;
}

JNIEXPORT void JNICALL Java_com_icsl_adbtest_AdbTest_jniInit(JNIEnv* env, jobject thiz)
{
	obj = env->NewGlobalRef(thiz);

	startTime.setTime();
	motorInterface = new ICSL::Quadrotor::MotorInterface();
	motorInterface->initialize();
	motorInterface->start();

	std::thread th(&run);
	th.detach();
}

}

