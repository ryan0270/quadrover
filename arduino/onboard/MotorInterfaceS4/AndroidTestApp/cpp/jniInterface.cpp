#include <jni.h>
#include <unistd.h>
#include <thread>
#include <android/log.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "TestActivity", __VA_ARGS__))


bool isShutdown = true;
bool doShutdown = true;
bool haveNewData = false;
float newFloat = 0;
JavaVM *myJVM = NULL;
jobject obj = 0;
jclass cls = 0;
jmethodID mid_sendFloat = 0;
jmethodID mid_sendInt = 0;

enum
{
	COMM_ADK_PREFIX = 0x0000FF00,
	COMM_ADK_SUFFIX = 0x0000FF00-1,
	COMM_ADK_MOTOR_N = 0xFF-2,
	COMM_ADK_MOTOR_E = 0xFF-3,
	COMM_ADK_MOTOR_S = 0xFF-4,
	COMM_ADK_MOTOR_W = 0xFF-5,
};

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
	jclass tmpcls = env->FindClass("com/icsl/serialtest/TestActivity");
	cls = (jclass)env->NewGlobalRef(tmpcls); // keep cls and, hence, mid_sendFloat valid even after this function exits
	mid_sendFloat = env->GetMethodID(cls, "sendFloat","(FI)I");
	if(mid_sendFloat == 0)
	{
		LOGI("Couldn't find java method sendFloat");
		return -1;
	}

	mid_sendInt= env->GetMethodID(cls, "sendInt","(II)I");
	if(mid_sendInt== 0)
	{
		LOGI("Couldn't find java method sendInt");
		return -1;
	}
	return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL Java_com_icsl_serialtest_TestActivity_doChad(JNIEnv* env, jobject thiz, jfloat val)
{
	newFloat = val;
	haveNewData = true;
}

JNIEXPORT void JNICALL Java_com_icsl_serialtest_TestActivity_shutdown(JNIEnv* env, jobject thiz)
{
	doShutdown = true;
	env->DeleteGlobalRef(cls);
	env->DeleteGlobalRef(obj);

	while(!isShutdown)
		usleep(10e3);
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
		if(haveNewData)
		{
			float val = newFloat;
			haveNewData = false;

			if(val > 0.9)
				val = 0;
			int chad = COMM_ADK_PREFIX;
			float bob = COMM_ADK_SUFFIX;
			env->CallBooleanMethod(obj, mid_sendInt, chad, timeoutMS);
			usleep(1e6);
			env->CallBooleanMethod(obj, mid_sendFloat, val, timeoutMS);
//			env->CallBooleanMethod(obj, mid_sendFloat, bob, timeoutMS);
		}

		usleep(10e3);
    }

	myJVM->DetachCurrentThread();
	isShutdown = true;
}

JNIEXPORT void JNICALL Java_com_icsl_serialtest_TestActivity_jniInit(JNIEnv* env, jobject thiz)
{
	obj = env->NewGlobalRef(thiz);

	std::thread th(&run);
	th.detach();
}

}

