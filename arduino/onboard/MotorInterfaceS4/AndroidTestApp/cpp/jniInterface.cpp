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
		LOGI("Couldn't find java class");
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
			float val = newFloat+0.5;
			haveNewData = false;
			env->CallBooleanMethod(obj, mid_sendFloat, val, timeoutMS);
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

