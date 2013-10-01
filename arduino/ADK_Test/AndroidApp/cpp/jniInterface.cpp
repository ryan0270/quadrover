#include <jni.h>
#include <unistd.h>
#include <thread>
#include <android/log.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "ADKTest", __VA_ARGS__))


bool isShutdown = true;
bool doShutdown = true;
JavaVM *myJVM = NULL;
jobject obj = 0;
jclass cls = 0;
jmethodID mid_sendMotorCommands = 0;

enum
{
	COMM_ADK_PREFIX = 0xDDCCBBAA,
	COMM_ADK_SUFFIX = 0xDDCCBBAA-2,
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
	jclass tmpcls = env->FindClass("com/icsl/adktest/ADKTest");
	cls = (jclass)env->NewGlobalRef(tmpcls); // keep cls and, hence, mid_sendMotorCommands valid even after this function exits
	mid_sendMotorCommands = env->GetMethodID(cls, "sendMotorCommands","(IIII)Z");
	if(mid_sendMotorCommands == 0)
	{
		LOGI("Couldn't find java method sendMotorCommands");
		return -1;
	}
	else
		LOGI("Found java function sendMotorCommands");

	return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL Java_com_icsl_adktest_ADKTest_jniShutdown(JNIEnv* env, jobject thiz)
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
	int vals[4];
	vals[0] = 0;
	vals[1] = 1;
	vals[2] = 2;
	vals[3] = 3;
    while( !doShutdown )
    {
		if(env->CallBooleanMethod(obj, mid_sendMotorCommands, vals[0], vals[1], vals[2], vals[3]))
		{
			for(int i=0; i<4; i++)
				vals[i]++;
			LOGI("Sent");
		}
		else
			LOGI("Couldn't send commands");
		usleep(500e3);
    }

	myJVM->DetachCurrentThread();
	isShutdown = true;
}

JNIEXPORT void JNICALL Java_com_icsl_adktest_ADKTest_jniInit(JNIEnv* env, jobject thiz)
{
	obj = env->NewGlobalRef(thiz);

	std::thread th(&run);
	th.detach();
}

}

