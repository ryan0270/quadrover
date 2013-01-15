#include <jni.h>
#include <string.h>

#include "../cpp/QuadPhone.h"
#include <opencv2/core/core.hpp>
#include "../cpp/TNT/tnt.h"
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <vector>

//using namespace std;
//using namespace cv;

static ICSL::Quadrotor::ChadPhone *chadPhone = NULL;

extern "C" {

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_onJNIStart(JNIEnv* env, jobject thiz)
{
	if(chadPhone == NULL)
		chadPhone = new ICSL::Quadrotor::ChadPhone();
	else
	{
		chadPhone->shutdown();
		delete chadPhone;
		chadPhone = NULL;
		chadPhone = new ICSL::Quadrotor::ChadPhone();
	}
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_setAssetManager(JNIEnv* env, jobject thiz, jobject assetManager)
{
	if(chadPhone == NULL)
		return;

	AAssetManager *mgr = AAssetManager_fromJava(env, assetManager);
	chadPhone->setAssetManager(mgr);
	chadPhone->initialize();
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_setNumCpuCores(JNIEnv* env, jobject thiz, jint numCores)
{
	if(chadPhone == NULL)
		return;

	chadPhone->setNumCpuCores(numCores);
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_setLogDir(JNIEnv* env, jobject thiz, jstring jdir)
{
	if(chadPhone == NULL)
		return;

	const char *str = env->GetStringUTFChars(jdir, NULL);
	String dir(str);
	chadPhone->setLogDir(dir);
	env->ReleaseStringUTFChars(jdir, str);
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_startLogging(JNIEnv* env, jobject thiz)
{
	if(chadPhone == NULL)
		return;

	chadPhone->startLogging();
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_getImage(JNIEnv* env, jobject thiz, jlong addr)
{
	if(chadPhone == NULL)
		return;

	chadPhone->copyImageData((cv::Mat*)addr);
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_QuadPhone_QuadPhone_getGyroValue(JNIEnv* env, jobject thiz, jlong addr)
{
	if(chadPhone == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> gyro = chadPhone->getGyroValue();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)gyro[0][0];
	elem[1] = (jfloat)gyro[1][0];
	elem[2] = (jfloat)gyro[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_QuadPhone_QuadPhone_getAccelValue(JNIEnv* env, jobject thiz, jlong addr)
{
	if(chadPhone == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> accel= chadPhone->getAccelValue();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)accel[0][0];
	elem[1] = (jfloat)accel[1][0];
	elem[2] = (jfloat)accel[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_QuadPhone_QuadPhone_getMagValue(JNIEnv* env, jobject thiz, jlong addr)
{
	if(chadPhone == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> mag = chadPhone->getMagValue();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)mag[0][0];
	elem[1] = (jfloat)mag[1][0];
	elem[2] = (jfloat)mag[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT jfloatArray JNICALL Java_com_icsl_QuadPhone_QuadPhone_getAttitude(JNIEnv* env, jobject thiz, jlong addr)
{
	if(chadPhone == NULL)
		return env->NewFloatArray(0);

	TNT::Array2D<double> att= chadPhone->getAttitude();
	jfloatArray jval = env->NewFloatArray(3);
	jfloat *elem = env->GetFloatArrayElements(jval,0);
	elem[0] = (jfloat)att[0][0];
	elem[1] = (jfloat)att[1][0];
	elem[2] = (jfloat)att[2][0];

	env->ReleaseFloatArrayElements(jval, elem, 0);
	return jval;
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_toggleViewType(JNIEnv* env, jobject thiz)
{
	if(chadPhone == NULL)
		return;

	chadPhone->toggleViewType();
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_toggleUseIbvs(JNIEnv* env, jobject thiz)
{
	if(chadPhone == NULL)
		return;

	chadPhone->toggleUseIbvs();
}

JNIEXPORT jintArray JNICALL Java_com_icsl_QuadPhone_QuadPhone_getVisionParams(JNIEnv* env, jobject thiz)
{
	if(chadPhone == NULL)
		return NULL;

	toadlet::egg::Collection<int> vals = chadPhone->getVisionParams();

	jintArray jval = env->NewIntArray(vals.size());
	jint *elem = env->GetIntArrayElements(jval,0);
	for(int i=0; i<vals.size(); i++)
		elem[i] = vals[i];

	env->ReleaseIntArrayElements(jval, elem, 0);

	return jval;
}


JNIEXPORT jint JNICALL Java_com_icsl_QuadPhone_QuadPhone_getImageProcTimeMS(JNIEnv* env, jobject thiz)
{
	if(chadPhone == NULL)
		return -1;

	return chadPhone->getImageProcTimeMS();
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_setVisionParams(JNIEnv* env, jobject thiz, jintArray jval)
{
	if(chadPhone == NULL)
		return;

	toadlet::egg::Collection<int> vals(env->GetArrayLength(jval));
	jint *elem = env->GetIntArrayElements(jval,0);
	for(int i=0; i<vals.size(); i++)
		vals[i] = elem[i];

	env->ReleaseIntArrayElements(jval,elem,0);

	chadPhone->setVisionParams(vals);
}

JNIEXPORT bool JNICALL Java_com_icsl_QuadPhone_QuadPhone_pcIsConnected(JNIEnv* env, jobject thiz)
{
	if(chadPhone == NULL)
		return false;

	return chadPhone->pcIsConnected();
}

JNIEXPORT void JNICALL Java_com_icsl_QuadPhone_QuadPhone_onJNIStop(JNIEnv* env, jobject thiz)
{
	if(chadPhone != NULL)
	{
		chadPhone->shutdown();
		delete chadPhone;
		chadPhone = NULL;
	}
}

JNIEXPORT void JNI_OnUnload(JavaVM* vm, void *reserved)
{
	if(chadPhone != NULL)
	{
		chadPhone->shutdown();
		delete chadPhone;
		chadPhone = NULL;
	}
}

}
