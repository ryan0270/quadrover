/*
 * Copyright (C) 2010 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

//BEGIN_INCLUDE(all)
#include <jni.h>
#include <errno.h>

#include <fstream>
#include <sstream>
#include <iterator>
#include <memory>
#include <vector>
#include <list>

#include <EGL/egl.h>
#include <GLES/gl.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/nonfree/features2d.hpp>

#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>

#include "toadlet/egg.h"

#include "../../Rover/cpp/Data.h"
#include "../../Rover/cpp/TNT/tnt.h"
#include "../../Rover/cpp/TNT/jama_cholesky.h"
#include "../../Rover/cpp/TNT/jama_eig.h"
#include "../../Rover/cpp/TNT_Utils.h"
#include "../../Rover/cpp/Time.h"
#include "../../Rover/cpp/constants.h"
#include "mapFuncs.h"

#include <android/sensor.h>
#include <android/log.h>
#include <native_app_glue/android_native_app_glue.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "VisionTestApp", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "VisionTestApp", __VA_ARGS__))

using namespace std;
using namespace ICSL::Rover;
using namespace ICSL::Quadrotor;
using namespace ICSL::Constants;
using namespace TNT;

void loadPhoneLog(String filename, 
				vector<pair<int, Time> > &imgIdList,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList,
				list<shared_ptr<DataVector> > &errCovList
				);
void loadPcLog(String filename,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList
		);
vector<string> tokenize(string str);

void findPoints(cv::Mat const &img, vector<cv::Point2f> &pts, float const &minDistance, float const &qualityLevel);
void findPoints(cv::Mat const &img, vector<cv::KeyPoint> &pts, float const &minDistance, float const &qualityLevel);

static void HarrisResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize, float harris_k);
static void EigenValResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize);

enum LogIDs
{
	LOG_ID_ACCEL = 1,
	LOG_ID_GYRO = 2,
	LOG_ID_MAGNOMETER = 3,
	LOG_ID_PRESSURE = 4,
	LOG_ID_IMAGE = 10,
	LOG_ID_GYRO_BIAS = -1003,
	LOG_ID_OBSV_ANG_INNOVATION = -1004,
	LOG_ID_OPTIC_FLOW = 12345,
	LOG_ID_OBSV_TRANS_ATT_BIAS = -710,
	LOG_ID_OBSV_TRANS_FORCE_GAIN = -711,
	LOG_ID_BAROMETER_HEIGHT = 1234,
	LOG_ID_MOTOR_CMDS = -1000,
	LOG_ID_CUR_ATT = -1002,
	LOG_ID_CUR_TRANS_STATE = -1012,
	LOG_ID_RECEIVE_VICON = 700,
	LOG_ID_CAMERA_POS = 800,
	LOG_ID_KALMAN_ERR_COV = -720,
};

/**
 * Our saved state data.
 */
struct saved_state {
    float angle;
    int32_t x;
    int32_t y;
};

/**
 * Shared state for our app.
 */
struct engine {
    struct android_app* app;

    ASensorManager* sensorManager;
    const ASensor* accelerometerSensor;
    ASensorEventQueue* sensorEventQueue;

    int animating;
    EGLDisplay display;
    EGLSurface surface;
    EGLContext context;
    int32_t width;
    int32_t height;
    struct saved_state state;
};

/**
 * Initialize an EGL context for the current display.
 */
static int engine_init_display(struct engine* engine) {
    // initialize OpenGL ES and EGL

    /*
     * Here specify the attributes of the desired configuration.
     * Below, we select an EGLConfig with at least 8 bits per color
     * component compatible with on-screen windows
     */
    const EGLint attribs[] = {
            EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
            EGL_BLUE_SIZE, 8,
            EGL_GREEN_SIZE, 8,
            EGL_RED_SIZE, 8,
            EGL_NONE
    };
    EGLint w, h, dummy, format;
    EGLint numConfigs;
    EGLConfig config;
    EGLSurface surface;
    EGLContext context;

    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

    eglInitialize(display, 0, 0);

    /* Here, the application chooses the configuration it desires. In this
     * sample, we have a very simplified selection process, where we pick
     * the first EGLConfig that matches our criteria */
    eglChooseConfig(display, attribs, &config, 1, &numConfigs);

    /* EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
     * guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
     * As soon as we picked a EGLConfig, we can safely reconfigure the
     * ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. */
    eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);

    ANativeWindow_setBuffersGeometry(engine->app->window, 0, 0, format);

    surface = eglCreateWindowSurface(display, config, engine->app->window, NULL);
    context = eglCreateContext(display, config, NULL, NULL);

    if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
        LOGW("Unable to eglMakeCurrent");
        return -1;
    }

    eglQuerySurface(display, surface, EGL_WIDTH, &w);
    eglQuerySurface(display, surface, EGL_HEIGHT, &h);

    engine->display = display;
    engine->context = context;
    engine->surface = surface;
    engine->width = w;
    engine->height = h;
    engine->state.angle = 0;

    // Initialize GL state.
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);
    glDisable(GL_DEPTH_TEST);

    return 0;
}

/**
 * Just the current frame in the display.
 */
static void engine_draw_frame(struct engine* engine) 
{
    if (engine->display == NULL) {
        // No display.
        return;
    }

    // Just fill the screen with a color.
    glClearColor(((float)engine->state.x)/engine->width, engine->state.angle,
            ((float)engine->state.y)/engine->height, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    eglSwapBuffers(engine->display, engine->surface);
}

/**
 * Tear down the EGL context currently associated with the display.
 */
static void engine_term_display(struct engine* engine) 
{
    if (engine->display != EGL_NO_DISPLAY) {
        eglMakeCurrent(engine->display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (engine->context != EGL_NO_CONTEXT) {
            eglDestroyContext(engine->display, engine->context);
        }
        if (engine->surface != EGL_NO_SURFACE) {
            eglDestroySurface(engine->display, engine->surface);
        }
        eglTerminate(engine->display);
    }
    engine->animating = 0;
    engine->display = EGL_NO_DISPLAY;
    engine->context = EGL_NO_CONTEXT;
    engine->surface = EGL_NO_SURFACE;
}

/**
 * Process the next input event.
 */
static int32_t engine_handle_input(struct android_app* app, AInputEvent* event) 
{
    struct engine* engine = (struct engine*)app->userData;
    if (AInputEvent_getType(event) == AINPUT_EVENT_TYPE_MOTION) {
        engine->animating = 1;
        engine->state.x = AMotionEvent_getX(event, 0);
        engine->state.y = AMotionEvent_getY(event, 0);
        return 1;
    }
    return 0;
}

/**
 * Process the next main command.
 */
static void engine_handle_cmd(struct android_app* app, int32_t cmd) 
{
    struct engine* engine = (struct engine*)app->userData;
    switch (cmd) {
        case APP_CMD_SAVE_STATE:
            // The system has asked us to save our current state.  Do so.
            engine->app->savedState = malloc(sizeof(struct saved_state));
            *((struct saved_state*)engine->app->savedState) = engine->state;
            engine->app->savedStateSize = sizeof(struct saved_state);
            break;
        case APP_CMD_INIT_WINDOW:
            // The window is being shown, get it ready.
            if (engine->app->window != NULL) {
                engine_init_display(engine);
                engine_draw_frame(engine);
            }
            break;
        case APP_CMD_TERM_WINDOW:
            // The window is being hidden or closed, clean it up.
            engine_term_display(engine);
            break;
        case APP_CMD_GAINED_FOCUS:
            // When our app gains focus, we start monitoring the accelerometer.
            if (engine->accelerometerSensor != NULL) {
                ASensorEventQueue_enableSensor(engine->sensorEventQueue,
                        engine->accelerometerSensor);
                // We'd like to get 60 events per second (in us).
                ASensorEventQueue_setEventRate(engine->sensorEventQueue,
                        engine->accelerometerSensor, (1000L/60)*1000);
            }
            break;
        case APP_CMD_LOST_FOCUS:
            // When our app loses focus, we stop monitoring the accelerometer.
            // This is to avoid consuming battery while not being used.
            if (engine->accelerometerSensor != NULL) {
                ASensorEventQueue_disableSensor(engine->sensorEventQueue,
                        engine->accelerometerSensor);
            }
            // Also stop animating.
            engine->animating = 0;
            engine_draw_frame(engine);
            break;
    }
}

/**
 * This is the main entry point of a native application that is using
 * android_native_app_glue.  It runs in its own thread, with its own
 * event loop for receiving input events and doing other things.
 */
void android_main(struct android_app* state) {
    struct engine engine;

    // Make sure glue isn't stripped.
    app_dummy();

    memset(&engine, 0, sizeof(engine));
    state->userData = &engine;
    state->onAppCmd = engine_handle_cmd;
    state->onInputEvent = engine_handle_input;
    engine.app = state;

    // Prepare to monitor accelerometer
//    engine.sensorManager = ASensorManager_getInstance();
//    engine.accelerometerSensor = ASensorManager_getDefaultSensor(engine.sensorManager,
//            ASENSOR_TYPE_ACCELEROMETER);
//    engine.sensorEventQueue = ASensorManager_createEventQueue(engine.sensorManager,
//            state->looper, LOOPER_ID_USER, NULL, NULL); 
    if (state->savedState != NULL) {
        // We are starting with a previous saved state; restore from it.
        engine.state = *(struct saved_state*)state->savedState;
    }

	////////////////////////////////////////////////////////////////////////////////////////////////////
	Log::alert("start chadding");

	string imgDir = "/sdcard/VisionTestApp/video";

	Log::alert("Loading log");
	vector<pair<int, Time> > imgIdList;
	list<shared_ptr<DataVector> > attDataList, transDataList, errCovList;
	loadPhoneLog("/sdcard/VisionTestApp/log.txt", imgIdList, attDataList, transDataList, errCovList);
	if(imgIdList.size() == 0)
	{
		Log::alert("Failed loading data");
		return;
	}

	// preload all images
	list<pair<int, cv::Mat> > imgList;
	int imgId = 1700;
	for(int i=0; i<100; i++)
	{
		cv::Mat img;
		while(img.data == NULL)
		{
			stringstream ss;
			ss << "img_" << ++imgId << ".bmp";
			img = cv::imread(imgDir+"/"+ss.str());
		}

		imgList.push_back(pair<int, cv::Mat>(imgId, img));
	}

	Log::alert("Loading vicon data");
	list<shared_ptr<DataVector> > viconAttDataList, viconTransDataList;
	loadPcLog("/sdcard/VisionTestApp/pcData_fullState.txt", viconAttDataList, viconTransDataList);

	Array2D<double> rotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI), createRotMat(0,(double)PI));
	Array2D<double> rotCamToPhone2 = blkdiag(rotCamToPhone, rotCamToPhone);
	Array2D<double> rotCamToPhone3 = blkdiag(rotCamToPhone, blkdiag(rotCamToPhone, rotCamToPhone));

	Array2D<double> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<double> rotPhoneToCam2 = transpose(rotCamToPhone2);
	Array2D<double> rotPhoneToCam3 = transpose(rotCamToPhone3);

	Log::alert("w00t w00t");

	cv::Mat img, imgGray, imgGrayRaw, imgPrev;
	vector<cv::Point2f> prevPoints, curPoints;
	Time prevTime, curTime;
	Array2D<double> attPrev, attCur, attChange;

	Array2D<double> Sv(3,3,0.0);
	double sz;
	double focalLength = 3.7*640/5.76;
	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = 2*pow(5,2);
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

	cv::Point2f center(320,240);

	tbb::task_scheduler_init init;


	Log::alert("////////////////////////////////////////////////////////////////////////////////////////////////////");

	float qualityLevel = 0.01;
	float minDistance = 20;

//	int maxCorners = 1000;
//	double qualityLevel = 0.02;
//	double minDistance = 30;
//	int blockSize = 3;
//	bool useHarrisDetector = false;
//	double k=0.04;
//	cv::GoodFeaturesToTrackDetector featureDetector(maxCorners, qualityLevel, minDistance, blockSize, useHarrisDetector, k);

	bool useViconState = false;

	//////////////////////////////////////////////////////////////////
	// using ORB descriptors
	//////////////////////////////////////////////////////////////////
	{
		curTime.setTimeMS(0);
		attPrev = createIdentity(3);
		attCur = createIdentity(3);
		attChange = createIdentity(3);
		int imgIdx = 0;
		double maxSigma = 30;
		list<pair<int, cv::Mat> >::iterator iter_imgList;
		iter_imgList = imgList.begin();
		long long numFeaturesAccum = 0;
		long long numMatchesAccum = 0;
		vector<cv::KeyPoint> prevKp, curKp;
		cv::Mat prevDescriptors, curDescriptors;

		Array2D<double> attState(6,1), transState(6,1), viconAttState(6,1), viconTransState(6,1);
		Array2D<double> errCov1(9,1), errCov(6,6,0.0);

		Array2D<double> mv(3,1), omega(3,1), vel;
		double mz, dt, z;

		fstream fs("../orbResults.txt", fstream::out);
		Time begin;
		while(iter_imgList != imgList.end())
		{
			imgId = iter_imgList->first;
			img = iter_imgList->second;
			iter_imgList++;
			while(imgIdList[imgIdx].first < imgId && imgIdx < imgIdList.size()) 
				imgIdx++;
			if(imgIdx == imgIdList.size() )
			{
				Log::alert("imgIdx exceeded vector");
				return;
			}
			prevTime.setTime(curTime);
			curTime = imgIdList[imgIdx].second;
			attState.inject(Data::interpolate(curTime, attDataList));
			transState.inject(Data::interpolate(curTime, transDataList));
			viconAttState.inject(Data::interpolate(curTime, viconAttDataList));
			viconTransState.inject(Data::interpolate(curTime, viconTransDataList));

			errCov1.inject(Data::interpolate(curTime, errCovList));
			for(int i=0; i<3; i++)
			{
				errCov[i][i] = errCov1[i][0];
				errCov[i][i+3] = errCov1[i+3][0];
				errCov[i+3][i+3] = errCov1[i+6][0];
			}

			// Rotate to camera coords
			attState = matmult(rotPhoneToCam2, attState);
			transState = matmult(rotPhoneToCam2, transState);
			errCov = matmult(rotPhoneToCam2, matmult(errCov, transpose(rotPhoneToCam2)));
			viconAttState = matmult(rotPhoneToCam2, viconAttState);
			viconTransState = matmult(rotPhoneToCam2, viconTransState);

			attPrev.inject(attCur);
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//				attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );

			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);

//			cv::cvtColor(img, imgGray, CV_BGR2GRAY);

			curKp.swap(prevKp);
			curKp.clear();
//			featureDetector.detect(imgGray, curKp);
			findPoints(imgGray, curKp, minDistance, qualityLevel);

			numFeaturesAccum += curKp.size();

			prevDescriptors = curDescriptors;
			curDescriptors.release();
			curDescriptors.data = NULL;
			cv::OrbFeatureDetector extractor(500, 2.0f, 4, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31);
			extractor.compute(imgGray, curKp, curDescriptors);

			curDescriptors.convertTo(curDescriptors, CV_32F);

			cv::FlannBasedMatcher matcher;
			vector<cv::DMatch> matches;
			matcher.match(prevDescriptors, curDescriptors, matches);

			float minDist = 1000;
			for(int i=0; i<matches.size(); i++)
				minDist = min(matches[i].distance, minDist);

			vector<cv::DMatch> goodMatches(matches);
//			for(int i=0; i<matches.size(); i++)
//				if(matches[i].distance < 5.0*minDist) goodMatches.push_back(matches[i]);
			numMatchesAccum += goodMatches.size();

			int N1, N2;
			N1 = N2 = goodMatches.size();
			Array2D<double> C(N1+1, N2+1, 0.0); // the extra row and column will stay at zero
			vector<cv::Point2f> prevPoints(N1), curPoints(N2);
			for(int i=0; i<N1; i++)
			{
				C[i][i] = 1;

				int idx1 = goodMatches[i].queryIdx;
				int idx2 = goodMatches[i].trainIdx;
				prevPoints[i] = prevKp[idx1].pt-center;
				curPoints[i] = curKp[idx2].pt-center;
			}

			// MAP velocity and height
			if(useViconState)
			{
				mv[0][0] = viconTransState[3][0];
				mv[1][0] = viconTransState[4][0];
				mv[2][0] = viconTransState[5][0];

				mz = -viconTransState[2][0]; // in camera coords, z is flipped
			}
			else
			{
				mv[0][0] = transState[3][0];
				mv[1][0] = transState[4][0];
				mv[2][0] = transState[5][0];

				mz = -transState[2][0]; // in camera coords, z is flipped
			}

			mz -= 0.1; // camera to vicon markers offset

			double dt;
			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;
			Array2D<double> omega = logSO3(attChange, dt);

			sz = sqrt( errCov[2][2]);
			Sv[0][0] = errCov[3][3];
			Sv[1][1] = errCov[4][4];
			Sv[2][2] = errCov[5][5];

			if(curPoints.size() > 0)
			{
				computeMAPEstimate(vel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);

//				fs << curTime.getMS() << "\t" << 98 << "\t";
//				for(int i=0; i<vel.dim1(); i++)
//					fs << vel[i][0] << "\t";
//				fs << endl;
//				fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;
			}
			imgPrev = img;

//			img.release();
//			img.data = NULL;
		}

//		fs.close();

		Log::alert(String() + "Avg num ORB features: " + ((double)numFeaturesAccum)/imgList.size());
		Log::alert(String() + "Avg num ORB matches: " + ((double)numMatchesAccum)/imgList.size());
		Log::alert(String() + "ORB time: " + begin.getElapsedTimeMS()/1.0e3);

	}

	Log::alert("////////////////////////////////////////////////////////////////////////////////////////////////////");

	//////////////////////////////////////////////////////////////////
	// Once for new algo
	//////////////////////////////////////////////////////////////////
	{
		curTime.setTimeMS(0);
		attPrev = createIdentity(3);
		attCur = createIdentity(3);
		attChange = createIdentity(3);
		int imgIdx = 0;
		double maxSigma = 30;
		list<pair<int, cv::Mat> >::iterator iter_imgList;
		iter_imgList = imgList.begin();
		long long numFeaturesAccum = 0;
		double numMatchesAccum = 0;

		Array2D<double> attState(6,1), transState(6,1), viconAttState(6,1), viconTransState(6,1);
		Array2D<double> errCov1(9,1), errCov(6,6,0.0);

		Array2D<double> mv(3,1), omega(3,1), vel;
		double mz, dt, z;

		vector<pair<Array2D<double>, Array2D<double> > > priorDistList;

		Array2D<double> C;

double ts0, ts1, ts2, ts3, ts4, ts5, ts6, ts7;
ts0 = ts1 = ts2 = ts3 = ts4 = ts5 = ts6 = ts7 = 0;
Time t0;

		fstream fs("../mapResults.txt", fstream::out);
		Time begin;
		while(iter_imgList != imgList.end())
		{
t0.setTime();
			imgId = iter_imgList->first;
			img = iter_imgList->second;
			iter_imgList++;
			while(imgIdList[imgIdx].first < imgId && imgIdx < imgIdList.size()) 
				imgIdx++;
			if(imgIdx == imgIdList.size() )
			{
				Log::alert("imgIdx exceeded vector");
				return;
			}
			prevTime.setTime(curTime);
			curTime = imgIdList[imgIdx].second;
			attState.inject(Data::interpolate(curTime, attDataList));
			transState.inject(Data::interpolate(curTime, transDataList));
			viconAttState.inject(Data::interpolate(curTime, viconAttDataList));
			viconTransState.inject(Data::interpolate(curTime, viconTransDataList));

			errCov1.inject(Data::interpolate(curTime, errCovList));
			for(int i=0; i<3; i++)
			{
				errCov[i][i] = errCov1[i][0];
				errCov[i][i+3] = errCov1[i+3][0];
				errCov[i+3][i+3] = errCov1[i+6][0];
			}

			// Rotate to camera coords
			attState = matmult(rotPhoneToCam2, attState);
			transState = matmult(rotPhoneToCam2, transState);
			errCov = matmult(rotPhoneToCam2, matmult(errCov, transpose(rotPhoneToCam2)));
			viconAttState = matmult(rotPhoneToCam2, viconAttState);
			viconTransState = matmult(rotPhoneToCam2, viconTransState);

			attPrev.inject(attCur);
			if(useViconState)
				attCur.inject( createRotMat_ZYX( viconAttState[2][0], viconAttState[1][0], viconAttState[0][0]) );
			else
				attCur.inject( createRotMat_ZYX( attState[2][0], attState[1][0], attState[0][0]) );
//			attChange.inject( matmult(attCur, transpose(attPrev)) );
			attChange.inject( matmult(transpose(attCur), attPrev) );

ts0 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			/////////////////////////////////////////////////////
			cv::cvtColor(img, imgGrayRaw, CV_BGR2GRAY);
			cv::GaussianBlur(imgGrayRaw, imgGray, cv::Size(5,5), 2, 2);

//			cv::cvtColor(img, imgGray, CV_BGR2GRAY);

ts1 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			curPoints.swap(prevPoints);
			findPoints(imgGray, curPoints, minDistance, qualityLevel);
			numFeaturesAccum += curPoints.size();

ts2 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			/////////////////////////////////////////////////////
			//  Prior distributions
			for(int i=0; i<curPoints.size(); i++)
				curPoints[i] -= center;
			if(useViconState)
			{
				mv[0][0] = viconTransState[3][0];
				mv[1][0] = viconTransState[4][0];
				mv[2][0] = viconTransState[5][0];

				mz = -viconTransState[2][0]; // in camera coords, z is flipped
			}
			else
			{
				mv[0][0] = transState[3][0];
				mv[1][0] = transState[4][0];
				mv[2][0] = transState[5][0];

				mz = -transState[2][0]; // in camera coords, z is flipped
			}

			mz -= 0.1; // camera to vicon markers offset

			if(prevTime.getMS() > 0)
				dt = Time::calcDiffNS(prevTime, curTime)/1.0e9;
			else
				dt = 0;
			omega.inject(logSO3(attChange, dt));
			priorDistList.clear();
			priorDistList = calcPriorDistributions(prevPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);

ts3 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			// Correspondence
			C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv);

ts4 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			for(int i=0; i<C.dim1()-1; i++)
				for(int j=0; j<C.dim2()-1; j++)
					numMatchesAccum += C[i][j];

			sz = sqrt( errCov[2][2]);
			Sv[0][0] = errCov[3][3];
			Sv[1][1] = errCov[4][4];
			Sv[2][2] = errCov[5][5];

			// MAP velocity and height
			if(prevPoints.size() > 0)
			{
				computeMAPEstimate(vel, z, prevPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
ts6 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();

				fs << curTime.getMS() << "\t" << 98 << "\t";
				for(int i=0; i<vel.dim1(); i++)
					fs << vel[i][0] << "\t";
				fs << endl;
				fs << curTime.getMS() << "\t" << 99 << "\t" << z << endl;
			}

ts7 += t0.getElapsedTimeNS()/1.0e9; t0.setTime();
			//////////////////////////////////////////////////
			//Drawing
			// prior distributions
//			if(priorDistList.size() > 0)
//				maxSigma = 0;
//			cv::Mat overlay = img.clone();
//			for(int i=0; i<priorDistList.size(); i++)
//			{
//				Array2D<double> Sd = priorDistList[i].second;
//				JAMA::Eigenvalue<double> eig_Sd(Sd);
//				Array2D<double> V, D;
//				eig_Sd.getV(V);
//				eig_Sd.getD(D);
//
//				for(int j=0; j<D.dim1(); j++)
//					maxSigma = max(maxSigma, sqrt(D[j][j]));

//				cv::Point2f pt(priorDistList[i].first[0][0], priorDistList[i].first[1][0]);
//				double width = 2*sqrt(D[0][0]);
//				double height = 2*sqrt(D[1][1]);
//				double theta = atan2(V[1][0], V[0][0]);
//				cv::RotatedRect rect(pt+center, cv::Size(width, height), theta);
//				cv::ellipse(overlay, rect, cv::Scalar(255,0,0), -1);
//			}
//			double opacity = 0.3;
//			cv::addWeighted(overlay, opacity, img, 1-opacity, 0, img);

			// current points
//			for(int i=0; i<curPoints.size(); i++)
//				cv::circle(img, curPoints[i]+center, 4, cv::Scalar(0,0,255), -1);
//			imshow("chad",img);
  
//			keypress = cv::waitKey() % 256;

			img.release();
			img.data = NULL;
		}

		fs.close();

		Log::alert(String()+"Avg num features: " + ((double)numFeaturesAccum)/imgList.size());
		Log::alert(String()+"Avg num matches: " + ((double)numMatchesAccum)/imgList.size());
		Log::alert(String()+"New total time: " + begin.getElapsedTimeMS()/1.0e3);
		Log::alert(String()+"\tts0: " + ts0);
		Log::alert(String()+"\tts1: " + ts1);
		Log::alert(String()+"\tts2: " + ts2);
		Log::alert(String()+"\tts3: " + ts3);
		Log::alert(String()+"\tts4: " + ts4);
		Log::alert(String()+"\tts5: " + ts5);
		Log::alert(String()+"\tts6: " + ts6);
		Log::alert(String()+"\tts7: " + ts7);

	}

	////////////////////////////////////////////////////////////////////////////////////////////////////

    // loop waiting for stuff to do.
    while (1) 
    {
        // Read all pending events.
        int ident;
        int events;
        struct android_poll_source* source;

        // If not animating, we will block forever waiting for events.
        // If animating, we loop until all events are read, then continue
        // to draw the next frame of animation.
        while ((ident=ALooper_pollAll(engine.animating ? 0 : -1, NULL, &events,
                (void**)&source)) >= 0) {

            // Process this event.
            if (source != NULL) {
                source->process(state, source);
            }

            // If a sensor has data, process it now.
            if (ident == LOOPER_ID_USER) {
                if (engine.accelerometerSensor != NULL) {
                    ASensorEvent event;
                    while (ASensorEventQueue_getEvents(engine.sensorEventQueue,
                            &event, 1) > 0) {
//                        LOGI("accelerometer: x=%f y=%f z=%f",
//                               event.acceleration.x, event.acceleration.y,
//                               event.acceleration.z);
                    }
                }
            }

            // Check if we are exiting.
            if (state->destroyRequested != 0) {
                engine_term_display(&engine);
                return;
            }
        }

        if (engine.animating) {
            // Done with events; draw next animation frame.
            engine.state.angle += .01f;
            if (engine.state.angle > 1) {
                engine.state.angle = 0;
            }

            // Drawing is throttled to the screen update rate, so there
            // is no need to do timing here.
            engine_draw_frame(&engine);
        }
    }

	LOGI("chad accomplished");
}

vector<string> tokenize(string str)
{
	istringstream iss(str);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens) );
	return tokens;
}

void loadPhoneLog(String filename, 
				vector<pair<int, Time> > &imgIdList,
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList,
				list<shared_ptr<DataVector> > &errCovList
				)
{
	imgIdList.clear();
	angleStateList.clear();
	transStateList.clear();
	errCovList.clear();

	string line;
	ifstream file(filename.c_str());
	if(file.is_open())
	{
		getline(file, line); // first line is a throw-away
		vector<string> tokens;

		while(file.good())
		{
			getline(file, line);
			stringstream ss(line);
			double time;
			int type;
			ss >> time >> type;
			switch(type)
			{
				case LOG_ID_IMAGE:
					{
						int imgID;
						ss >> imgID;

						Time t;
						t.setTimeMS(time);
						pair<int, Time> data(imgID, t);
						imgIdList.push_back(data);
					}
					break;
				case LOG_ID_CUR_ATT:
					{
						Array2D<double> attState(6,1);
						for(int i=0; i<6; i++)
							ss >> attState[i][0];

						shared_ptr<DataVector> data(new DataVector());
						data->timestamp.setTimeMS(time);
						data->data = attState.copy();
						angleStateList.push_back(data);
					}
					break;
				case LOG_ID_CUR_TRANS_STATE:
					{
						Array2D<double> transState(6,1);
						for(int i=0; i<6; i++)
							ss >> transState[i][0];

						shared_ptr<DataVector> data(new DataVector());
						data->timestamp.setTimeMS(time);
						data->data = transState.copy();
						transStateList.push_back(data);
					}
					break;
				case LOG_ID_KALMAN_ERR_COV:
					{
						Array2D<double> errCov(9,1);
						for(int i=0; i<9; i++)
							ss >> errCov[i][0];

						shared_ptr<DataVector> data(new DataVector());
						data->timestamp.setTimeMS(time);
						data->data = errCov.copy();
						errCovList.push_back(data);

					}
					break;
			}
		}

		file.close();
	}
	else
		Log::alert("Couldn't find " + filename);
}

void loadPcLog(String filename, 
				list<shared_ptr<DataVector> > &angleStateList,
				list<shared_ptr<DataVector> > &transStateList
				)
{
	angleStateList.clear();
	transStateList.clear();

	Array2D<double> rotViconToQuad = createRotMat(0, (double)PI);
	Array2D<double> rotQuadToPhone = matmult(createRotMat(2,-0.25*PI), createRotMat(0,(double)PI));
	Array2D<double> rotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI), createRotMat(0,(double)PI));
	Array2D<double> rotPhoneToCam = transpose(rotCamToPhone);
	Array2D<double> rotViconToPhone = matmult(rotQuadToPhone, rotViconToQuad);

	Array2D<double> rotQuadToPhone2 = blkdiag(rotQuadToPhone, rotQuadToPhone);
	Array2D<double> rotViconToPhone2 = blkdiag(rotViconToPhone, rotViconToPhone);

	string line;
	ifstream file(filename.c_str());
	if(file.is_open())
	{
		vector<string> tokens;

		while(file.good())
		{
			getline(file, line);
			stringstream ss(line);
			double time;
			int type;
			ss >> time >> type;

			// This file is assumed to be all state data
			Array2D<double> angleState(6,1), transState(6,1);
			for(int i=0; i<6; i++)
				ss >> angleState[i][0];
			for(int i=0; i<6; i++)
				ss >> transState[i][0];

			angleState = matmult(rotQuadToPhone2, angleState);
			transState = matmult(rotViconToPhone2, transState);

			shared_ptr<DataVector> dataAngle(new DataVector()), dataTrans(new DataVector());;
			dataAngle->timestamp.setTimeMS(time);
			dataAngle->data = angleState.copy();
			dataTrans->timestamp.setTimeMS(time);
			dataTrans->data = transState.copy();
	
			angleStateList.push_back(dataAngle);
			transStateList.push_back(dataTrans);
		}

		file.close();
	}
	else
		Log::alert("Couldn't find " + filename);
}

void findPoints(cv::Mat const &img, vector<cv::Point2f> &pts, float const &minDistance, float const &qualityLevel)
{
	vector<cv::KeyPoint> kp;
	findPoints(img, kp, minDistance, qualityLevel);
	cv::KeyPoint::convert(kp, pts);
}

void findPoints(cv::Mat const &img, vector<cv::KeyPoint> &pts, float const &minDistance, float const &qualityLevel)
{
	vector<cv::KeyPoint> tempKp1;
	cv::Ptr<cv::FastFeatureDetector> fastDetector(new cv::FastFeatureDetector(10));
	int maxKp = 1000;
	int gridRows = 3;
	int gridCols = 3;
	cv::GridAdaptedFeatureDetector detector(fastDetector, maxKp, gridRows, gridCols);
	detector.detect(img, tempKp1);
//	FAST(img, tempKp1, 10, true);
//	HarrisResponses(img, tempKp, 7, 0.04f);
	EigenValResponses(img, tempKp1, 5);

	float maxScore = -0xFFFFFFF;
	for(int i=0; i<tempKp1.size(); i++)
		maxScore = max(maxScore, tempKp1[i].response);

	vector<cv::KeyPoint> tempKp;
	tempKp.reserve(tempKp1.size());
	float threshold = qualityLevel*maxScore;
	for(int i=0; i<tempKp1.size(); i++)
		if(tempKp1[i].response > threshold)
			tempKp.push_back(tempKp1[i]);

//	cv::Mat eig;
//	eig.create(img.size(), CV_32F);
//	int MINEIGVAL = 0;
//	int HARRIS = 1;
//	cornerMinEigenVal(img, eig, 7, 3, MINEIGVAL);
//	for(int i=0; i<tempKp.size(); i++)
//	{
//		int x = tempKp[i].pt.x;
//		int y = tempKp[i].pt.y;
//		const float* row = (const float*)(eig.data + y*eig.step);
//		float eig = row[x];
//cout << "loc: (" << x << ", " << y << ")" << endl;
//cout << "point " << i << " eig 1: " << eig << endl;
//cout << "point " << i << " eig 2: " << tempKp[i].response << endl;
//cout << "----------------------------------" << endl;
//	}

	// group points into grid
	const int cellSize = minDistance+0.5;
	const int nGridX= (img.cols+cellSize-1)/cellSize;
	const int nGridY= (img.rows+cellSize-1)/cellSize;

	vector<vector<int> > grids(nGridX*nGridY); // stores index of keypoints in each grid
	vector<int> gridId(tempKp.size());
	for(int kpId=0; kpId < tempKp.size(); kpId++)
	{
		int xGrid = tempKp[kpId].pt.x / cellSize;
		int yGrid = tempKp[kpId].pt.y / cellSize;
		int gid = yGrid*nGridX+xGrid;
		grids[gid].push_back(kpId);
		gridId[kpId] = gid;
	}

	// Now pick the strongest points 
	// Right now, for reduced computation, it is using a minDistance x minDistance box for exclusion rather
	// than a minDistance radius circle
	vector<bool> isActive(gridId.size(), true);
	pts.clear();
	for(int i=0; i<gridId.size(); i++)
	{
		if(!isActive[i])
			continue;

		float curResponse = tempKp[i].response;
		cv::Point2f *curPt = &tempKp[i].pt;

		// turn off other points in this grid
		int gid = gridId[i];
		bool isStrongest = true;;
//		for(int j=0; j<grids[gid].size(); j++)
		int j=0;
		while(isStrongest && j < grids[gid].size() )
		{
			int kpId = grids[gid][j];
			if(kpId != i && curResponse >= tempKp[kpId].response)
				isActive[kpId] = false;
			else if(kpId != i && isActive[kpId])
				isStrongest = false;

			j++;
		}

		if(!isStrongest)
		{
			isActive[i] = false;
			continue;
		}

		// need to check neighbor grids too
		int xGrid = gid % nGridX;
		int yGrid = gid / nGridX;

		int neighborOffsetX[] = {-1, 0, 1, -1, 1, -1, 0, 1};
		int neighborOffsetY[] = {-1, -1, -1, 0, 0, 1, 1, 1};
//		for(int j=0; j<8; j++)
		j = 0;
		while(isStrongest && j < 8)
		{
			int nx = xGrid+neighborOffsetX[j];
			int ny = yGrid+neighborOffsetY[j];
			if(nx < 0 || nx > nGridX-1 || ny < 0 || ny > nGridY-1)
			{
				j++;
				continue;
			}

			int ngid = ny*nGridX+nx;
//			for(int k=0; k<grids[ngid].size(); k++)
			int k=0;
			while(isStrongest && k < grids[ngid].size() )
			{
				int kpId = grids[ngid][k];
				if(!isActive[kpId])
				{
					k++;
					continue;
				}

				cv::Point2f *pt = &tempKp[kpId].pt;
				if(abs(pt->x - curPt->x) <= minDistance && abs(pt->y - curPt->y) <= minDistance)
				{
					if(kpId != i && curResponse >= tempKp[kpId].response)
						isActive[kpId] = false;
					else if(kpId != i && isActive[kpId])
						isStrongest = false;
				}

				k++;
			}

			j++;
		}

		if(!isStrongest)
		{
			isActive[i] = false;
			continue;
		}

//		double minD = 0xFFFFFF;
//		double minJ = 0;
//		for(int j=0; j<tempKp.size(); j++)
//		{
//			if(j == i || !isActive[j])
//				continue;
//			cv::Point2f *pt = &tempKp[j].pt;
//			double dist = sqrt( pow(curPt->x - pt->x, 2) + pow(curPt->y - pt->y,2));
//			if(dist < minD)
//			{
//				minD = dist;
//				minJ =j;
//			}
//		}

		pts.push_back(tempKp[i]);
	}

//	sort(pts.begin(), pts.end(), [&](cv::KeyPoint const &a, cv::KeyPoint const &b){return a.response > b.response;});
}

// taken from OpenCV ORB code
static void HarrisResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize, float harris_k)
{
    CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );

    size_t ptidx, ptsize = pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    int step = (int)(img.step/img.elemSize1());
    int r = blockSize/2;

    float scale = (1 << 2) * blockSize * 255.0f;
    scale = 1.0f / scale;
    float scale_sq_sq = scale * scale * scale * scale;

	cv::AutoBuffer<int> ofsbuf(blockSize*blockSize);
    int* ofs = ofsbuf;
    for( int i = 0; i < blockSize; i++ )
        for( int j = 0; j < blockSize; j++ )
            ofs[i*blockSize + j] = (int)(i*step + j);

    for( ptidx = 0; ptidx < ptsize; ptidx++ )
    {
        int x0 = cvRound(pts[ptidx].pt.x - r);
        int y0 = cvRound(pts[ptidx].pt.y - r);

        const uchar* ptr0 = ptr00 + y0*step + x0;
        int a = 0, b = 0, c = 0;

        for( int k = 0; k < blockSize*blockSize; k++ )
        {
            const uchar* ptr = ptr0 + ofs[k];
            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        }
        pts[ptidx].response = ((float)a * b - (float)c * c -
                               harris_k * ((float)a + b) * ((float)a + b))*scale_sq_sq;
    }
}

static void EigenValResponses(const cv::Mat& img, vector<cv::KeyPoint>& pts, int blockSize)
{
    CV_Assert( img.type() == CV_8UC1 && blockSize*blockSize <= 2048 );

    size_t ptidx, ptsize = pts.size();

    const uchar* ptr00 = img.ptr<uchar>();
    int step = (int)(img.step/img.elemSize1());
    int r = blockSize/2;

    float scale = (1 << 2) * blockSize * 255.0f;
    scale = 1.0f / scale;
    float scale_sq = scale * scale;

	cv::AutoBuffer<int> ofsbuf(blockSize*blockSize);
    int* ofs = ofsbuf;
    for( int i = 0; i < blockSize; i++ )
        for( int j = 0; j < blockSize; j++ )
            ofs[i*blockSize + j] = (int)(i*step + j);

    for( ptidx = 0; ptidx < ptsize; ptidx++ )
    {
        int x0 = cvRound(pts[ptidx].pt.x - r);
        int y0 = cvRound(pts[ptidx].pt.y - r);

        const uchar* ptr0 = ptr00 + y0*step + x0;
        int a = 0, b = 0, c = 0;

        for( int k = 0; k < blockSize*blockSize; k++ )
        {
            const uchar* ptr = ptr0 + ofs[k];
			// this is sobel
//            int Ix = (ptr[1] - ptr[-1])*2 + (ptr[-step+1] - ptr[-step-1]) + (ptr[step+1] - ptr[step-1]);
//            int Iy = (ptr[step] - ptr[-step])*2 + (ptr[step-1] - ptr[-step-1]) + (ptr[step+1] - ptr[-step+1]);
			// this is Scharr
            int Ix = (ptr[1] - ptr[-1])*10 + (ptr[-step+1] - ptr[-step-1])*3 + (ptr[step+1] - ptr[step-1])*3;
            int Iy = (ptr[step] - ptr[-step])*10 + (ptr[step-1] - ptr[-step-1])*3 + (ptr[step+1] - ptr[-step+1])*3;
            a += Ix*Ix;
            b += Iy*Iy;
            c += Ix*Iy;
        }

		double  m00 = 0.5*a*scale_sq;
		double  m01 =     c*scale_sq;
		double m11 = 0.5*b*scale_sq;
        pts[ptidx].response = m00 + m11 - sqrt( (m00-m11)*(m00-m11) + m01*m01);
		
//cout << pts[ptidx].response << endl;
		int chad = 0;
    }
}
//END_INCLUDE(all)
