#include <jni.h>
#include <errno.h>
#include <EGL/egl.h>
#include <GLES/gl.h>

#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Time.h"

#include "ActiveObject.h"
#include "funcs.h"

#include <android/sensor.h>
#include <android/log.h>
#include <native_app_glue/android_native_app_glue.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "VisionTestApp", __VA_ARGS__))
#define LOGW(...) ((void)__android_log_print(ANDROID_LOG_WARN, "VisionTestApp", __VA_ARGS__))


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
void android_main(struct android_app* state)
{
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
	using namespace TNT;
	using namespace std;
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
	string dataDir;
	int dataSet = 0;
	int startImg=0, endImg=0;
	switch(dataSet)
	{
		case 0:
			dataDir = "/sdcard/TargetFindTest/dataSets/Sep19";
			startImg = 1360;
			endImg = 2600;
			endImg = startImg+200;
			break;
	}

	string imgDir;
	imgDir = dataDir + "/video";

	vector<pair<int, Time>> imgIdList;
	// preload all images
	list<pair<int, shared_ptr<cv::Mat>>> imgList;
	int imgId = startImg;
	int numImages;
	numImages = endImg-startImg;;
	Log::alert("Loading images");
	for(int i=0; i<numImages; i++)
	{
		cv::Mat img;
		while(img.data == NULL)
		{
			stringstream ss;
			ss << "image_" << ++imgId << ".bmp";
			img = cv::imread(imgDir+"/"+ss.str());
		}

		shared_ptr<cv::Mat> pImg(new cv::Mat);
		img.copyTo(*pImg);

		imgList.push_back(pair<int, shared_ptr<cv::Mat>>(imgId, pImg));
	}
	Log::alert("done");

	// Camera calibration
	cv::Point2f center;
	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;
	cv::FileStorage fs;
	string filename = dataDir + "/s3Calib_640x480.yml";
	fs.open(filename.c_str(), cv::FileStorage::READ);
	if( fs.isOpened() )
	{
		mCameraMatrix_640x480 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraDistortionCoeffs = shared_ptr<cv::Mat>(new cv::Mat());

		fs["camera_matrix"] >> *mCameraMatrix_640x480;
		fs["distortion_coefficients"] >> *mCameraDistortionCoeffs;
		Log::alert(String()+"Camera calib loaded from " + filename.c_str());

		mCameraMatrix_320x240 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraMatrix_640x480->copyTo( *mCameraMatrix_320x240 );
		(*mCameraMatrix_320x240) = (*mCameraMatrix_320x240)*0.5;

		center.x = mCameraMatrix_320x240->at<double>(0,2);
		center.y = mCameraMatrix_320x240->at<double>(1,2);
	}
	else
	Log::alert(String()+"Failed to open " + filename.c_str());
	fs.release();

	vector<shared_ptr<ActiveObject>> activeObjects;

	Array2D<double> Sn = 10*10*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0];
	SnInv[1][1] = 1.0/Sn[1][1];
	double varxi = pow(500,2);
	double probNoCorr = 0.0000001;

	list<pair<int, shared_ptr<cv::Mat>>>::const_iterator imgIter = imgList.begin();
	cv::Mat img(240,320, CV_8UC3), oldImg(240,320,CV_8UC3);
	TimeKeeper::times.resize(100);
	for(int i=0; i<TimeKeeper::times.size(); i++)
		TimeKeeper::times[i] = 0;
	int activeCnt = 0;
	int imgCnt = 0;
	Time curTime;
	Log::alert("Starting run");
	while(imgIter != imgList.end())
	{
Log::alert("--------------------------------------------------");
Log::alert(String()+"imgCnt: "+imgCnt);
		curTime.addTimeMS(33);
		img = *(imgIter->second);

Time start;
		vector<vector<cv::Point>> allContours = findContours(img);
Log::alert(String()+"numContours: "+allContours.size());

TimeKeeper::times[0] += start.getElapsedTimeNS()/1.0e6; start.setTime();
		
		vector<shared_ptr<ActiveObject>> curObjects = objectify(allContours,Sn,SnInv,varxi,probNoCorr,curTime);
Log::alert(String()+"numObjects: "+curObjects.size());

TimeKeeper::times[1] += start.getElapsedTimeNS()/1.0e6; start.setTime();
		/////////////////// Get location priors for active objects ///////////////////////
		Array2D<double> mv(3,1,0.0);
		Array2D<double> Sv = 0.2*0.2*createIdentity((double)3);
		double mz = 1;
		double sz = 0.05;
		double f = mCameraMatrix_640x480->at<double>(0,0);
		Array2D<double> omega(3,1,0.0);

		for(int i=0; i<activeObjects.size(); i++)
		{
			shared_ptr<ActiveObject> ao = activeObjects[i];
			ao->updatePosition(mv, Sv, mz, sz*sz, f, center, omega, curTime);
		}

TimeKeeper::times[2] += start.getElapsedTimeNS()/1.0e6; start.setTime();
		/////////////////// make matches ///////////////////////
		vector<Match> goodMatches;
		vector<shared_ptr<ActiveObject>> repeatObjects;
		matchify(activeObjects, curObjects, goodMatches, repeatObjects, Sn, SnInv, varxi, probNoCorr, curTime);
		activeCnt += activeObjects.size();
Log::alert(String()+"activeCnt: "+activeCnt);

TimeKeeper::times[3] += start.getElapsedTimeNS()/1.0e6; start.setTime();

		for(int i=0; i<activeObjects.size(); i++)
		{
			shared_ptr<ActiveObject> ao = activeObjects[i];
			String str;
			str = str+ao->id+": \t";
			str = str+ao->lastCenter.x+"x"+ao->lastCenter.y+"\t";
			str = str+ao->mom.m00+"\t";
			str = str+ao->life;
			Log::alert(str);
		}

		imgIter++;
		imgCnt++;
	}

	for(int i=0; i<TimeKeeper::times.size(); i++)
		if(TimeKeeper::times[i] != 0)
			Log::alert(String()+"t" + i +":\t" + (TimeKeeper::times[i]/imgCnt));
	Log::alert(String()+"avg algo time: " + TimeKeeper::sum(0,4)/imgCnt);
	Log::alert(String()+"num objects:\t" + activeCnt/imgCnt);

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
