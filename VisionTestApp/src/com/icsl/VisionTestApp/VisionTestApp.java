package com.icsl.VisionTestApp;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

public class VisionTestApp extends android.app.NativeActivity
{
	private static final String ME = "VisionTestApp";
	private boolean mHasRun = false;

//    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
//        @Override
//        public void onManagerConnected(int status) {
//            switch (status) {
//                case LoaderCallbackInterface.SUCCESS:
//                {
//                    Log.i(ME, "OpenCV loaded successfully");
//                    System.loadLibrary("toadlet_egg");
//					System.loadLibrary("VisionTestApp_native");
//                    Intent intent = new Intent(VisionTestApp.this, android.app.NativeActivity.class);
//                    VisionTestApp.this.startActivity(intent);
//                } break;
//                default:
//                {
//                    super.onManagerConnected(status);
//                } break;
//            }
//        }
//    };
//
//    public VisionTestApp() {
//        Log.i(ME, "Instantiated new " + this.getClass());
//    }
//
//   @Override
//    public void onResume()
//    {
//        super.onResume();
//
//		if(!mHasRun)
//			OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
//
//		mHasRun = true;
//    }

	static
	{
		System.loadLibrary("toadlet_egg");
	}
}
