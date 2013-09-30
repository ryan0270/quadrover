package com.icsl.adktest;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.widget.ScrollView;
import android.widget.TextView;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.SystemClock;
import java.util.ArrayList;
import java.util.List;


public class TestActivity extends Activity implements Runnable {

    private final String TAG = "ADK_Test";

    private TextView mTitleTextView;
    private TextView mDumpTextView;
    private ScrollView mScrollView;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.serial_console);
        mTitleTextView = (TextView) findViewById(R.id.demoTitle);
        mDumpTextView = (TextView) findViewById(R.id.consoleText);
        mScrollView = (ScrollView) findViewById(R.id.demoScroller);
    }

    @Override
    protected void onPause() {
        super.onPause();
		mDoThreadRun = false;
		while(!mIsThreadDone)
		{
			try{ Thread.sleep(10); }
			catch(Exception e){};
		}
//		shutdown();
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

//		jniInit();
    }

	boolean mIsThreadDone = true;
	boolean mDoThreadRun = false;
	public void run()
	{
		Log.i(TAG,"Starting runner");
		mIsThreadDone = false;
		mDoThreadRun = true;
		int timeoutMS = 500;
		while(mDoThreadRun)
		{
//			if(mIsNewDataReady && mDriver != null)
//			{
//				float val = mNewFloat;
//				doChad(val);
//				mIsNewDataReady = false;
//			}
//
//			try{ Thread.sleep(10); }
//			catch(Exception e){ Log.e(TAG,"Caught sleeping"); }
		}
		
		mIsThreadDone = true;
	}


//	public native void doChad(float val);
//	public native void jniInit();
//	public native void shutdown();
//
//	static{
//		System.loadLibrary("SerialTest");
//	}

}
