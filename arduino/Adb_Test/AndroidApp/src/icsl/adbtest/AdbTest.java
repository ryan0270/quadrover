package com.icsl.adbtest;

import android.content.Context;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.ToggleButton;

import android.app.Activity;
import android.util.Log;
import android.widget.ScrollView;
import android.widget.TextView;

public class AdbTest extends Activity implements Runnable {

    private final String ME = "AdbTest";

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
		jniShutdown();
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

		(new Thread(this)).start();
		jniInit();
    }

	@Override
	public void onDestroy() {
		super.onDestroy();
	}

	boolean mIsThreadDone = true;
	boolean mDoThreadRun = false;
	public void run()
	{
		Log.i(ME,"Starting runner");
		mIsThreadDone = false;
		mDoThreadRun = true;
		int timeoutMS = 500;
		boolean blinkOn = false;
		int[] curVals = new int[4];
		curVals[0] = 0;
		curVals[1] = 1;
		curVals[2] = 2;
		curVals[3] = 3;
		while(mDoThreadRun)
		{
		}
		
		mIsThreadDone = true;
	}

	public void updateDisplay(final float val)
	{
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				mDumpTextView.append( String.valueOf(val) + "\n");
				mScrollView.smoothScrollTo(0, mDumpTextView.getBottom());
			}
		});
	}

	public byte[] int2ByteArray(int val, int numBytes)
	{
		byte[] chad = new byte[numBytes];
		for(int i=0; i<numBytes; i++)
			chad[numBytes-i-1] = (byte)(val >> i*8);

		return chad;
	}

	// I don't use this so it's not well tested
	public byte[] float2ByteArray(float val)
	{
		int valBytes = Float.floatToIntBits(val);
		byte[] chad = new byte[4];
		chad[0] = (byte)(valBytes >> 3*8);
		chad[1] = (byte)(valBytes >> 2*8);
		chad[2] = (byte)(valBytes >> 1*8);
		chad[3] = (byte)(valBytes >> 0*8);

		return chad;
	}

	public native void jniInit();
	public native void jniShutdown();

	static{
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("adbTest");
	}

}
