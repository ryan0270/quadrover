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
		Log.i(ME,"Starting java runner");
		mIsThreadDone = false;
		mDoThreadRun = true;
		while(mDoThreadRun)
		{
			final float val = getSonarValue();
			runOnUiThread(new Runnable() {
				@Override
				public void run() {
					mDumpTextView.append( String.valueOf(val) + "\n");
					mScrollView.smoothScrollTo(0, mDumpTextView.getBottom());
				}
			});

			try{ Thread.sleep(1000); }
			catch(Exception e){};
		}

		Log.i(ME,"Java runner dead");
		
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

	public native void jniInit();
	public native void jniShutdown();
	public native float getSonarValue();

	static{
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("adbTest");
	}

}
