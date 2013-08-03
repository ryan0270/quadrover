/* Copyright 2011 Google Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 * Project home page: http://code.google.com/p/usb-serial-for-android/
 */

package com.icsl.serialtest;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.widget.ScrollView;
import android.widget.TextView;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.util.HexDump;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.nio.ByteBuffer;

public class TestActivity extends Activity implements Runnable {

    private final String TAG = TestActivity.class.getSimpleName();

    /**
     * Driver instance, passed in statically via
     * {@link #show(Context, UsbSerialDriver)}.
     *
     * <p/>
     * This is a devious hack; it'd be cleaner to re-create the driver using
     * arguments passed in with the {@link #startActivity(Intent)} intent. We
     * can get away with it because both activities will run in the same
     * process, and this is a simple demo.
     */
    private static UsbSerialDriver mDriver = null;

    private TextView mTitleTextView;
    private TextView mDumpTextView;
    private ScrollView mScrollView;

    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();

    private SerialInputOutputManager mSerialIoManager;

	private float mNewFloat;
	private boolean mIsNewDataReady = false;
    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {

        @Override
        public void onRunError(Exception e) {
            Log.d(TAG, "Runner stopped.");
        }

		private ByteBuffer msgBuffer = ByteBuffer.allocate(128);
        @Override
        public void onNewData(final byte[] data) {
			msgBuffer.put(data);
			if(data[data.length-1] == 0x0A)
			{
				mNewFloat = Float.parseFloat(new String(msgBuffer.array()));
				msgBuffer = ByteBuffer.allocate(128);
				mIsNewDataReady = true;

				final float val = mNewFloat;
				TestActivity.this.runOnUiThread(new Runnable() {
					@Override
					public void run() {
						if(data[data.length-1] == 0x0A)
						{
							mDumpTextView.append( String.valueOf(val) + "\n");
							mScrollView.smoothScrollTo(0, mDumpTextView.getBottom());
						}
					}
				});
			}
		}
    };

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
        stopIoManager();
        if (mDriver != null) {
            try {
                mDriver.close();
            } catch (IOException e) {
                // Ignore.
            }
            mDriver = null;
        }
		shutdown();
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();
        Log.d(TAG, "Resumed, mDriver=" + mDriver);
        if (mDriver == null) {
            mTitleTextView.setText("No serial device.");
        } else {
            try {
                mDriver.open();
                mDriver.setParameters(115200, 8, UsbSerialDriver.STOPBITS_1, UsbSerialDriver.PARITY_NONE);
            } catch (IOException e) {
                Log.e(TAG, "Error setting up device: " + e.getMessage(), e);
                mTitleTextView.setText("Error opening device: " + e.getMessage());
                try {
                    mDriver.close();
                } catch (IOException e2) {
                    // Ignore.
                }
                mDriver = null;
                return;
            }
            mTitleTextView.setText("Serial device: " + mDriver.getClass().getSimpleName());
			(new Thread(this)).start();
        }
        onDeviceStateChange();
		jniInit();
    }

	boolean mIsThreadDone = true;
	boolean mDoThreadRun = false;
	public void run()
	{
		Log.i(TAG,"Starting runner");
		mIsThreadDone = false;
		mDoThreadRun = true;
		ByteBuffer msgBuffer = ByteBuffer.allocate(128);
		int timeoutMS = 500;
		while(mDoThreadRun)
		{
			if(mIsNewDataReady && mDriver != null)
			{
				float val = mNewFloat;
				doChad(val);
				mIsNewDataReady = false;
			}

			try{ Thread.sleep(10); }
			catch(Exception e){ Log.e(TAG,"Caught sleeping"); }
		}
		
		mIsThreadDone = true;
	}

	public int sendFloat(float val, int timeoutMS)
	{
		int valBytes = Float.floatToIntBits(val);
		byte[] chad = new byte[4];
//		chad[4] = (byte)0xFF; // terminate character
		chad[3] = (byte)(valBytes >> 3*8);
		chad[2] = (byte)(valBytes >> 2*8);
		chad[1] = (byte)(valBytes >> 1*8);
		chad[0] = (byte)(valBytes >> 0*8);

		int numBytesSent = 0;
		try
		{ numBytesSent = mDriver.write(chad, timeoutMS); }
		catch(Exception ex){ numBytesSent = -1;};

		return numBytesSent;
	}

    private void stopIoManager() {
        if (mSerialIoManager != null) {
            Log.i(TAG, "Stopping io manager ..");
            mSerialIoManager.stop();
            mSerialIoManager = null;
        }
    }

    private void startIoManager() {
        if (mDriver != null) {
            Log.i(TAG, "Starting io manager ..");
            mSerialIoManager = new SerialInputOutputManager(mDriver, mListener);
            mExecutor.submit(mSerialIoManager);
        }
    }

    private void onDeviceStateChange() {
        stopIoManager();
        startIoManager();
    }

	public native void doChad(float val);
	public native void jniInit();
	public native void shutdown();

    /**
     * Starts the activity, using the supplied driver instance.
     *
     * @param context
     * @param driver
     */
    static void show(Context context, UsbSerialDriver driver) {
        mDriver = driver;
        final Intent intent = new Intent(context, TestActivity.class);
        intent.addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP | Intent.FLAG_ACTIVITY_NO_HISTORY);
        context.startActivity(intent);
    }

	static{
		System.loadLibrary("SerialTest");
	}

}
