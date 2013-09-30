package com.icsl.adktest;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.ParcelFileDescriptor;
import android.util.Log;
import android.view.View;
import android.widget.ToggleButton;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.widget.ScrollView;
import android.widget.TextView;

import android.hardware.usb.UsbDevice;

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.SystemClock;
import java.util.ArrayList;
import java.util.List;

import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;


public class ADKTest extends Activity implements Runnable {

    private final String TAG = "ADK_Test";

	private static final String ACTION_USB_PERMISSION = "com.google.android.DemoKit.action.USB_PERMISSION";

    private TextView mTitleTextView;
    private TextView mDumpTextView;
    private ScrollView mScrollView;

	private UsbManager mUsbManager;
	private PendingIntent mPermissionIntent;
	private boolean mPermissionRequestPending;

	UsbAccessory mAccessory;
	ParcelFileDescriptor mFileDescriptor;
	FileInputStream mInputStream;
	FileOutputStream mOutputStream;

	private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver()
	{
		@Override
		public void onReceive(Context context, Intent intent)
		{
			String action = intent.getAction();
			if (ACTION_USB_PERMISSION.equals(action))
			{
				synchronized (this)
				{
					UsbAccessory accessory = (UsbAccessory)intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
					if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false))
						openAccessory(accessory);
					else
						Log.d(TAG, "permission denied for accessory " + accessory);
					mPermissionRequestPending = false;
				}
			}
			else if (UsbManager.ACTION_USB_ACCESSORY_DETACHED.equals(action))
			{
				UsbAccessory accessory = (UsbAccessory)intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
				if (accessory != null && accessory.equals(mAccessory))
					closeAccessory();
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

        mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
		mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
		IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
		filter.addAction(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
		registerReceiver(mUsbReceiver, filter);
 
//		if (getLastCustomNonConfigurationInstance() != null) {
//			mAccessory = (UsbAccessory) getLastCustomNonConfigurationInstance();
//			openAccessory(mAccessory);
//		}
    }

    @Override
    protected void onPause() {
        super.onPause();
		closeAccessory();

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

		if (mInputStream != null && mOutputStream != null)
			return;
 
		UsbAccessory[] accessories = mUsbManager.getAccessoryList();
		UsbAccessory accessory = (accessories == null ? null : accessories[0]);
		if (accessory != null)
		{
			if (mUsbManager.hasPermission(accessory))
				openAccessory(accessory);
			else
			{
				synchronized (mUsbReceiver)
				{
					if (!mPermissionRequestPending)
					{
						mUsbManager.requestPermission(accessory,mPermissionIntent);
						mPermissionRequestPending = true;
					}
				}
			}
		}
		else
			Log.d(TAG, "mAccessory is null");

//		jniInit();
    }

	@Override
	public void onDestroy() {
		unregisterReceiver(mUsbReceiver);
		super.onDestroy();
	}

	private void openAccessory(UsbAccessory accessory)
	{
		mFileDescriptor = mUsbManager.openAccessory(accessory);
		if (mFileDescriptor != null)
		{
			mAccessory = accessory;
			FileDescriptor fd = mFileDescriptor.getFileDescriptor();
			mInputStream = new FileInputStream(fd);
			mOutputStream = new FileOutputStream(fd);
			Log.d(TAG, "accessory opened");
		}
		else
			Log.d(TAG, "accessory open fail");
	}
 
	private void closeAccessory()
	{
		try
		{
			if (mFileDescriptor != null)
				mFileDescriptor.close();
		}
		catch (IOException e){}
		finally
		{
			mFileDescriptor = null;
			mAccessory = null;
		}
	}

	boolean mIsThreadDone = true;
	boolean mDoThreadRun = false;
	public void run()
	{
		Log.i(TAG,"Starting runner");
		mIsThreadDone = false;
		mDoThreadRun = true;
		int timeoutMS = 500;
		boolean blinkOn = false;
		while(mDoThreadRun)
		{
			if(mOutputStream != null)
			{
				blinkOn = !blinkOn;
				byte[] buff = new byte[1];
				buff[0] = blinkOn ? (byte)1 : (byte)0;

				try
				{ mOutputStream.write(buff); }
				catch (IOException e)
				{ Log.e(TAG, "write failed: ", e); }
			}
//			if(mIsNewDataReady && mDriver != null)
//			{
//				float val = mNewFloat;
//				doChad(val);
//				mIsNewDataReady = false;
//			}
//
			try{ Thread.sleep(500); }
			catch(Exception e){ Log.e(TAG,"Caught sleeping"); }
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
