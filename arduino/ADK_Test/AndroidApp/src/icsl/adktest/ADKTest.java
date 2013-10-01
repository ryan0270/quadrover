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

    private final String ME = "ADK_Test";

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
						Log.d(ME, "permission denied for accessory " + accessory);
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
		jniShutdown();
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
			Log.d(ME, "mAccessory is null");

		(new Thread(this)).start();
		jniInit();
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
			Log.d(ME, "accessory opened");
		}
		else
			Log.d(ME, "accessory open fail");
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
//			if(sendMotorCommands(curVals))
//			{
//				for(int i=0; i<curVals.length; i++)
//					curVals[i]++;
//			}
			try{ Thread.sleep(500); }
			catch(Exception e){ Log.e(ME,"Caught sleeping"); }
		}
		
		mIsThreadDone = true;
	}

	public boolean sendMotorCommands(int cmd0, int cmd1, int cmd2, int cmd3)
	{
		if(mOutputStream == null)
			return false;

		byte[] b0 = int2ByteArray(cmd0,2);
		byte[] b1 = int2ByteArray(cmd1,2);
		byte[] b2 = int2ByteArray(cmd2,2);
		byte[] b3 = int2ByteArray(cmd3,2);

		byte[] buff = new byte[8];
		buff[0] = b0[0];
		buff[1] = b0[1];
		buff[2] = b1[0];
		buff[3] = b1[1];
		buff[4] = b2[0];
		buff[5] = b2[1];
		buff[6] = b3[0];
		buff[7] = b3[1];

		boolean success = false;
		try
		{
			mOutputStream.write(buff);
			success = true;
		}
		catch (IOException e)
		{ Log.e(ME, "write failed: ", e); }

		return success;
	}

	public byte[] int2ByteArray(int val, int numBytes)
	{
		byte[] chad = new byte[numBytes];
		for(int i=0; i<numBytes; i++)
			chad[i] = (byte)(val >> i*8);

		return chad;
	}

	// I don't use this so it's not well tested
	public byte[] float2ByteArray(float val)
	{
		int valBytes = Float.floatToIntBits(val);
		byte[] chad = new byte[4];
		chad[3] = (byte)(valBytes >> 3*8);
		chad[2] = (byte)(valBytes >> 2*8);
		chad[1] = (byte)(valBytes >> 1*8);
		chad[0] = (byte)(valBytes >> 0*8);

		return chad;
	}

//	public native void doChad(float val);
	public native void jniInit();
	public native void jniShutdown();

	static{
		System.loadLibrary("adkTest");
	}

}
