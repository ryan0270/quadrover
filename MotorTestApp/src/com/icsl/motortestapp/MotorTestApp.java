package com.icsl.motortestapp;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.os.SystemClock;
import android.widget.SeekBar;
import android.widget.TextView;
import android.util.Log;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.driver.UsbSerialProber;
import com.hoho.android.usbserial.util.HexDump;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

public class MotorTestApp extends Activity implements Runnable
{
	private final static String ME="MotorTestApp";
	private TextView mTxtCommStatus;
	private TextView mTxtMotorVal1, mTxtMotorVal2, mTxtMotorVal3, mTxtMotorVal4;
	private SeekBar mSldMotorVal1, mSldMotorVal2, mSldMotorVal3, mSldMotorVal4;

	boolean mIsThreadDone = true;
	boolean mDoThreadRun = false;

	int mMotorVals[] = new int[4];

	// for usb comm with arduino
    private static UsbSerialDriver mDriver = null;
    private final ExecutorService mExecutor = Executors.newSingleThreadExecutor();
    private SerialInputOutputManager mSerialIoManager;
	private float mNewFloat;
	private boolean mIsNewDataReady = false;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

		mTxtCommStatus = (TextView)findViewById(R.id.txtCommStatus);

		mTxtMotorVal1 = (TextView)findViewById(R.id.txtMotorVal1);
		mTxtMotorVal2 = (TextView)findViewById(R.id.txtMotorVal2);
		mTxtMotorVal3 = (TextView)findViewById(R.id.txtMotorVal3);
		mTxtMotorVal4 = (TextView)findViewById(R.id.txtMotorVal4);

		mSldMotorVal1 = (SeekBar)findViewById(R.id.sldMotorVal1);
		mSldMotorVal2 = (SeekBar)findViewById(R.id.sldMotorVal2);
		mSldMotorVal3 = (SeekBar)findViewById(R.id.sldMotorVal3);
		mSldMotorVal4 = (SeekBar)findViewById(R.id.sldMotorVal4);

		mSldMotorVal1.setMax( 1<<11 );
		mSldMotorVal2.setMax( 1<<11 );
		mSldMotorVal3.setMax( 1<<11 );
		mSldMotorVal4.setMax( 1<<11 );

		for(int i=0; i<4; i++)
			mMotorVals[i] = 0;

		mSldMotorVal1.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener(){
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
			{
				mMotorVals[0] = progress;
				final int val = progress;
				runOnUiThread(new Runnable(){
					public void run(){
						mTxtMotorVal1.setText(String.valueOf(val));
					}
				});
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar){};

			@Override
			public void onStopTrackingTouch(SeekBar seekbar){};
		});

		mSldMotorVal2.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener(){
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
			{
				mMotorVals[1] = progress;
				final int val = progress;
				runOnUiThread(new Runnable(){
					public void run(){
						mTxtMotorVal2.setText(String.valueOf(val));
					}
				});
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar){};

			@Override
			public void onStopTrackingTouch(SeekBar seekbar){};
		});

		mSldMotorVal3.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener(){
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
			{
				mMotorVals[2] = progress;
				final int val = progress;
				runOnUiThread(new Runnable(){
					public void run(){
						mTxtMotorVal3.setText(String.valueOf(val));
					}
				});
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar){};

			@Override
			public void onStopTrackingTouch(SeekBar seekbar){};
		});

		mSldMotorVal4.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener(){
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
			{
				mMotorVals[3] = progress;
				final int val = progress;
				runOnUiThread(new Runnable(){
					public void run(){
						mTxtMotorVal4.setText(String.valueOf(val));
					}
				});
			}

			@Override
			public void onStartTrackingTouch(SeekBar seekBar){};

			@Override
			public void onStopTrackingTouch(SeekBar seekbar){};
		});
    }

	@Override
    protected void onPause() {
        super.onPause();
		mDoThreadRun = false;
		while(!mIsThreadDone)
		{
			Log.i(ME,"Waiting");
			SystemClock.sleep(10);
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
//		shutdown();
        finish();
    }

    @Override
    protected void onResume() {
        super.onResume();

//		SystemClock.sleep(1000);

        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
		// a bit hackish
		// For now I'm assuming just one device attached
		for (final UsbDevice device : manager.getDeviceList().values())
		{
			final List<UsbSerialDriver> drivers = UsbSerialProber.probeSingleDevice(manager, device);
			Log.d(ME, "Found usb device: " + device);
			if (drivers.isEmpty())
				Log.d(ME, "  - No UsbSerialDriver available.");
			else
				mDriver = drivers.get(0);
		}

        Log.d(ME, "Resumed, mDriver=" + mDriver);
        if (mDriver == null)
		{
			mTxtCommStatus.setText("No device");
        }
		else
		{
            try
			{
                mDriver.open();
                mDriver.setParameters(115200, 8, UsbSerialDriver.STOPBITS_1, UsbSerialDriver.PARITY_NONE);
            } 
			catch (IOException e) 
			{
                Log.e(ME, "Error setting up device: " + e.getMessage(), e);
				mTxtCommStatus.setText("Error opening device: " + e.getMessage());
                try 
				{ mDriver.close(); } 
				catch (IOException e2) { }
                mDriver = null;
                return;
            }
			(new Thread(this)).start();
        }
        onDeviceStateChange();
//		jniInit();
    }

	public void run()
	{
		Log.i(ME,"Starting runner");
		mIsThreadDone = false;
		mDoThreadRun = true;
		int timeoutMS = 100;
		
		int commPrefix = 0x00CCBBAA;
//		int commPrefix = 0xFFFFFFFF;
//		commPrefix = commPrefix & 0x000000AA;
//		commPrefix = 170;
		while(mDoThreadRun)
		{
//			runOnUiThread(new Runnable(){
//				public void run(){
//					mTxtMotorVal1.setText(String.valueOf(mMotorVals[0]));
//					mTxtMotorVal2.setText(String.valueOf(mMotorVals[1]));
//					mTxtMotorVal3.setText(String.valueOf(mMotorVals[2]));
//					mTxtMotorVal4.setText(String.valueOf(mMotorVals[3]));
//				}
//			});
			if(mDriver != null)
			{
				sendInt(commPrefix, timeoutMS);
				for(int motorID=0; motorID<4; motorID++)
					sendInt(mMotorVals[motorID], timeoutMS);
//				Log.i(ME,"----------------------------------------");
//				for(int motorID=0; motorID<4; motorID++)
//					Log.i(ME,"Chadding "+String.valueOf(mMotorVals[motorID]));
			}

			SystemClock.sleep(10);
		}
		
		Log.i(ME,"Runner done");
		mIsThreadDone = true;
	}

	// This is what receives the actual events from the arduino
    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {

        @Override
        public void onRunError(Exception e) {
            Log.d(ME, "USB comm error: "+ e.toString());
			stopIoManager();
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
			}
		}
    };

    private void stopIoManager()
	{
        if (mSerialIoManager != null) {
            Log.i(ME, "Stopping io manager ..");
            mSerialIoManager.stop();
            mSerialIoManager = null;
			mTxtCommStatus.setText("Disconnected");
        }
    }

    private void startIoManager()
	{
        if (mDriver != null) {
            Log.i(ME, "Starting io manager ..");
            mSerialIoManager = new SerialInputOutputManager(mDriver, mListener);
            mExecutor.submit(mSerialIoManager);
			mTxtCommStatus.setText("Connected");
        }
    }

    private void onDeviceStateChange()
	{
        stopIoManager();
        startIoManager();
    }

	public int sendCommControlMessage(int val, int timeoutMS)
	{
		byte[] chad = new byte[4];
		chad[3] = (byte)(val >>> 3*8);
		chad[2] = (byte)(val >>> 2*8);
		chad[1] = (byte)(val >>> 1*8);
		chad[0] = (byte)(val >>> 0*8);

		int numBytesSent = 0;
		try
		{ numBytesSent = mDriver.write(chad, timeoutMS); }
		catch(Exception ex){ numBytesSent = -1;};

		return numBytesSent;
	}

	// BEWARE
	// The arduino boards don't all use 4-byte int's
	public int sendInt(int val, int timeoutMS)
	{
		byte[] chad = new byte[4];
		chad[3] = (byte)(val >> 3*8);
		chad[2] = (byte)(val >> 2*8);
		chad[1] = (byte)(val >> 1*8);
		chad[0] = (byte)(val >> 0*8);

		int numBytesSent = 0;
		try
		{ numBytesSent = mDriver.write(chad, timeoutMS); }
		catch(Exception ex){ numBytesSent = -1;};

		return numBytesSent;
	}
}
