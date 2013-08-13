package com.icsl.motortestapp;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;
import android.util.Log;

import com.hoho.android.usbserial.driver.UsbSerialDriver;
import com.hoho.android.usbserial.util.HexDump;
import com.hoho.android.usbserial.util.SerialInputOutputManager;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.nio.ByteBuffer;

public class MotorTestApp extends Activity
{
	private final static String ME="MotorTestApp";
	private TextView mTxtMotorVal1, mTxtMotorVal2, mTxtMotorVal3, mTxtMotorVal4;
	private SeekBar mSldMotorVal1, mSldMotorVal2, mSldMotorVal3, mSldMotorVal4;

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

		mSldMotorVal1.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener(){
			@Override
			public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser)
			{
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
//		mDoThreadRun = false;
//		while(!mIsThreadDone)
//		{
//			try{ Thread.sleep(10); }
//			catch(Exception e){};
//		}
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
        Log.d(ME, "Resumed, mDriver=" + mDriver);
        if (mDriver == null) {
			mTxtCommStatus = "No device";
        } else {
            try {
                mDriver.open();
                mDriver.setParameters(115200, 8, UsbSerialDriver.STOPBITS_1, UsbSerialDriver.PARITY_NONE);
            } catch (IOException e) {
                Log.e(ME, "Error setting up device: " + e.getMessage(), e);
				mTxtCommStatus.setText("Error opening device: " + e.getMessage());
                try {
                    mDriver.close();
                } catch (IOException e2) {
                    // Ignore.
                }
                mDriver = null;
                return;
            }
			mTxtCommStatus.setText("Connected");
//			(new Thread(this)).start();
        }
        onDeviceStateChange();
//		jniInit();
    }

	// This is what receives the actual events from the arduino
    private final SerialInputOutputManager.Listener mListener =
            new SerialInputOutputManager.Listener() {

        @Override
        public void onRunError(Exception e) {
            Log.d(ME, "Runner stopped.");
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

	public int sendFloat(float val, int timeoutMS)
	{
		int valBytes = Float.floatToIntBits(val);
		byte[] chad = new byte[4];
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

	// BEWARE: on some arduino's int is a 16-bit type
	public int sendInt(int val, int timeoutMS)
	{
		byte[] chad = ByteBuffer.allocate(4).putInt(val).array();

		int numBytesSent = 0;
		try
		{ numBytesSent = mDriver.write(chad, timeoutMS); }
		catch(Exception ex){ numBytesSent = -1;};

		return numBytesSent;
	}
}
