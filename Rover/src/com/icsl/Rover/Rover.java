package com.icsl.Rover;

import android.app.Activity;
import android.content.ServiceConnection;
import android.content.ComponentName;
// import android.hardware.Camera;
// import android.hardware.Camera.PreviewCallback;
// import android.hardware.Camera.Size;
import android.graphics.BitmapFactory;
import android.graphics.BitmapFactory.Options;
import android.graphics.ImageFormat;
import android.media.CamcorderProfile;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.os.Environment;
import android.os.IBinder;
import android.view.View;
import android.view.SurfaceView;
import android.view.SurfaceHolder;
import android.widget.ImageView;
import android.content.Intent;
import android.widget.EditText;
import android.widget.TextView;
import android.graphics.Bitmap;
import android.util.Log;
import android.content.BroadcastReceiver;
import android.content.IntentFilter;
import android.os.ParcelFileDescriptor;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.content.Context;
import android.app.PendingIntent;

import java.lang.String;
import java.lang.Integer;
import java.io.File;
import java.io.FileFilter;
import java.util.regex.Pattern;
import java.util.List;
import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Rover extends Activity implements Runnable
{
	public final static String EXTRA_MESSAGE = "com.icsl.Rover.MESSAGE";
	private static final String ME = "Rover";

	private Mat mImage;
	private Bitmap mBitmap;
	private boolean mThreadRun, mThreadIsDone, mOpenCVManagerConnected;

	private ImageView mIvImageDisplay;
	private TextView mTvGyro, mTvAccel, mTvMag, mTvImgProcTime;
	private TextView mTvRoll, mTvPitch, mTvYaw;

	RoverService mService = null;
	boolean mBound = false;

//	Camera mCamera = null;
//	MediaRecorder mMediaRecorder = null;
//	long mLastPreviewTimeNS = 0;
//	Mat mImgYUV, mImgRGB;
//	double mAvgProcTime = 0;
//	int mImgProcCnt = 0;
//	double mAvgDT = 0;
//	byte[] mImgBuffer;
	
	// For arduino comm
	private static final String ACTION_USB_PERMISSION = "com.google.example.USB_PERMISSION";
	UsbAccessory mAccessory;
	private UsbManager mUsbManager;
	private PendingIntent mPermissionIntent;
	private boolean mPermissionRequestPending;
	ParcelFileDescriptor mFileDescriptor = null;
	FileInputStream mInputStream = null;
	FileOutputStream mOutputStream = null;

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
		mThreadIsDone = true;
		mOpenCVManagerConnected = false;

		mIvImageDisplay = (ImageView)findViewById(R.id.ivImageDisplay);

		mTvGyro = (TextView)findViewById(R.id.tvGyroValue);
		mTvAccel= (TextView)findViewById(R.id.tvAccelValue);
		mTvMag= (TextView)findViewById(R.id.tvMagValue);
		mTvImgProcTime= (TextView)findViewById(R.id.tvImgProcTime);

		mTvRoll = (TextView)findViewById(R.id.tvRollAngle);
		mTvPitch= (TextView)findViewById(R.id.tvPitchAngle);
		mTvYaw= (TextView)findViewById(R.id.tvYawAngle);

        mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
		mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION), 0);
		IntentFilter filter = new IntentFilter(ACTION_USB_PERMISSION);
		filter.addAction(UsbManager.ACTION_USB_ACCESSORY_DETACHED);
		registerReceiver(mUsbReceiver, filter);
    }

    @Override
	public void onResume()
	{
		super.onResume();
		mBitmap = Bitmap.createBitmap(320, 240, Bitmap.Config.ARGB_8888);
		mIvImageDisplay.setImageBitmap(mBitmap);

		if (mInputStream != null && mOutputStream != null)
			return;
 
		UsbAccessory[] accessories = mUsbManager.getAccessoryList();
		UsbAccessory accessory = (accessories == null ? null : accessories[0]);
		if (accessory != null)
		{
			if (mUsbManager.hasPermission(accessory))
			{
				mAccessory = accessory;
				if(mService != null)
					mService.openAccessory(accessory);
			}
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
	}

    @Override
	public void onPause()
	{
		mThreadRun = false;
		while(!mThreadIsDone)
		{

			try{
				Thread.sleep(100);
			} catch(Exception e){
				Log.e(ME,"Caught sleeping 2");}
		}

		if(mImage != null)
			mImage.release();
		mImage = null;

		// There seems be a thread issue where android is still trying to draw the bitmap during
		// shutdown after I've already recycled mBitmap. So set the image to null before recycling.
		mIvImageDisplay.setImageBitmap(null);
		if(mBitmap != null)
			mBitmap.recycle();
		mBitmap = null;

		super.onPause();

		Log.i(ME,"Java paused");
	}

	@Override
	public void onStop()
	{
		super.onStop();
		if(mBound)
		{
			if(mService != null)
				mService.closeAccessory();
			unbindService(mConnection);
			mBound = false;
		}
		Log.i(ME,"Java stopped");
	}

	public void onBtnStartService_clicked(View v)
	{
		Intent roverServiceIntent = new Intent(Rover.this, RoverService.class);
		roverServiceIntent.putExtra("com.icsl.Rover.USB_ACCESSORY", mAccessory);
		startService(roverServiceIntent);
		bindService(roverServiceIntent, mConnection, BIND_AUTO_CREATE);
	}

	public void onBtnStopService_clicked(View v)
	{
		if(mBound)
		{
			unbindService(mConnection);
			mBound = false;
		}
		Intent roverServiceIntent = new Intent(Rover.this, RoverService.class);
		stopService(roverServiceIntent);
	}

	public void run()
	{
		Log.i(ME, "Java runner started");
		mThreadIsDone = false;
		mThreadRun = true;
		while(mThreadRun)
		{
			if(mBound && mService != null && !mService.isConnectedToPC())
			{
				mBitmap = mService.getImage();

				float gyro[] = mService.getRoverGyroValue();
				float accel[] = mService.getRoverAccelValue();
				float mag[] = mService.getRoverMagValue();
				float att[] = mService.getRoverAttitude();
				int imgProcTimeMS = mService.getRoverImageProcTimeMS();

				final String strGyro, strAccel, strMag, strImgProcTime, strRoll, strPitch, strYaw;
				if(gyro.length > 2)
					strGyro = String.format("Gyro:\t\t%1.2f\t\t%1.2f\t\t%1.2f",gyro[0],gyro[1],gyro[2]);
				else
					strGyro = "Gyro: n/a";
				if(accel.length > 2)
					strAccel = String.format("Accel:\t\t%1.2f\t\t%1.2f\t\t%1.2f",accel[0],accel[1],accel[2]);
				else
					strAccel = "Accel: n/a";
				if(mag.length > 2)
					strMag = String.format("Mag:\t\t%1.2f\t\t%1.2f\t\t%1.2f",mag[0],mag[1],mag[2]);
				else
					strMag = "Mag: n/a";
				strImgProcTime = "Proc Time: "+String.valueOf(imgProcTimeMS)+"ms";
				if(att.length > 2)
					strRoll = String.format("Roll:\t%1.3f",att[0]);
				else
					strRoll = "Roll: n/a";
				if(att.length > 2)
					strPitch = String.format("Pitch:\t%1.3f",att[1]);
				else
					strPitch = "Pitch: n/a";
				if(att.length > 2)
					strYaw = String.format("Yaw:\t%1.3f",att[2]);
				else
					strYaw = "Yaw: n/a";

				// Running stuff on the UI thread seems to interfere
				// with the camera acquistion rate
				// For now this is ok since it only happens when I'm not connected to the PC
				// and, hence, don't care about flight performacne
				runOnUiThread(new Runnable(){
					public void run(){ 
//						mTvGyro.setText(strGyro);
//						mTvAccel.setText(strAccel);
//						mTvMag.setText(strMag);
//						mTvImgProcTime.setText(strImgProcTime);
//						mTvRoll.setText(strRoll);
//						mTvPitch.setText(strPitch);
//						mTvYaw.setText(strYaw);

						if(mBitmap != null)
							mIvImageDisplay.setImageBitmap(mBitmap); 
					}
				});
			}

			try{
				Thread.sleep(100);
			} catch(Exception e){
				Log.e(ME,"Caught sleeping");
			}
		}

		Log.i(ME,"Java Runner dead *************************");
		mThreadIsDone = true;
	}

	private ServiceConnection mConnection = new ServiceConnection() {
		@Override
		public void onServiceConnected(ComponentName className, IBinder service) {
			RoverService.RoverBinder binder = (RoverService.RoverBinder) service;
			mService = binder.getService();
			if(mAccessory != null)
				mService.openAccessory(mAccessory);
			mBound = true;
			Log.i(ME, "Bound to Rover service");
		}

//		@Override
//		public void onServiceDisconnected(ComponentName arg0)
//		{ mBound = false; Log.i(ME, "Unbound from Rover service"); }
		
		@Override
		public void onServiceDisconnected(ComponentName arg0) {
			mBound = false;
			Log.i(ME, "Unbound from Rover service");
		}
	};

	// for arduino comm
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
					{
						mAccessory = accessory;
						if(mService != null)
							mService.openAccessory(accessory);
					}
					else
						Log.d(ME, "permission denied for accessory " + accessory);
					mPermissionRequestPending = false;
				}
			}
			else if (UsbManager.ACTION_USB_ACCESSORY_DETACHED.equals(action))
			{
				Log.i(ME,"Got the detached notification");
				UsbAccessory accessory = (UsbAccessory)intent.getParcelableExtra(UsbManager.EXTRA_ACCESSORY);
				if (accessory != null && accessory.equals(mAccessory))
				{
					mAccessory = null;
					if(mService != null)
						mService.closeAccessory();
				}
			}
		}
	};

	static{
		System.loadLibrary("opencv_java");
	}
}

