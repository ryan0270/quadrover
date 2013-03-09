package com.icsl.Rover;

import android.app.Activity;
import android.content.ServiceConnection;
import android.content.ComponentName;
import android.os.Bundle;
import android.os.Environment;
import android.os.IBinder;
import android.view.View;
import android.widget.ImageView;
import android.content.Intent;
import android.widget.EditText;
import android.widget.TextView;
import android.graphics.Bitmap;
import android.util.Log;

import java.lang.String;
import java.lang.Integer;
import java.io.File;
import java.io.FileFilter;
import java.util.regex.Pattern;

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

	RoverService mService;
	boolean mBound = false;

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

    }

    @Override
	public void onResume()
	{
		super.onResume();
		mBitmap = Bitmap.createBitmap(320, 240, Bitmap.Config.ARGB_8888);
		mIvImageDisplay.setImageBitmap(mBitmap);

//		onJNIStart();
//
//		mImage = new Mat(240,320,CvType.CV_8UC3,new Scalar(127));
//		File logDir = new File(Environment.getExternalStorageDirectory().toString()+"/"+ME);
//		if(!logDir.exists())
//		{
//			Log.i(ME,"Log dir: "+logDir.toString()+" does not exist. Creating it.");
//			logDir.mkdir();
//		}
//		else
//			Log.i(ME,"Log dir: "+logDir.toString()+" already exists.");
//		File imgLogDir = new File(logDir.getAbsolutePath()+"/images");
//		if(imgLogDir.exists())
//		{
//			File bakDir = new File(imgLogDir.getAbsolutePath()+"_bak");
//			if(bakDir.exists())
//				deleteDir(bakDir);
//			imgLogDir.renameTo(bakDir);
//		}
//		imgLogDir.mkdir();
//		setLogDir(logDir.toString());
//		File logFile = new File(logDir.getAbsolutePath()+"/log.txt");
//		if(logFile.exists())
//		{
//			File bakFile = new File(logFile.getAbsolutePath()+".bak");
//			if(bakFile.exists())
//				bakFile.delete();
//			logFile.renameTo(bakFile);
//		}
//		startLogging();

//		populateVisionParams();

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

//		if (mOpenCVManagerConnected)
//			onJNIStop();


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
			unbindService(mConnection);
			mBound = false;
		}
		Log.i(ME,"Java stopped");
	}

	public void onBtnStartService_clicked(View v)
	{
		Intent roverServiceIntent = new Intent(Rover.this, RoverService.class);
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
//		toggleViewType();
	}

	public void run()
	{
		Log.i(ME, "Java runner started");
		long startTime = System.nanoTime();
		mThreadIsDone = false;
		mThreadRun = true;
		while(mThreadRun)
		{
			if(mBound)
			{
				mBitmap = mService.getImage();

				final float gyro[] = mService.getRoverGyroValue();
				final float accel[] = mService.getRoverAccelValue();
				final float mag[] = mService.getRoverMagValue();
				final float att[] = mService.getRoverAttitude();
				final int imgProcTimeMS = mService.getRoverImageProcTimeMS();
				runOnUiThread(new Runnable(){
					public void run(){ 
						mTvGyro.setText(String.format("Gyro:\t\t%1.2f\t\t%1.2f\t\t%1.2f",gyro[0],gyro[1],gyro[2]));
						mTvAccel.setText(String.format("Accel:\t\t%1.2f\t\t%1.2f\t\t%1.2f",accel[0],accel[1],accel[2]));
						mTvMag.setText(String.format("Mag:\t\t%1.2f\t\t%1.2f\t\t%1.2f",mag[0],mag[1],mag[2]));
						mTvImgProcTime.setText("Proc Time: "+String.valueOf(imgProcTimeMS)+"ms");
						mTvRoll.setText(String.format("Roll:\t%1.3f",att[0]));
						mTvPitch.setText(String.format("Pitch:\t%1.3f",att[1]));
						mTvYaw.setText(String.format("Yaw:\t%1.3f",att[2]));

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
}

