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

import java.lang.String;
import java.lang.Integer;
import java.io.File;
import java.io.FileFilter;
import java.util.regex.Pattern;
import java.util.List;

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

//	Camera mCamera = null;
//	MediaRecorder mMediaRecorder = null;
//	long mLastPreviewTimeNS = 0;
//	Mat mImgYUV, mImgRGB;
//	double mAvgProcTime = 0;
//	int mImgProcCnt = 0;
//	double mAvgDT = 0;
//	byte[] mImgBuffer;
	

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

//		try{
//			mCamera = Camera.open();
//			Camera.Parameters camParams = mCamera.getParameters();
//			List<Size> previewSizes = camParams.getSupportedPreviewSizes();
//			camParams.setPreviewSize(640,480);
//
////			List<int[]> fpsList = camParams.getSupportedPreviewFpsRange();
////			int[] fps = fpsList.get(fpsList.size()-1);
////			camParams.setPreviewFpsRange((fps[0]), fps[1]);
//			camParams.setPreviewFpsRange(30000, 30000);
//
//			if(camParams.getVideoStabilization())
//			{
//				Log.i(ME, "I have video stabilization");
//				camParams.setVideoStabilization(true);
//			}
//			else
//				Log.i(ME, "I don't have video stabilization");
//
//			Size preferredVideoSize = camParams.getPreferredPreviewSizeForVideo();
//			Log.i(ME, "Preferred video size: "+String.valueOf(preferredVideoSize.width)+"x"+String.valueOf(preferredVideoSize.height));
//
//			camParams.setExposureCompensation( camParams.getMaxExposureCompensation() );
//
//			camParams.setFocusMode( Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO );
//
//			mCamera.setParameters(camParams);
//			SurfaceView dummy = new SurfaceView(getBaseContext());
//			mCamera.setPreviewDisplay(dummy.getHolder());
//
//			mMediaRecorder = new MediaRecorder();
//			mMediaRecorder.setCamera(mCamera);
//			mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
//			mMediaRecorder.setProfile(CamcorderProfile.get(CamcorderProfile.QUALITY_480P));
//			mMediaRecorder.prepare();
//		}
//		catch(Exception e){Log.i(ME, e.toString());}
//
//		Camera.Size imgSize = mCamera.getParameters().getPreviewSize();
//		int imgFormat = mCamera.getParameters().getPreviewFormat();
//		Log.i(ME,"Image Format: "+String.valueOf(imgFormat));
//		int bytesPerPixel = ImageFormat.getBitsPerPixel(imgFormat)/8;
//		mImgBuffer = new byte[(int)(imgSize.width*imgSize.height*bytesPerPixel*1.5)]; // I'm not sure why the 1.5 needs to be there
//		mCamera.addCallbackBuffer(mImgBuffer);
//		mBitmap = Bitmap.createBitmap(imgSize.width, imgSize.height, Bitmap.Config.ARGB_8888);
//		mImgYUV = new Mat(imgSize.height+imgSize.height/2, imgSize.width, CvType.CV_8UC1);
//		mImgRGB = new Mat(imgSize.height, imgSize.width, CvType.CV_8UC4);
//		mCamera.setPreviewCallbackWithBuffer( new Camera.PreviewCallback(){
//			@Override
//			public void onPreviewFrame(byte[] data, Camera cam)
//			{
//				if(mBitmap == null)
//					return;
//
//				if(mLastPreviewTimeNS == 0)
//					mLastPreviewTimeNS = System.nanoTime();
//				else
//				{
//					double dt = (System.nanoTime()-mLastPreviewTimeNS)/1.0e9;
//					mAvgDT = (mAvgDT*mImgProcCnt+dt)/(mImgProcCnt+1);
//					mLastPreviewTimeNS = System.nanoTime();
//				}
//				
//				mImgProcCnt++;
//				mImgYUV.put(0,0,data);
//				Imgproc.cvtColor(mImgYUV,mImgRGB,Imgproc.COLOR_YUV420sp2RGB);
//				Utils.matToBitmap(mImgRGB, mBitmap);
//				mIvImageDisplay.setImageBitmap(mBitmap);
//
//				mCamera.addCallbackBuffer(mImgBuffer);
//			}
//		});
//
//		mCamera.startPreview();

		(new Thread(this)).start();
	}

    @Override
	public void onPause()
	{
//		if(mCamera != null)
//		{
//			mCamera.setPreviewCallback(null);
//			mCamera.release();
//			mCamera = null;
//		}
//		if(mMediaRecorder != null)
//		{
//			mMediaRecorder.reset();
//			mMediaRecorder.release();
//			mMediaRecorder = null;
//		}
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
	}

	public void run()
	{
		Log.i(ME, "Java runner started");
		mThreadIsDone = false;
		mThreadRun = true;
		while(mThreadRun)
		{
			if(mBound && !mService.isConnectedToPC())
			{
//				mBitmap = mService.getImage();
//
//				float gyro[] = mService.getRoverGyroValue();
//				float accel[] = mService.getRoverAccelValue();
//				float mag[] = mService.getRoverMagValue();
//				float att[] = mService.getRoverAttitude();
//				int imgProcTimeMS = mService.getRoverImageProcTimeMS();
//
//				final String strGyro = String.format("Gyro:\t\t%1.2f\t\t%1.2f\t\t%1.2f",gyro[0],gyro[1],gyro[2]);
//				final String strAccel = String.format("Accel:\t\t%1.2f\t\t%1.2f\t\t%1.2f",accel[0],accel[1],accel[2]);
//				final String strMag = String.format("Mag:\t\t%1.2f\t\t%1.2f\t\t%1.2f",mag[0],mag[1],mag[2]);
//				final String strImgProcTime = "Proc Time: "+String.valueOf(imgProcTimeMS)+"ms";
//				final String strRoll = String.format("Roll:\t%1.3f",att[0]);
//				final String strPitch = String.format("Pitch:\t%1.3f",att[1]);
//				final String strYaw = String.format("Yaw:\t%1.3f",att[2]);
//				runOnUiThread(new Runnable(){
//					public void run(){ 
////						mTvGyro.setText(strGyro);
////						mTvAccel.setText(strAccel);
////						mTvMag.setText(strMag);
////						mTvImgProcTime.setText(strImgProcTime);
////						mTvRoll.setText(strRoll);
////						mTvPitch.setText(strPitch);
////						mTvYaw.setText(strYaw);
//
//						if(mBitmap != null)
//							mIvImageDisplay.setImageBitmap(mBitmap); 
//					}
//				});
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

	static{
		System.loadLibrary("opencv_java");
	}
}

