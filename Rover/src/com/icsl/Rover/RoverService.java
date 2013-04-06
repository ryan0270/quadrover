package com.icsl.Rover;

import android.app.Service;
import android.app.Notification;
import android.app.Notification.Builder;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.TaskStackBuilder;
import android.content.Intent;
import android.content.Context;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.hardware.Camera.Size;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.media.CamcorderProfile;
import android.media.MediaRecorder;
import android.os.Binder;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.Process;
import android.os.IBinder;
import android.os.Message;
import android.os.HandlerThread;
import android.os.PowerManager;
import android.util.Log;
import android.widget.Toast;
import android.view.SurfaceView;
import android.view.SurfaceHolder;

import java.lang.Thread;
import java.io.File;
import java.util.List;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RoverService extends Service {
//	private ServiceHandler mServiceHandler;
	private final static String ME = "RoverService";

	public Notification.Builder mNotificationBuilder;

	private PowerManager.WakeLock mWakeLock;

	private final IBinder mBinder = new RoverBinder();

	private Camera mCamera = null;
	private MediaRecorder mMediaRecorder = null;
	private byte[] mImgBuffer;
	private Mat mImgYUV, mImgRGB;
	private long mLastPreviewTimeNS = 0;
	private int mImgProcCnt = 0;
	private double mAvgDT = 0;
	private Mat mImage;
	private Bitmap mBmp = null;

	public class RoverBinder extends Binder 
	{ RoverService getService(){ return RoverService.this; } }

	@Override
	public void onCreate()
	{
		File logDir = new File(Environment.getExternalStorageDirectory().toString()+"/"+ME);
		if(!logDir.exists())
		{
			Log.i(ME,"Log dir: "+logDir.toString()+" does not exist. Creating it.");
			logDir.mkdir();
		}
		File imgLogDir = new File(logDir.getAbsolutePath()+"/images");
		if(imgLogDir.exists())
		{
			File bakDir = new File(imgLogDir.getAbsolutePath()+"_bak");
			if(bakDir.exists())
				deleteDir(bakDir);
			imgLogDir.renameTo(bakDir);
		}
		imgLogDir.mkdir();
		File videoDir = new File(logDir.getAbsolutePath()+"/video");
		Log.i(ME,"Video dir = "+videoDir.getAbsolutePath());
		if(videoDir.exists())
		{
			File bakDir = new File(videoDir.getAbsolutePath()+"_bak");
			if(bakDir.exists())
				deleteDir(bakDir);
			videoDir.renameTo(bakDir);
		}
		videoDir.mkdir();
		File logFile = new File(logDir.getAbsolutePath()+"/log.txt");
		if(logFile.exists())
		{
			File bakFile = new File(logFile.getAbsolutePath()+".bak");
			if(bakFile.exists())
				bakFile.delete();
			logFile.renameTo(bakFile);
		}
		File matchDataFile = new File(logDir.getAbsolutePath()+"/matchData.xml");
		if(matchDataFile.exists())
		{
			File bakFile = new File(matchDataFile.getAbsolutePath()+".bak");
			if(bakFile.exists())
				bakFile.delete();
			matchDataFile.renameTo(bakFile);
		}

		onJNIStart();
		setLogDir(logDir.toString());
		startLogging();

		mNotificationBuilder = new Notification.Builder(this);
		mNotificationBuilder.setContentTitle("Rover");
		mNotificationBuilder.setSmallIcon(R.drawable.ic_launcher);
		Notification noti = mNotificationBuilder.build();
		startForeground(1, noti);

		// taken from http://developer.android.com/guide/topics/ui/notifiers/notifications.html
		Intent mainActivityIntent = new Intent(this, Rover.class);
		TaskStackBuilder stackBuilder = TaskStackBuilder.create(this);
		stackBuilder.addParentStack(Rover.class);
		stackBuilder.addNextIntent(mainActivityIntent);
		PendingIntent mainActivityPendingIntent = stackBuilder.getPendingIntent(0, PendingIntent.FLAG_UPDATE_CURRENT);
		mNotificationBuilder.setContentIntent(mainActivityPendingIntent);

		final NotificationManager nm = (NotificationManager)getSystemService(Context.NOTIFICATION_SERVICE);
		nm.notify(1, mNotificationBuilder.build());

		PowerManager pm = (PowerManager)getSystemService(Context.POWER_SERVICE);
		mWakeLock = pm.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK, ME);

		openCamera();

		Toast.makeText(this, "Rover fetching", Toast.LENGTH_SHORT).show();
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int startId)
	{
		mWakeLock.acquire();

		// If we get killed, after returning from here, play dead
		return START_NOT_STICKY;
	}

    @Override
	public void onDestroy()
	{
		Log.i(ME,"Rover service stop started");
		if(mCamera != null)
		{
			mCamera.setPreviewCallback(null);
			mCamera.release();
			mCamera = null;
		}
		if(mMediaRecorder != null)
		{
			mMediaRecorder.reset();
			mMediaRecorder.release();
			mMediaRecorder = null;
		}

		onJNIStop();
		mWakeLock.release();
		Toast.makeText(this, "Rover sleeping", Toast.LENGTH_SHORT).show();
		stopForeground(true);
		Log.i(ME,"Rover service stopped");
		
		super.onDestroy();
	}

	@Override
	public IBinder onBind(Intent intent)
	{
		return mBinder;
	}

	private void openCamera()
	{
		try{
			mCamera = Camera.open();
			Camera.Parameters camParams = mCamera.getParameters();
			List<Size> previewSizes = camParams.getSupportedPreviewSizes();
			camParams.setPreviewSize(640,480);

			//			List<int[]> fpsList = camParams.getSupportedPreviewFpsRange();
			//			int[] fps = fpsList.get(fpsList.size()-1);
			//			camParams.setPreviewFpsRange((fps[0]), fps[1]);
			camParams.setPreviewFpsRange(30000, 30000);

			if(camParams.getVideoStabilization())
			{
				Log.i(ME, "I have video stabilization");
				camParams.setVideoStabilization(true);
			}
			else
				Log.i(ME, "I don't have video stabilization");

			Size preferredVideoSize = camParams.getPreferredPreviewSizeForVideo();
			Log.i(ME, "Preferred video size: "+String.valueOf(preferredVideoSize.width)+"x"+String.valueOf(preferredVideoSize.height));

//			camParams.setExposureCompensation( camParams.getMaxExposureCompensation() );

			camParams.setFocusMode( Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO );

			mCamera.setParameters(camParams);
			SurfaceView dummy = new SurfaceView(getBaseContext());
			mCamera.setPreviewDisplay(dummy.getHolder());

			mMediaRecorder = new MediaRecorder();
			mMediaRecorder.setCamera(mCamera);
			mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
			mMediaRecorder.setProfile(CamcorderProfile.get(CamcorderProfile.QUALITY_480P));
			mMediaRecorder.prepare();
		}
		catch(Exception e){Log.i(ME, e.toString());}

		Camera.Size imgSize = mCamera.getParameters().getPreviewSize();
		int imgFormat = mCamera.getParameters().getPreviewFormat();
		Log.i(ME,"Image Format: "+String.valueOf(imgFormat));
		int bytesPerPixel = ImageFormat.getBitsPerPixel(imgFormat)/8;
		mImgBuffer = new byte[(int)(imgSize.width*imgSize.height*bytesPerPixel*1.5)]; // I'm not sure why the 1.5 needs to be there
		mCamera.addCallbackBuffer(mImgBuffer);
		mImgYUV = new Mat(imgSize.height+imgSize.height/2, imgSize.width, CvType.CV_8UC1);
		mCamera.setPreviewCallbackWithBuffer( new Camera.PreviewCallback(){
			@Override
			public void onPreviewFrame(byte[] data, Camera cam)
			{
				// System.nanoTime() is the same as clock_gettime(CLOCK_MONOTONIC, &tv) in c++
				long timestamp = System.nanoTime();
//				if(mLastPreviewTimeNS == 0)
//					mLastPreviewTimeNS = System.nanoTime();
//				else
//				{
//					double dt = (System.nanoTime()-mLastPreviewTimeNS)/1.0e9;
//					mAvgDT = (mAvgDT*mImgProcCnt+dt)/(mImgProcCnt+1);
//					mLastPreviewTimeNS = System.nanoTime();
//
//					Log.i(ME, "mAvgDT: "+String.valueOf(mAvgDT));
//				}
  	
				mImgProcCnt++;
				mImgYUV.put(0,0,data);
				passNewImage(mImgYUV.getNativeObjAddr(), timestamp);
		
				mCamera.addCallbackBuffer(mImgBuffer);
			}
		});

		mCamera.startPreview();
	}

	// taken from 
	// http://www.rgagnon.com/javadetails/java-0483.html
	static private boolean deleteDir(File path)
	{
		if( path.exists() ) {
			File[] files = path.listFiles();
			for(int i=0; i<files.length; i++) {
				if(files[i].isDirectory()) {
					deleteDir(files[i]);
				}
				else {
					files[i].delete();
				}
			}
		}
		return( path.delete() );
	}

	public Bitmap getImage()
	{
		if(pcIsConnected())
			return null;

		try{
			if( getImage(mImage.getNativeObjAddr()) )
				Imgproc.cvtColor(mImage,mImage,Imgproc.COLOR_BGR2RGB);
			else
				return null;
			if(mBmp == null || mBmp.getWidth() != mImage.width() || mBmp.getHeight() != mImage.height())
				mBmp = Bitmap.createBitmap(mImage.width(), mImage.height(), Bitmap.Config.ARGB_8888);
			Utils.matToBitmap(mImage, mBmp);
		} catch(Exception e){
			mImage = null;
			mBmp = null;
		}
		return mBmp;
	}

	float[] getRoverGyroValue(){ return getGyroValue(); }
	float[] getRoverAccelValue(){ return getAccelValue(); }
	float[] getRoverMagValue(){ return getMagValue(); }
	float[] getRoverAttitude(){ return getAttitude(); }
	int getRoverImageProcTimeMS(){ return getImageProcTimeMS(); }
	boolean isConnectedToPC(){ return pcIsConnected();}
//	boolean isConnectedToPC(){ return true; }

	public native String nativeTest();
	public native void onJNIStart();
	public native void onJNIStop();
	public native void setNumCpuCores(int numCores);
	public native void setLogDir(String jdir);
	public native void startLogging();
	public native boolean getImage(long addr);
	public native float[] getGyroValue();
	public native float[] getAccelValue();
	public native float[] getMagValue();
	public native float[] getAttitude();
	public native int getImageProcTimeMS();
	public native boolean pcIsConnected();
	public native void passNewImage(long addr, long timestampNS);

	static
	{
		System.loadLibrary("opencv_java");
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("Rover");
	}
}
