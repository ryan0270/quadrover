package com.icsl.Rover;

import android.app.Service;
import android.app.Notification;
import android.app.Notification.Builder;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.TaskStackBuilder;
import android.content.Intent;
import android.content.Context;
import android.graphics.Bitmap;
import android.os.Binder;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.os.Process;
import android.os.IBinder;
import android.os.Message;
import android.os.HandlerThread;
import android.os.PowerManager;
import android.widget.Toast;
import android.util.Log;

import java.lang.Thread;
import java.io.File;

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
	MediaRecorder mMediaRecorder = null;
	byte[] mImgBuffer;

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
	public void onPause()
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
		mBitmap = Bitmap.createBitmap(imgSize.width, imgSize.height, Bitmap.Config.ARGB_8888);
		mImgYUV = new Mat(imgSize.height+imgSize.height/2, imgSize.width, CvType.CV_8UC1);
//		mImgRGB = new Mat(imgSize.height, imgSize.width, CvType.CV_8UC4);
		mCamera.setPreviewCallbackWithBuffer( new Camera.PreviewCallback(){
			@Override
			public void onPreviewFrame(byte[] data, Camera cam)
			{
				if(mBitmap == null)
				return;
	
				if(mLastPreviewTimeNS == 0)
					mLastPreviewTimeNS = System.nanoTime();
				else
				{
					double dt = (System.nanoTime()-mLastPreviewTimeNS)/1.0e9;
					mAvgDT = (mAvgDT*mImgProcCnt+dt)/(mImgProcCnt+1);
					mLastPreviewTimeNS = System.nanoTime();
				}
		
				mImgProcCnt++;
				mImgYUV.put(0,0,data);
				passNewImage(&mImgYUV);
//				Imgproc.cvtColor(mImgYUV,mImgRGB,Imgproc.COLOR_YUV420sp2RGB);
//				Utils.matToBitmap(mImgRGB, mBitmap);
//				mIvImageDisplay.setImageBitmap(mBitmap);
		
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

		Mat img = new Mat();
		Bitmap bmp;
		try{
			getImage(img.getNativeObjAddr());
			Imgproc.cvtColor(img,img,Imgproc.COLOR_BGR2RGB);
			bmp = Bitmap.createBitmap(img.width(), img.height(), Bitmap.Config.ARGB_8888);
			Utils.matToBitmap(img, bmp);
		} catch(Exception e){
			img = null;
			bmp = null;
		}
		if(img != null)
			img.release();
		return bmp;
	}

	float[] getRoverGyroValue(){ return getGyroValue(); }
	float[] getRoverAccelValue(){ return getAccelValue(); }
	float[] getRoverMagValue(){ return getMagValue(); }
	float[] getRoverAttitude(){ return getAttitude(); }
	int getRoverImageProcTimeMS(){ return getImageProcTimeMS(); }

	public native String nativeTest();
	public native void onJNIStart();
	public native void onJNIStop();
	public native void setNumCpuCores(int numCores);
	public native void setLogDir(String jdir);
	public native void startLogging();
	public native void getImage(long addr);
	public native float[] getGyroValue();
	public native float[] getAccelValue();
	public native float[] getMagValue();
	public native float[] getAttitude();
	public native int getImageProcTimeMS();
//	public native void toggleViewType();
//	public native void toggleUseIbvs();
//	public native int[] getVisionParams();
//	public native void setVisionParams(int[] p);
	public native boolean pcIsConnected();

	static
	{
		System.loadLibrary("opencv_java");
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("Rover");
	}
}
