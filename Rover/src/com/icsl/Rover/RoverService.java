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
import java.util.Timer;
import java.util.TimerTask;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RoverService extends Service {
//	private ServiceHandler mServiceHandler;
	private final static String ME = "RoverService";

	public Timer mTimer;
	public Notification.Builder mNotificationBuilder;

	private PowerManager.WakeLock mWakeLock;

	private final IBinder mBinder = new RoverBinder();

	public class RoverBinder extends Binder 
	{ RoverService getService(){ return RoverService.this; } }

	@Override
	public void onCreate()
	{
		onJNIStart();

		File logDir = new File(Environment.getExternalStorageDirectory().toString()+"/"+ME);
		if(!logDir.exists())
		{
			Log.i(ME,"Log dir: "+logDir.toString()+" does not exist. Creating it.");
			logDir.mkdir();
		}
		else
			Log.i(ME,"Log dir: "+logDir.toString()+" already exists.");
		File imgLogDir = new File(logDir.getAbsolutePath()+"/images");
		if(imgLogDir.exists())
		{
			File bakDir = new File(imgLogDir.getAbsolutePath()+"_bak");
			if(bakDir.exists())
				deleteDir(bakDir);
			imgLogDir.renameTo(bakDir);
		}
		imgLogDir.mkdir();
		setLogDir(logDir.toString());
		File logFile = new File(logDir.getAbsolutePath()+"/log.txt");
		if(logFile.exists())
		{
			File bakFile = new File(logFile.getAbsolutePath()+".bak");
			if(bakFile.exists())
				bakFile.delete();
			logFile.renameTo(bakFile);
		}
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

//		TimerTask task = new TimerTask(){
//			public void run(){
//				String str = String.format("Tmeas: %1.1f\tTinf: %1.1f\tTcpu: %1.1f", getMeasuredTemp(), getTempEst(), getCpuTemp(false));
//
//				mNotificationBuilder.setContentText(str);
//				Notification noti = mNotificationBuilder.build();
//
//				nm.notify(1, noti);
//			}
//		};

		PowerManager pm = (PowerManager)getSystemService(Context.POWER_SERVICE);
		mWakeLock = pm.newWakeLock(PowerManager.SCREEN_DIM_WAKE_LOCK, ME);

//		mTimer = new Timer();
//		mTimer.schedule(task, 100, 500);

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
//		mTimer.cancel();
//		mTimer.purge();
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
	public native void toggleViewType();
	public native void toggleUseIbvs();
	public native int[] getVisionParams();
	public native void setVisionParams(int[] p);
	public native boolean pcIsConnected();

	static
	{
		System.loadLibrary("opencv_java");
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("Rover");
	}
}
