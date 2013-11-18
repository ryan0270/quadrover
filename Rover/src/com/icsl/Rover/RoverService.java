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
import android.media.MediaRecorder.OutputFormat;
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
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.os.ParcelFileDescriptor;

import java.lang.Thread;
import java.io.File;
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

public class RoverService extends Service
{
//	private ServiceHandler mServiceHandler;
	private final static String ME = "RoverService";

	public Notification.Builder mNotificationBuilder;

	private PowerManager.WakeLock mWakeLock = null;

	private final IBinder mBinder = new RoverBinder();

	private Camera mCamera = null;
	private MediaRecorder mMediaRecorder = null;
	private byte[] mImgBuffer;
	private Mat mImgYUV, mImgRGB;
	private long mLastPreviewTimeNS = 0;
	private int mImgProcCnt = 0;
	private double mAvgDT = 0;
	private Mat mImage = null;
	private Bitmap mBmp = null;

	private UsbAccessory mAccessory;
	private UsbManager mUsbManager;
	private PendingIntent mPermissionIntent;
	private boolean mPermissionRequestPending;
	private ParcelFileDescriptor mFileDescriptor = null;
	private FileInputStream mInputStream;
	private FileOutputStream mOutputStream;

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
//		File imgLogDir = new File(logDir.getAbsolutePath()+"/images");
//		if(imgLogDir.exists())
//		{
//			File bakDir = new File(imgLogDir.getAbsolutePath()+"_bak");
//			if(bakDir.exists())
//				deleteDir(bakDir);
//			imgLogDir.renameTo(bakDir);
//		}
//		imgLogDir.mkdir();
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

		openCamera();

        mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);

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

		Toast.makeText(this, "Rover fetching", Toast.LENGTH_SHORT).show();
	}

	@Override
	public int onStartCommand(Intent intent, int flags, int startId)
	{
		if(mWakeLock != null)
			mWakeLock.acquire();

		// If we get killed, after returning from here, play dead
		return START_NOT_STICKY;
	}

    @Override
	public void onDestroy()
	{
		super.onDestroy();
		Log.i(ME,"Rover service stop started");
		try
		{
			if(mCamera != null)
			{
				mCamera.setPreviewCallback(null);
				mCamera.release();
				mCamera = null;
			}
		}
		catch(Exception e){Log.i(ME,"destroy exception: "+e);}

		try
		{
			if(mMediaRecorder != null)
			{
				mMediaRecorder.reset();
				mMediaRecorder.release();
				mMediaRecorder = null;
			}
		}
		catch(Exception e){Log.i(ME,"destroy exception: "+e);}

		try
		{
			if(mAdkReadMonitor != null)
			{
				mAdkReadMonitor.stop();
				mAdkReadMonitorThread.join();
			}
			if(mAdkMotorCmdSender != null)
			{
				mAdkMotorCmdSender.stop();
				mAdkMotoCmdSenderThread.join();
			}
			mAdkReadMonitor = null;
			mAdkMotorCmdSender = null;
		}
		catch(Exception e){Log.i(ME,"destroy exception: "+e);}

		onJNIStop();
		if(mWakeLock != null && mWakeLock.isHeld())
			mWakeLock.release();
		Toast.makeText(this, "Rover sleeping", Toast.LENGTH_SHORT).show();
		stopForeground(true);
		Log.i(ME,"Rover service stopped");
	}

	@Override
	public IBinder onBind(Intent intent)
	{
		return mBinder;
	}

	private Object imgLock = new Object();
	private Object adkLock = new Object();
	private void openCamera()
	{
		try{
			mCamera = Camera.open();
			Camera.Parameters camParams = mCamera.getParameters();
			List<Size> previewSizes = camParams.getSupportedPreviewSizes();
//			camParams.setPreviewSize(640,480);
			camParams.setPreviewSize(320,240);

			List<int[]> fpsList = camParams.getSupportedPreviewFpsRange();
			for(int i=0; i<fpsList.size(); i++)
			{
				int[] fps = fpsList.get(i);
				Log.i(ME, "Supported fps: " + String.valueOf(fpsList.get(i)[0]) + ", " + String.valueOf(fpsList.get(i)[1]));
			}
			//	camParams.setPreviewFpsRange((fps[0]), fps[1]);
			camParams.setPreviewFpsRange(30000, 30000);
			// I'm not sure if this overwrites the ISO, or other, settings
// 			camParams.setVideoStabilization(true);
			if(camParams.getVideoStabilization())
				Log.i(ME, "I have video stabilization");
			else
				Log.i(ME, "I don't have video stabilization");

			Size preferredVideoSize = camParams.getPreferredPreviewSizeForVideo();
			Log.i(ME, "Preferred video size: "+String.valueOf(preferredVideoSize.width)+"x"+String.valueOf(preferredVideoSize.height));

//			camParams.setExposureCompensation( camParams.getMaxExposureCompensation() );

//			camParams.setFocusMode( Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO );
			camParams.setFocusMode( Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
//			camParams.setSceneMode( Camera.Parameters.SCENE_MODE_SPORTS );

			// iso 100, 200, 400, 800
			camParams.set("iso",800);
			// with high iso there seems to be more banding
			camParams.setAntibanding( Camera.Parameters.ANTIBANDING_AUTO );
			// I'm not sure what fast-fps-mode does
//			camParams.set("fast-fps-mode","on");
//			String [] params = camParams.flatten().split(";");
//			Log.i(ME,"Camera params");
//			for(int i=0; i<params.length; i++)
//				Log.i(ME,"\t"+params[i]);

			mCamera.setParameters(camParams);
			SurfaceView dummy = new SurfaceView(getBaseContext());
			mCamera.setPreviewDisplay(dummy.getHolder());

			// I can't decide if using video record mode helps
			// the preview images or not
			//
			// MediaRecorder runs in a state machine and without setting
			// things up in the right order it fails. I'm too lazy to figure
			// out which of  the below steps I could skip.
//			mMediaRecorder = new MediaRecorder();
//			mMediaRecorder.setCamera(mCamera);
//			mMediaRecorder.setAudioSource(MediaRecorder.AudioSource.MIC);
//			mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
//			mMediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.THREE_GPP);
//			mMediaRecorder.setAudioEncoder(MediaRecorder.AudioEncoder.AMR_NB);
//			mMediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.H264);
//			File logDir = new File(Environment.getExternalStorageDirectory().toString()+"/"+ME);
//			mMediaRecorder.setOutputFile(logDir.toString()+"/vid.mpg");
//			mMediaRecorder.setVideoSize(640,480);
//			mMediaRecorder.setVideoFrameRate(30);
//			mMediaRecorder.prepare();
		}
		catch(Exception e){Log.i(ME,"CHADDDDDDDD"); Log.e(ME, e.toString());}

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
//					Log.i(ME, "--------------------------------------------------");
//					Log.i(ME, "dt: " + String.valueOf(dt));
//					Log.i(ME, "mAvgDT: "+String.valueOf(mAvgDT));
//				}
  	
				mImgProcCnt++;
				synchronized(imgLock)
				{
					mImgYUV.put(0,0,data);
					passNewImage(mImgYUV.getNativeObjAddr(), timestamp);
				}
		
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
			if(mImage == null)
				mImage = new Mat(240,320,CvType.CV_8UC3);
//				mImage = new Mat(480,640,CvType.CV_8UC3);

			if( getImage(mImage.getNativeObjAddr()) )
				Imgproc.cvtColor(mImage,mImage,Imgproc.COLOR_BGR2RGB);
			else
				return null;
			if(mBmp == null || mBmp.getWidth() != mImage.width() || mBmp.getHeight() != mImage.height())
				mBmp = Bitmap.createBitmap(mImage.width(), mImage.height(), Bitmap.Config.ARGB_8888);
			Utils.matToBitmap(mImage, mBmp);
		} catch(Exception e){
			Log.e(ME,e.toString());
			mImage = null;
			mBmp = null;
		}
		return mBmp;
	}

	public void openAccessory(UsbAccessory accessory)
	{
		mAccessory = accessory;
		mFileDescriptor = mUsbManager.openAccessory(accessory);
		if (mFileDescriptor != null)
		{
			FileDescriptor fd = mFileDescriptor.getFileDescriptor();

			mInputStream = new FileInputStream(fd);
			mAdkReadMonitor = new ADKReadMonitor();
			mAdkReadMonitorThread = new Thread(mAdkReadMonitor);
			mAdkReadMonitorThread.start();

			mOutputStream = new FileOutputStream(fd);
			mAdkMotorCmdSender = new ADKMotorCmdSender();
			mAdkMotoCmdSenderThread = new Thread(mAdkMotorCmdSender);
			mAdkMotoCmdSenderThread.start();

			Log.d(ME, "accessory opened");
//			setAdkConnected(true);
		}
		else
			Log.d(ME, "accessory open fail");
	}

	public void closeAccessory()
	{
		Log.i(ME,"Closing accessory");
		try
		{
			if(mAdkReadMonitor != null)
			{
				mAdkReadMonitor.stop();
				mAdkReadMonitorThread.join();
			}
			if(mAdkMotorCmdSender != null)
			{
				mAdkMotorCmdSender.stop();
				mAdkMotoCmdSenderThread.join();
			}
			mAdkReadMonitor = null;
			mAdkMotorCmdSender = null;

			if (mFileDescriptor != null)
				mFileDescriptor.close();
		}
		catch (Exception e){}

		mFileDescriptor = null;
		mAccessory = null;
		Log.i(ME,"Closing the accessory");
	}

	public byte[] int2ByteArray(int val, int numBytes)
	{
		byte[] chad = new byte[numBytes];
		for(int i=0; i<numBytes; i++)
			chad[numBytes-i-1] = (byte)(val >> i*8);

		return chad;
	}

	// I don't use this so it's not well tested
	public byte[] float2ByteArray(float val)
	{
		int valBytes = Float.floatToIntBits(val);
		byte[] chad = new byte[4];
		chad[0] = (byte)(valBytes >> 3*8);
		chad[1] = (byte)(valBytes >> 2*8);
		chad[2] = (byte)(valBytes >> 1*8);
		chad[3] = (byte)(valBytes >> 0*8);

		return chad;
	}
	
	// mInputStream.read() blocks and I can't use mInputStream.available()
	// due to an apparent bug (always throws and exception) so I have to run
	// separate threads for read/write
	private class ADKReadMonitor implements Runnable
	{
		public void stop()
		{
			Log.i(ME,"Telling adk cmd runner to stop");
			mRunning = false;
		}

		public void run()
		{
			mRunning = true;
			Thread me = Thread.currentThread();
			me.setPriority(Thread.MAX_PRIORITY-2);
			while(mRunning)
			{
				if(mInputStream != null)
				{
					try
					{
						byte[] buff = new byte[2];
						int ret = mInputStream.read(buff, 0, 2);
						long timestamp = System.nanoTime();
						int val = ByteBuffer.wrap(buff).getShort();
						onNewSonarReading(val, timestamp);
					}
					catch (Exception e)
					{
						Log.e(ME,"Error reading stream: "+e.toString());
						mInputStream = null;
						if(mFileDescriptor != null)
						{
							FileDescriptor fd = mFileDescriptor.getFileDescriptor();
							mInputStream = new FileInputStream(fd);
						}
					}
				}

				try{ Thread.sleep(1); }
				catch(Exception e){ Log.e(ME,"Caught sleeping"); }
			}

			Log.i(ME,"ADKMonitor runner finished");
		}

		private boolean mRunning;
	}
	private ADKReadMonitor mAdkReadMonitor = null;
	private Thread mAdkReadMonitorThread = null;

	private class ADKMotorCmdSender implements Runnable
	{
		public void stop()
		{
			Log.i(ME,"Telling adk cmd send runner to stop");
			mRunning = false;
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
			{
				Log.e(ME, "write failed: ", e);
				mInputStream = null;
				mOutputStream = null;
				if (mFileDescriptor != null)
				{
					try
					{ mFileDescriptor.close(); }
					catch (Exception e1){}
				}
				mFileDescriptor = null;
			}

			return success;
		}

		public void run()
		{
			mRunning = true;

			Thread me = Thread.currentThread();
			me.setPriority(Thread.MAX_PRIORITY);

			long lastTime = System.nanoTime();
			long curTime;
			while(mRunning)
			{
				curTime = System.nanoTime();
				if(curTime-lastTime > 20e6)
					Log.i(ME, "Long time: " + String.valueOf((int)((curTime-lastTime)/1.e6 + 0.5)));
				lastTime = curTime;

				int[] cmds = getMotorCmds();
				if(cmds.length == 4)
					sendMotorCommands(cmds[0], cmds[1], cmds[2], cmds[3]);

				try{ Thread.sleep(5); }
				catch(Exception e){ Log.e(ME,"Caught sleeping"); }
			}

			Log.i(ME,"ADKMotorCmdSender runner finished");
		}

		private boolean mRunning;
	}
	private ADKMotorCmdSender mAdkMotorCmdSender= null;
	private Thread mAdkMotoCmdSenderThread= null;

	float[] getRoverGyroValue(){ return getGyroValue(); }
	float[] getRoverAccelValue(){ return getAccelValue(); }
	float[] getRoverMagValue(){ return getMagValue(); }
	float[] getRoverAttitude(){ return getAttitude(); }
	int getRoverImageProcTimeMS(){ return getImageProcTimeMS(); }
	boolean isConnectedToPC(){ return pcIsConnected();}

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
	public native int[] getMotorCmds();
	public native int getImageProcTimeMS();
	public native boolean pcIsConnected();
	public native void passNewImage(long addr, long timestampNS);
	public native void onNewSonarReading(int val, long timestampNS);

	static
	{
		System.loadLibrary("opencv_java");
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("Rover");
	}
}
