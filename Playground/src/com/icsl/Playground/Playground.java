package com.icsl.Playground;

import android.app.Activity;
import android.os.Bundle;
import android.os.Environment;
import android.view.View;
import android.widget.ImageView;
import android.content.Intent;
import android.widget.EditText;
import android.widget.TextView;
import android.graphics.Bitmap;
import android.util.Log;
import android.content.res.AssetManager;

import java.lang.String;
import java.lang.Integer;
import java.io.File;
import java.io.FileFilter;
import java.util.regex.Pattern;

//import org.opencv.android.BaseLoaderCallback;
//import org.opencv.android.LoaderCallbackInterface;
//import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Playground extends Activity implements Runnable
{
	private static final String ME = "Playground";

	private Mat mImage;
	private Bitmap mBitmap;
	private boolean mThreadRun, mThreadIsDone;

	private ImageView mIvImageDisplay;
	private TextView mTvImgProcTime;

    /** called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
		mThreadIsDone = true;

		mIvImageDisplay = (ImageView)findViewById(R.id.ivImageDisplay);
		mTvImgProcTime= (TextView)findViewById(R.id.tvImgProcTime);
    }

    @Override
	public void onResume()
	{
		super.onResume();

		mBitmap = Bitmap.createBitmap(320, 240, Bitmap.Config.ARGB_8888);
		mIvImageDisplay.setImageBitmap(mBitmap);

		onJNIStart();

		setNumCpuCores(getNumCores());
//		setNumCpuCores(4);

		mImage = new Mat(240,320,CvType.CV_8UC3,new Scalar(127));


		File logDir = new File(Environment.getExternalStorageDirectory().toString()+"/Playground");
		if(!logDir.exists())
			logDir.mkdir();
		setLogDir(logDir.toString());
		startLogging();

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
				Log.e(ME,"caught sleeping 2");}
		}

		if(mImage != null)
			mImage.release();
		mImage = null;

		onJNIStop();


		// there seems be a Thread issue where android is still trying to draw the Bitmap during
		// shutdown after i've already recycled mBitmap. so set the image to null before recycling.
		mIvImageDisplay.setImageBitmap(null);
		if(mBitmap != null)
			mBitmap.recycle();
		mBitmap = null;

		super.onPause();

		Log.i(ME,"java paused");
	}

	@Override
	public void onStop()
	{
		Log.i(ME,"java stopped");
		super.onStop();
	}

	public void run()
	{
		Log.i(ME, "java runner started");
		long startTime = System.nanoTime();
		mThreadIsDone = false;
		mThreadRun = true;
		while(mThreadRun)
		{
			getImage(mImage.getNativeObjAddr());

			Mat img = new Mat();
			try{
				Imgproc.cvtColor(mImage,img,Imgproc.COLOR_BGR2RGB);
				Utils.matToBitmap(img, mBitmap);
			} catch(Exception e){
				img = null;
				//				mBitmap.recycle();
				//				mBitmap= null;
			}

			if(img != null)
			{
				img.release();

					runOnUiThread(new Runnable(){
						public void run(){ 
							mTvImgProcTime.setText("proc time: "+String.valueOf(getImageProcTimeMS())+"ms");

							mIvImageDisplay.setImageBitmap(mBitmap); 
						}
					});
				}
			}

			try{
				Thread.sleep(50);
			} catch(Exception e){
				Log.e(ME,"caught sleeping");
		}

		Log.i(ME,"java runner dead *************************");
		mThreadIsDone = true;
	}

	/**
	 * Gets the number of cores available in this device, across all processors.
	 * Requires: Ability to peruse the fileSystem at "/sys/devices/system/cpu"
	 * @return The number of cores, or 1 if failed to get result
	 *
	 * Taken from
	 * http://makingmoneywithandroid.com/forum/showThread.php?tid=298&pid=1663#pid1663
	 */
	private int getNumCores() 
	{
		//Private Class to display only CPU devices in the directory listing
		class CpuFilter implements FileFilter {
			@Override
			public boolean accept(File pathname) {
				//Check if filename is "cpu", followed by a single digit number
				if(Pattern.matches("cpu[0-9]", pathname.getName())) {
					return true;
				}
				return false;
			}      
		}

		try {
			//Get directory containing CPU info
			File dir = new File("/sys/devices/system/cpu/");
			//Filter to only list the devices we care about
			File[] files = dir.listFiles(new CpuFilter());
			Log.d(ME, "CPU Count: "+files.length);
			//Return the number of cores (virtual CPU devices)
			return files.length;
		} catch(Exception e) {
			//Print exception
			Log.d(ME, "CPU Count: Failed.");
			e.printStackTrace();
			//Default to return 1 core
			return 1;
		}
	}

	public native void onJNIStart();
	public native void onJNIStop();
	public native void setNumCpuCores(int numCores);
	public native void setLogDir(String jdir);
	public native void startLogging();
	public native void getImage(long addr);
	public native int getImageProcTimeMS();

	static {
		System.loadLibrary("opencv_java");
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("Playground");
	}
}

