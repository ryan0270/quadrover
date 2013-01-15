package com.icsl.QuadPhone;

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

public class QuadPhone extends Activity implements Runnable
{
	public final static String EXTRA_MESSAGE = "com.icsl.QuadPhone.MESSAGE";
	private static final String ME = "QuadPhone";

	private Mat mImage;
	private Bitmap mBitmap;
	private boolean mThreadRun, mThreadIsDone, mOpenCVManagerConnected;

	private ImageView mIvImageDisplay;
	private EditText mEditBox1Min, mEditBox1Max;
	private EditText mEditBox2Min, mEditBox2Max;
	private EditText mEditBox3Min, mEditBox3Max;
	private EditText mEditBox4Min, mEditBox4Max;
	private EditText mEditSatMin, mEditSatMax;
	private EditText mEditValMin, mEditValMax;
	private EditText mEditCircMin, mEditCircMax;
	private EditText mEditConvMin, mEditConvMax;
	private TextView mTvGyro, mTvAccel, mTvMag, mTvImgProcTime;
	private TextView mTvRoll, mTvPitch, mTvYaw;

	static AssetManager mAssetManager;

//	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this){
//		@Override
//		public void onManagerConnected(int status){
//			switch(status) {
//				case LoaderCallbackInterface.SUCCESS:
//					{
//						Log.i(ME, "OpenCV loaded successfully");
//
//						// Load native library after OpenCV initialization
//						System.loadLibrary("opencv_core");
//						System.loadLibrary("opencv_imgproc");
//						System.loadLibrary("opencv_flann");
//						System.loadLibrary("opencv_highgui");
//						System.loadLibrary("opencv_features2d");
//						System.loadLibrary("opencv_calib3d");
//						System.loadLibrary("opencv_ml");
//						System.loadLibrary("opencv_video");
//						System.loadLibrary("opencv_objdetect");
//						System.loadLibrary("opencv_contrib");
//						System.loadLibrary("opencv_legacy");
//						System.loadLibrary("opencv_nonfree");
//						System.loadLibrary("opencv_photo");
//						System.loadLibrary("opencv_stitching");
//						System.loadLibrary("opencv_ts");
//						System.loadLibrary("opencv_videostab");
//						System.loadLibrary("toadlet_egg");
//						System.loadLibrary("QuadPhone");
//						mOpenCVManagerConnected = true;
//					} break;
//				default:
//					{
//						super.onManagerConnected(status);
//					} break;
//			}
//		}
//	};

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
		mThreadIsDone = true;
		mOpenCVManagerConnected = false;

		mIvImageDisplay = (ImageView)findViewById(R.id.ivImageDisplay);
		mEditBox1Min = (EditText)findViewById(R.id.editBox1Min);
        mEditBox1Max = (EditText)findViewById(R.id.editBox1Max);
        mEditBox2Min = (EditText)findViewById(R.id.editBox2Min);
        mEditBox2Max = (EditText)findViewById(R.id.editBox2Max);
        mEditBox3Min = (EditText)findViewById(R.id.editBox3Min);
        mEditBox3Max = (EditText)findViewById(R.id.editBox3Max);
        mEditBox4Min = (EditText)findViewById(R.id.editBox4Min);
        mEditBox4Max = (EditText)findViewById(R.id.editBox4Max);
        mEditSatMin  = (EditText)findViewById(R.id.editSatMin);
        mEditSatMax  = (EditText)findViewById(R.id.editSatMax);
        mEditValMin  = (EditText)findViewById(R.id.editValMin);
        mEditValMax  = (EditText)findViewById(R.id.editValMax);
        mEditCircMin = (EditText)findViewById(R.id.editCircMin);
        mEditCircMax = (EditText)findViewById(R.id.editCircMax);
        mEditConvMin = (EditText)findViewById(R.id.editConvMin);
        mEditConvMax = (EditText)findViewById(R.id.editConvMax);

		mTvGyro = (TextView)findViewById(R.id.tvGyroValue);
		mTvAccel= (TextView)findViewById(R.id.tvAccelValue);
		mTvMag= (TextView)findViewById(R.id.tvMagValue);
		mTvImgProcTime= (TextView)findViewById(R.id.tvImgProcTime);

		mTvRoll = (TextView)findViewById(R.id.tvRollAngle);
		mTvPitch= (TextView)findViewById(R.id.tvPitchAngle);
		mTvYaw= (TextView)findViewById(R.id.tvYawAngle);

		mAssetManager = getAssets();
    }

    @Override
	public void onResume()
	{
		super.onResume();

//		if(!OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback))
//		{
//			Log.e(ME, "Cannot connect to OpenCV Manager");
//			return;
//		}

		mBitmap = Bitmap.createBitmap(320, 240, Bitmap.Config.ARGB_8888);
		mIvImageDisplay.setImageBitmap(mBitmap);

//		int counter = 0;
//		while(++counter < 10 && !mOpenCVManagerConnected)
//		{
//			Log.e(ME, "Waiting on OpenCV manager");
//			try{
//				Thread.sleep(1000);
//			} catch (Exception e)
//			{ Log.e(ME, "wtc, why can't I sleep?"); }
//
//		}
//		if(!mOpenCVManagerConnected)
//		{
//			Log.e(ME, "OpenCV Manager didn't start so I quit");
//			return;
//		}
		onJNIStart();
		setAssetManager(mAssetManager);

		setNumCpuCores(getNumCores());

		mImage = new Mat(240,320,CvType.CV_8UC3,new Scalar(127));
		setLogDir(Environment.getExternalStorageDirectory().toString()+"/QuadPhone");
		startLogging();

		populateVisionParams();

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
			onJNIStop();


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
		Log.i(ME,"Java stopped");
		super.onStop();
	}

	public void onBtnToggleImageProcessing_clicked(View v)
	{
		toggleUseIbvs();
	}

	public void onBtnToggleGray_clicked(View v)
	{
		toggleViewType();
	}

	public void onBtnApplyVisionParams_clicked(View v)
	{
		int[] p = new int[16];
		try{
			p[0]  = Integer.decode(mEditBox1Min.getText().toString());
			p[1]  = Integer.decode(mEditBox1Max.getText().toString());
			p[2]  = Integer.decode(mEditBox2Min.getText().toString());
			p[3]  = Integer.decode(mEditBox2Max.getText().toString());
			p[4]  = Integer.decode(mEditBox3Min.getText().toString());
			p[5]  = Integer.decode(mEditBox3Max.getText().toString());
			p[6]  = Integer.decode(mEditBox4Min.getText().toString());
			p[7]  = Integer.decode(mEditBox4Max.getText().toString());
			p[8]  = Integer.decode(mEditSatMin.getText().toString());
			p[9]  = Integer.decode(mEditSatMax.getText().toString());
			p[10] = Integer.decode(mEditValMin.getText().toString());
			p[11] = Integer.decode(mEditValMax.getText().toString());
			p[12] = Integer.decode(mEditCircMin.getText().toString());
			p[13] = Integer.decode(mEditCircMax.getText().toString());
			p[14] = Integer.decode(mEditConvMin.getText().toString());
			p[15] = Integer.decode(mEditConvMax.getText().toString());

			setVisionParams(p);
		} catch(Exception e){
			Log.i(ME,"Couldn't convert a vision parameter so I'm ignoring them all");
		}

	}

	public void onBtnResetVisionParams_clicked(View v)
	{
		populateVisionParams();
	}

	public void populateVisionParams()
	{
		runOnUiThread(new Runnable(){ public void run(){
			int[] p = getVisionParams();
			mEditBox1Min.setText(String.valueOf(p[0]));
			mEditBox1Max.setText(String.valueOf(p[1]));
			mEditBox2Min.setText(String.valueOf(p[2]));
			mEditBox2Max.setText(String.valueOf(p[3]));
			mEditBox3Min.setText(String.valueOf(p[4]));
			mEditBox3Max.setText(String.valueOf(p[5]));
			mEditBox4Min.setText(String.valueOf(p[6]));
			mEditBox4Max.setText(String.valueOf(p[7]));
			mEditSatMin.setText(String.valueOf(p[8]));
			mEditSatMax.setText(String.valueOf(p[9]));
			mEditValMin.setText(String.valueOf(p[10]));
			mEditValMax.setText(String.valueOf(p[11]));
			mEditCircMin.setText(String.valueOf(p[12]));
			mEditCircMax.setText(String.valueOf(p[13]));
			mEditConvMin.setText(String.valueOf(p[14]));
			mEditConvMax.setText(String.valueOf(p[15]));
		}});
	}

	public void run()
	{
		Log.i(ME, "Java runner started");
		long startTime = System.nanoTime();
		mThreadIsDone = false;
		mThreadRun = true;
		while(mThreadRun)
		{
			if(!pcIsConnected())
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

					final float gyro[] = getGyroValue();
					final float accel[] = getAccelValue();
					final float mag[] = getMagValue();
					final float att[] = getAttitude();
//					final TextView tvGyro = mTvGyro;
//					final TextView tvAccel= mTvAccel;
//					final TextView tvMag= mTvMag;
//					final TextView tvTime = mTvImgProcTime;
					runOnUiThread(new Runnable(){
						public void run(){ 
							mTvGyro.setText(String.format("Gyro:\t\t%1.2f\t\t%1.2f\t\t%1.2f",gyro[0],gyro[1],gyro[2]));
							mTvAccel.setText(String.format("Accel:\t\t%1.2f\t\t%1.2f\t\t%1.2f",accel[0],accel[1],accel[2]));
							mTvMag.setText(String.format("Mag:\t\t%1.2f\t\t%1.2f\t\t%1.2f",mag[0],mag[1],mag[2]));
							mTvImgProcTime.setText("Proc Time: "+String.valueOf(getImageProcTimeMS())+"ms");
							mTvRoll.setText(String.format("Roll:\t%1.3f",att[0]));
							mTvPitch.setText(String.format("Pitch:\t%1.3f",att[1]));
							mTvYaw.setText(String.format("Yaw:\t%1.3f",att[2]));

							mIvImageDisplay.setImageBitmap(mBitmap); 
						}
					});
				}
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

	/**
	 * Gets the number of cores available in this device, across all processors.
	 * Requires: Ability to peruse the filesystem at "/sys/devices/system/cpu"
	 * @return The number of cores, or 1 if failed to get result
	 *
	 * Taken from
	 * http://makingmoneywithandroid.com/forum/showthread.php?tid=298&pid=1663#pid1663
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
	public native void setAssetManager(AssetManager am);

	static {
//		System.loadLibrary("opencv_core");
//		System.loadLibrary("opencv_imgproc");
//		System.loadLibrary("opencv_flann");
//		System.loadLibrary("opencv_highgui");
//		System.loadLibrary("opencv_features2d");
//		System.loadLibrary("opencv_calib3d");
//		System.loadLibrary("opencv_ml");
//		System.loadLibrary("opencv_video");
//		System.loadLibrary("opencv_objdetect");
//		System.loadLibrary("opencv_contrib");
//		System.loadLibrary("opencv_legacy");
//		System.loadLibrary("opencv_nonfree");
//		System.loadLibrary("opencv_photo");
//		System.loadLibrary("opencv_stitching");
//		System.loadLibrary("opencv_ts");
//		System.loadLibrary("opencv_videostab");
		System.loadLibrary("opencv_java");
		System.loadLibrary("toadlet_egg");
		System.loadLibrary("QuadPhone");
	}
}

