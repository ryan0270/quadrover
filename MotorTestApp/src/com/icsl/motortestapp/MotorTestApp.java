package com.icsl.motortestapp;

import android.app.Activity;
import android.os.Bundle;
import android.widget.SeekBar;
import android.widget.TextView;
import android.util.Log;

public class MotorTestApp extends Activity
{
	private final static String ME="MotorTestApp";
	private TextView mTxtMotorVal1, mTxtMotorVal2, mTxtMotorVal3, mTxtMotorVal4;
	private SeekBar mSldMotorVal1, mSldMotorVal2, mSldMotorVal3, mSldMotorVal4;

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
		mSldMotorVal1.setMax( 1<<11 );
		mSldMotorVal2 = (SeekBar)findViewById(R.id.sldMotorVal2);
		mSldMotorVal2.setMax( 1<<11 );
		mSldMotorVal3 = (SeekBar)findViewById(R.id.sldMotorVal3);
		mSldMotorVal3.setMax( 1<<11 );
		mSldMotorVal4 = (SeekBar)findViewById(R.id.sldMotorVal4);
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
}
