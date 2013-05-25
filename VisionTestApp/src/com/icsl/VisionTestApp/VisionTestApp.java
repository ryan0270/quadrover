package com.icsl.VisionTestApp;

import android.app.Activity;

public class VisionTestApp extends android.app.NativeActivity
{
	static
	{
		System.loadLibrary("toadlet_egg");
	}
}
