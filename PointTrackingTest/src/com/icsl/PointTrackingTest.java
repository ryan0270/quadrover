package com.icsl;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;

public class PointTrackingTest extends android.app.NativeActivity
{
	private static final String ME = "PointTrackingTest";
	private boolean mHasRun = false;

	static
	{
		System.loadLibrary("toadlet_egg");
	}
}
