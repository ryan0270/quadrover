#include "Observer_Angular.h"
#include "TNT_Utils.h"

#include <time.h>

using namespace TNT;
using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{

Observer_Angular::Observer_Angular() :
	mGyroBias(3,1,0.0),
	mGyroBiasFirst(3,1,0.0),
	mInnovation(3,1,0.0),
	mLastGyro(3,1,0.0),
	mLastAccel(3,1,0.0),
	mLastMagnometer(3,1,0.0),
	mAccelDirNom(3,1,0.0),
	mMagDirNom(3,1,0.0),
	mAttitudeVicon(3,1,0.0),
	mCurAttitude(3,1,0.0),
	mCurRotMat(3,3,0.0),
	mCurVel(3,1,0.0)
{
	mMutex_all.unlock();
	mRunning = true;

	mGainP = 1;
	mGainI = 0.0; 
	mAccelWeight = 2;
	mMagWeight = 0.3;

	mGainB = 2;
	mIntSat = 0.005;
	// abs(mBias) < mIntSat+mGainI/mGainB*sum(weight)

	mQuadLogger = NULL;

	mBurnInStartTime.clear();
	mBurnInPeriodMS = 0;
	mBurnCount = 0;

	mAccelSensor = NULL;
	mGyroSensor = NULL;
	mMagSensor = NULL;

	
	mAccelDirNom[2][0] = 1; // accel is the opposite direction as gravity
	mMagDirNom[2][0] = 0;

	mDone = true; // need this to be true in case the run thread never gets started

	mUseViconAttitude = false;
}

Observer_Angular::~Observer_Angular()
{
	mMutex_all.unlock();
}

void Observer_Angular::initialize()
{
	mSensorManager = ASensorManager_getInstance();
	ALooper *looper = ALooper_forThread();
//	ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
	const int LOOPER_ID_USER =3; // this is defined in /opt/android-ndk/sources/android/native_app_glue/android_native_app_glue.h but that files isn't in the include path
//	mSensorEventQueue = ASensorManager_createEventQueue(mSensorManager, looper, LOOPER_ID_USER, NULL, NULL);
	mSensorEventQueue = ASensorManager_createEventQueue(mSensorManager, looper, LOOPER_ID_USER, (ALooper_callbackFunc)&inputDetected, NULL);

	mAccelSensor = ASensorManager_getDefaultSensor(mSensorManager, ASENSOR_TYPE_ACCELEROMETER);
	if(mAccelSensor != NULL)
	{
		const char* name = ASensor_getName(mAccelSensor);
		const char* vendor = ASensor_getVendor(mAccelSensor);
		Log::alert(String()+"Accel sensor:\n\t"+name+"\n\t"+vendor);
		ASensorEventQueue_enableSensor(mSensorEventQueue, mAccelSensor);
		ASensorEventQueue_setEventRate(mSensorEventQueue, mAccelSensor, 10*1000); // this is the best it actually achieves
	}
  mGyroSensor = ASensorManager_getDefaultSensor(mSensorManager, ASENSOR_TYPE_GYROSCOPE);
  if(mGyroSensor != NULL)
  {
  	const char* name = ASensor_getName(mGyroSensor);
  	const char* vendor = ASensor_getVendor(mGyroSensor);
  	Log::alert(String()+"Gyro sensor:\n\t"+name+"\n\t"+vendor);
  	ASensorEventQueue_enableSensor(mSensorEventQueue, mGyroSensor);
  	ASensorEventQueue_setEventRate(mSensorEventQueue, mGyroSensor, 5*1000);
  }
	mMagSensor = ASensorManager_getDefaultSensor(mSensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);
	if(mMagSensor != NULL)
	{
		const char* name = ASensor_getName(mMagSensor);
		const char* vendor = ASensor_getVendor(mMagSensor);
		Log::alert(String()+"Mag sensor:\n\t"+name+"\n\t"+vendor);
		ASensorEventQueue_enableSensor(mSensorEventQueue, mMagSensor);
		ASensorEventQueue_setEventRate(mSensorEventQueue, mMagSensor, 10*1000); // this is the best it actually achieves
	}

//	// list out all available sensors
//	const ASensor* const* sensorList;
//	int numSensors = ASensorManager_getSensorList(mSensorManager, &sensorList);
//	for(int i=1; i<numSensors; i++)
//	{
//		const char* name = ASensor_getName(sensorList[i]);
//		const char* vendor=  ASensor_getVendor(sensorList[i]);
//		int type = ASensor_getType(sensorList[i]);
//		String str = "Sensor \n";
//		str = str+"\t"+name+"\n";
//		str = str+"\t"+vendor+"\n";
//		str = str+"\t"+type;
//		Log::alert(str);
//	}

	reset();
}

void Observer_Angular::shutdown()
{
	Log::alert("------------------------- Observer_Angular shutdown started  --------------------------------------------------");
	mRunning = false;
	toadlet::egg::System sys;
Log::alert("------------------------- Observer_Angular shutdown 1 --------------------------------------------------");
	while(!mDone) // since join doesn't seem to work correctly in NDK
		sys.msleep(10);

Log::alert("------------------------- Observer_Angular shutdown 2 --------------------------------------------------");
	if(mMagSensor != NULL)
		ASensorEventQueue_disableSensor(mSensorEventQueue, mMagSensor);
Log::alert("------------------------- Observer_Angular shutdown 3 --------------------------------------------------");
	if(mAccelSensor != NULL)
		ASensorEventQueue_disableSensor(mSensorEventQueue, mAccelSensor);
Log::alert("------------------------- Observer_Angular shutdown 4 --------------------------------------------------");
	if(mGyroSensor != NULL)
		ASensorEventQueue_disableSensor(mSensorEventQueue, mGyroSensor);

Log::alert("------------------------- Observer_Angular shutdown 5 --------------------------------------------------");
	if(mSensorManager != NULL && mSensorEventQueue != NULL)
		ASensorManager_destroyEventQueue(mSensorManager, mSensorEventQueue);

	Log::alert("------------------------- Observer_Angular is donified --------------------------------------------------");
}

void Observer_Angular::reset()
{
	mMutex_all.lock();
	mCurRotMat.inject(createIdentity(3));
	mCurAttitude = extractEulerAngles(mCurRotMat);
	mCurVel.inject(Array2D<double>(3,1,0.0));

//	mGyroBias = Array2D<double>(3,1,0.0);

	mLastGyro.inject(Array2D<double>(3,1,0.0));
	mLastAccel.inject(Array2D<double>(3,1,0.0));
	mLastAccel[2][0] = GRAVITY;
//	mLastMagnometer = Array2D<double>(3,1,0.0);
	mLastMagnometer.inject(mMagDirNom);
	mLastUpdateTime.clear();
	mMutex_all.unlock();
}

void Observer_Angular::run()
{
	mDone = false;
	System sys;
	Time lastInnovationUpdateTime;
	uint64 lastGyroUpdateTimeUS = 0;
	Array2D<double> gyroSum(3,1,0.0);
	Array2D<double> magSum(3,1,0.0);
	Array2D<double> accelSum(3,1,0.0);
	bool haveNewAccel = false;
	bool haveNewMagnometer = false;
	double calMinX = -9.7;
	double calMaxX = 9.65;
	double calMinY = -10.2;
	double calMaxY = 10.05;
	double calMinZ = -10.05;
	double calMaxZ = 9.5;

	double offsetX = 0.5*(calMaxX+calMinX);
	double offsetY = 0.5*(calMaxY+calMinY);
	double offsetZ = 0.5*(calMaxZ+calMinZ);
	double scaleX = 0.5*(calMaxX-calMinX)/GRAVITY;
	double scaleY = 0.5*(calMaxY-calMinY)/GRAVITY;
	double scaleZ = 0.5*(calMaxZ-calMinZ)/GRAVITY;

	while(mRunning)
	{
		ASensorEvent event;
		while(ASensorEventQueue_getEvents(mSensorEventQueue, &event, 1) > 0)
		{
			uint64 curTimeMS;
			uint64 logType = -1;
			switch(event.type)
			{
				case ASENSOR_TYPE_ACCELEROMETER:
					{
						logType = ACCEL;

						mLastAccel[0][0] = event.data[0];
						mLastAccel[1][0] = event.data[1];
						mLastAccel[2][0] = event.data[2];
						haveNewAccel = true;
						
						// for data logging at the bottom
//						event.data[0] = mLastAccel[0][0];
//						event.data[1] = mLastAccel[1][0];
//						event.data[2] = mLastAccel[2][0];
						mMutex_all.unlock();
					}
					break;
				case ASENSOR_TYPE_GYROSCOPE:
					{
						logType = GYRO;

						mMutex_all.lock();
						mLastGyro[0][0] = event.data[0];
						mLastGyro[1][0] = event.data[1];
						mLastGyro[2][0] = event.data[2];

						if(doingBurnIn())
						{
							if(mStartTime.getElapsedTimeMS() > 1000) // ignore early data since it might be junk
							{
								if(mBurnCount == 0)
									Log::alert(String("Beginning burn in."));
								gyroSum += mLastGyro;
								magSum += mLastMagnometer;
								accelSum += mLastAccel;
								mBurnCount++;
							}
							lastInnovationUpdateTime.setTime();
						}
						else if(mBurnCount > 0) // we've finished burn in so calculate bias
						{
							mGyroBias.inject(1.0/mBurnCount*gyroSum);
							mGyroBiasFirst.inject(mGyroBias);
							mMagDirNom.inject(1.0/norm2(magSum)*magSum);

							/////////////////////////// HACK //////////////////////
//							mMagDirNom[0][0] = -21.7;
//							mMagDirNom[1][0] = 5.3;
//							mMagDirNom[2][0] = -40.4;
							mMagDirNom[0][0] = -20.5;
							mMagDirNom[1][0] = 7.0;
							mMagDirNom[2][0] = -39.0;
							mMagDirNom = 1.0/norm2(mMagDirNom)*mMagDirNom;
							/////////////////////////// HACK //////////////////////


							String str = String()+"Burn in done ("+mBurnCount+"): \t";
							for(int i=0; i<3; i++)
								str = str+mGyroBias[i][0]+"\t";
							Log::alert(str);

							str = String()+"mag dir: \t";
							for(int i=0; i<mMagDirNom.dim1(); i++)
								str = str+mMagDirNom[i][0]+"\t";
							Log::alert(str);

							mBurnCount = 0;
						}
						mMutex_all.unlock();
						if(lastGyroUpdateTimeUS > 0)
						{
							double dtUS = event.timestamp/1.0e3-lastGyroUpdateTimeUS;
							doGyroUpdate(dtUS/1.0e6);
						}
						lastGyroUpdateTimeUS = event.timestamp/1.0e3; // timestamp appears to be in nanoseconds
					}
					break;
				case ASENSOR_TYPE_MAGNETIC_FIELD:
					{
						logType = MAGNOMETER;
						mMutex_all.lock();
						mLastMagnometer[0][0] = event.data[0];
						mLastMagnometer[1][0] = event.data[1];
						mLastMagnometer[2][0] = event.data[2];
//						mLastMagnometer = 1.0/norm2(mLastMagnometer)*mLastMagnometer;

//						if(!mHaveFirstMagnometer)
//						{
//							mMagDirNom.inject(matmult(mCurRotMat,mLastMagnometer));
//							mMagDirNom[0][0] = 0;
//							mMagDirNom[1][0] = 1;
//							mMagDirNom[2][0] = 0;
//						}
						haveNewMagnometer = true;
						mMutex_all.unlock();
					}
					break;
				default:
					Log::alert(String()+"Unknown sensor event: "+event.type);
			}

			if(!doingBurnIn() && haveNewAccel && haveNewMagnometer && event.type != 11)
			{
				doInnovationUpdate(lastInnovationUpdateTime.getElapsedTimeUS()/1.0e6);
				lastInnovationUpdateTime.setTime();
				haveNewAccel = false;
				haveNewMagnometer = false;
			}

			if(mQuadLogger != NULL && logType != -1)
			{
				String s=String()+" " + mStartTime.getElapsedTimeMS() + "\t" + event.type+"\t"+event.data[0]+"\t"+event.data[1]+"\t"+event.data[2]+"\t"+event.data[3];
				mQuadLogger->addLine(s,logType);
			}
		}

		sys.msleep(2);
	}

	mDone = true;
	Log::alert("------------------ QuadLogger runner dead --------------------");
}

void Observer_Angular::doInnovationUpdate(double dt)
{
	mMutex_all.lock();
	// orthogonalize the directions (see Hua et al (2011) - Nonlinear attitude estimation with measurement decoupling and anti-windpu gyro-bias compensation)
	Array2D<double> uB = 1.0/norm2(mLastAccel)*mLastAccel;
	Array2D<double> uI = mAccelDirNom;
	/////////////////////////////////////////////////////////////////////
//	Array2D<double> vB = 1.0/norm2(mLastMagnometer)*mLastMagnometer;
//	Array2D<double> vI = mMagDirNom;
	/////////////////////////////////////////////////////////////////////
//	Array2D<double> gravDir = matmult(transpose(mCurRotMat), uI);
//	Array2D<double> vB = cross(gravDir, mLastMagnometer);
//	Array2D<double> vI = cross(uI, mMagDirNom);
//	vI = 1.0/norm2(vI)*vI;
	/////////////////////////////////////////////////////////////////////
//	Array2D<double> vB = cross(uB, mLastMagnometer);
//	vB = 1.0/norm2(vB)*vB;
//	Array2D<double> vI = cross(uI, mMagDirNom);
//	vI = 1.0/norm2(vI)*vI;
	/////////////////////////////////////////////////////////////////////
	Array2D<double> vB = cross(-1.0*mAccelDirNom, mLastMagnometer);
	vB = 1.0/norm2(vB)*vB;
	Array2D<double> vI = cross(-1.0*uI, mMagDirNom);
	vI = 1.0/norm2(vI)*vI;
	/////////////////////////////////////////////////////////////////////

	Array2D<double> transR = transpose(mCurRotMat);
	if(norm2(mLastAccel-mAccelDirNom*GRAVITY) < 3)
	{
		mInnovation = mAccelWeight*cross(uB, matmult(transR, uI));
		mInnovation += mMagWeight*cross(vB, matmult(transR, vI));
	}
	else
		mInnovation = mMagWeight*cross(vB, matmult(transR, vI));

	// add any extra measurements that may have come in
	while(mExtraDirsMeasured.size() > 0)
	{
		double k = mExtraDirsWeight.back();
		Array2D<double> dMeas = mExtraDirsMeasured.back();
		Array2D<double> dInertial = mExtraDirsInertial.back();
		mInnovation += k*cross(dMeas, matmult(transR, dInertial));

		mExtraDirsWeight.pop_back();
		mExtraDirsMeasured.pop_back();
		mExtraDirsInertial.pop_back();
	}

	Time now;
	for(int i=0; i<mGyroBias.dim1(); i++)
		mGyroBias[i][0] += -dt*mGainI*mInnovation[i][0];
	mLastBiasUpdateTime.setTime();

	String s1=String() + mStartTime.getElapsedTimeMS() + "\t" + "-1003" +"\t";
	for(int i=0; i<mGyroBias.dim1(); i++)
		s1 = s1+mGyroBias[i][0] + "\t";
	mMutex_all.unlock();

	mQuadLogger->addLine(s1,OBSV_BIAS);
}

// Based on Hamel and Mahoney's nonlinear SO3 observer
void Observer_Angular::doGyroUpdate(double dt)
{
	if(doingBurnIn())
	{
		mLastUpdateTime.clear();
		mLastBiasUpdateTime.clear();
		return;
	}
	if(mLastUpdateTime.getMS() == 0)
	{
		mLastUpdateTime.setTime();
		return;
	}
	Time curTime;
//	double dt = Time::calcDiffUS(mLastUpdateTime,curTime)/1.0e6;
	mLastUpdateTime.setTime(curTime);

	mMutex_all.lock();
	Array2D<double> gyro;
	mCurVel = mLastGyro - mGyroBias;
	gyro = mCurVel+mGainP*mInnovation;

	Array2D<double> gyro_x = convert_SO3toso3(gyro);

	Array2D<double> A = createIdentity(3); // A is the amount of rotation that has occured
	double velMag = norm2(gyro);
	double s = sin(velMag*dt);
	double c = cos(velMag*dt);
	Array2D<double> gyro_x_sq = matmult(gyro_x,gyro_x);
	if( velMag > 1e-10 )
		A = createIdentity(3)+ s/velMag*gyro_x + (1-c)/velMag/velMag*gyro_x_sq;

	mCurRotMat = matmult(mCurRotMat,A);
	mCurAttitude = extractEulerAngles(mCurRotMat);

	Array2D<double> att;
	if(mUseViconAttitude)
		att = mAttitudeVicon.copy();
	else
		att = mCurAttitude.copy();
///////////////////// temp hack ///////////////////////
att[2][0] = mAttitudeVicon[2][0];
///////////////////// temp hack ///////////////////////

	Array2D<double> vel = mCurVel.copy();
	mMutex_all.unlock();

	for(int i=0; i<mListeners.size(); i++)
		mListeners[i]->onObserver_AngularUpdated(att, vel);
}

Array2D<double> Observer_Angular::convert_so3toSO3(Array2D<double> const &so3)
{
	Array2D<double> SO3(3,1);
	SO3[0][0] = so3[2][1];
	SO3[1][0] = so3[0][2];
	SO3[2][0] = so3[1][0];

	return SO3;
}

Array2D<double> Observer_Angular::convert_SO3toso3(Array2D<double> const &SO3)
{
	Array2D<double> so3(3,3,0.0);
	so3[1][2] = -(so3[2][1] = SO3[0][0]);
	so3[2][0] = -(so3[0][2] = SO3[1][0]);
	so3[0][1] = -(so3[1][0] = SO3[2][0]);

	return so3;
}

Array2D<double> Observer_Angular::extractEulerAngles(Array2D<double> const &rotMat)
{
	Array2D<double> euler(3,1);
	euler[0][0] = atan2(rotMat[2][1], rotMat[2][2]); // roll ... @TODO: this should be range checked
	euler[1][0] = atan2(-rotMat[2][0], sqrt(rotMat[0][0]*rotMat[0][0] + rotMat[1][0]*rotMat[1][0])); // pitch 
	euler[2][0] = atan2(rotMat[1][0], rotMat[0][0]);

	return euler;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread safe data accessors
////////////////////////////////////////////////////////////////////////////////////////////////////
Array2D<double> Observer_Angular::getCurAttitude()
{
	Array2D<double> att;
	mMutex_all.lock();
	if(mUseViconAttitude)
		att = mAttitudeVicon.copy();
	else
		att = mCurAttitude.copy();
	mMutex_all.unlock();

	return att;
}

Array2D<double> Observer_Angular::getCurVel()
{
	Array2D<double> vel;
	mMutex_all.lock();
	vel = mCurVel.copy();
	mMutex_all.unlock();
	return vel;
}

Array2D<double> Observer_Angular::getBias()
{
	Array2D<double> bias;
	mMutex_all.lock();
	bias = mGyroBias.copy();
	mMutex_all.unlock();
	return bias;
}

Array2D<double> Observer_Angular::getLastGyro()
{
	Array2D<double> lastGyro;
	mMutex_all.lock();
	lastGyro = mLastGyro.copy();
	mMutex_all.unlock();
	return lastGyro;
}

Array2D<double> Observer_Angular::getLastAccel()
{
	Array2D<double> lastAccel;
	mMutex_all.lock();
	lastAccel = mLastAccel.copy();
	mMutex_all.unlock();
	return lastAccel;
}

Array2D<double> Observer_Angular::getLastMagnometer()
{
	Array2D<double> lastMag;
	mMutex_all.lock();
	lastMag = mLastMagnometer.copy();
	mMutex_all.unlock();
	return lastMag;
}

void Observer_Angular::setWeights(double accelWeight, double magWeight)
{
	mMutex_all.lock();
	mAccelWeight = accelWeight;
	mMagWeight = magWeight;
	mMutex_all.unlock();
}

void Observer_Angular::addDirectionMeasurement(Array2D<double> const &dirMeas, Array2D<double> const &dirInertial, double weight)
{
	mMutex_all.lock();
	mExtraDirsMeasured.push_back(dirMeas.copy());
	mExtraDirsInertial.push_back(dirInertial.copy());
	mExtraDirsWeight.push_back(weight);
	mMutex_all.unlock();
}

void Observer_Angular::setYawZero()
{
	Array2D<double> temp;
	mMutex_all.lock();
	Array2D<double> rot = createRotMat(2, -mCurAttitude[2][0]);
	mCurRotMat = matmult(rot, mCurRotMat);
	mMagDirNom.inject(1.0/norm2(mLastMagnometer)*mLastMagnometer);
	/////////////////////////// HACK //////////////////////
//	mMagDirNom[2][0] = 0;
//	mMagDirNom = 1.0/norm2(mMagDirNom);
	/////////////////////////// HACK //////////////////////
	temp = mMagDirNom.copy();
	mMutex_all.unlock();

	String str1 = String()+" "+mStartTime.getElapsedTimeMS()+"\t-805\t";
	for(int i=0; i<temp.dim1(); i++)
		str1 = str1+temp[i][0]+"\t";
	mQuadLogger->addLine(str1, PC_UPDATES);
}

void Observer_Angular::onNewCommObserverReset()
{
	reset();
	Log::alert("Observer reset");
	String str = String()+" " + mStartTime.getElapsedTimeMS() + "\t-200\t";
	mQuadLogger->addLine(str,PC_UPDATES);
}

void Observer_Angular::onNewCommObserverGain(double gainP, double gainI, double gravWeight, double compWeight, double gravBandwidth)
{
	setGainP(gainP);
	setGainI(gainI);
	setWeights(gravWeight, compWeight);

	Log::alert("Observer gains updated");
	String str = String()+" " + mStartTime.getElapsedTimeMS() + "\t-210\t";
	str = str+gainP+"\t"+gainI+"\t";
	str = str+gravWeight+"\t"+compWeight;
	mQuadLogger->addLine(str, PC_UPDATES);
}

void Observer_Angular::onNewCommSetYawZero()
{
	setYawZero();
}

void Observer_Angular::onNewCommStateVicon(toadlet::egg::Collection<float> const &data)
{
	Array2D<double> R = matmult(createRotMat(2,-0.25*PI), createRotMat(0,(double)PI));
	mMutex_all.lock();
	for(int i=0; i<3; i++)
		mAttitudeVicon[i][0] = data[i];
	mAttitudeVicon = matmult(R, mAttitudeVicon);
	mMutex_all.unlock();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Sensor event handler
////////////////////////////////////////////////////////////////////////////////////////////////////
int Observer_Angular::inputDetected(int fd, int events, void *data)
{
	return 1;
}

};
};
