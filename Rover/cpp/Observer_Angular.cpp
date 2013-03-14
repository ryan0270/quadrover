#include "Observer_Angular.h"
#include "TNT_Utils.h"

#include <time.h>

using namespace TNT;
using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{

Observer_Angular::Observer_Angular() :
	mGyroBias(3,1,0.0),
	mInnovation(3,1,0.0),
	mGyro(3,1,0.0),
	mAccel(3,1,0.0),
	mMagnometer(3,1,0.0),
	mAccelDirNom(3,1,0.0),
	mMagDirNom(3,1,0.0),
	mCurAttitude(3,1,0.0),
	mCurRotMat(3,3,0.0),
	mCurVel(3,1,0.0)
{
	mNewAccelReady = mNewGyroReady = mNewMagReady = false;
	mRunning = false;;
	mDone = true;

	mGainP = 1;
	mGainI = 0.0; 
	mAccelWeight = 2;
	mMagWeight = 0;

	mGainB = 2;
	mIntSat = 0.005;
	// abs(mBias) < mIntSat+mGainI/mGainB*sum(weight)

	mQuadLogger = NULL;

	mBurnCount = 0;

	mAccelDirNom[2][0] = 1; // accel is the opposite direction as gravity
//	mMagDirNom[0][0] = -20.5;
//	mMagDirNom[1][0] = 7.0;
//	mMagDirNom[2][0] = -39.0;
	mMagDirNom[0][0] = 6;
	mMagDirNom[1][0] = -1;
	mMagDirNom[2][0] = -15;
	mMagDirNom = 1.0/norm2(mMagDirNom)*mMagDirNom;

	mDoingBurnIn = true;

	mYawVicon = 0;

	mAccelData = mGyroData = mMagData = NULL;
}

Observer_Angular::~Observer_Angular()
{
	mMutex_data.unlock();
}

void Observer_Angular::initialize()
{
	reset();
}

void Observer_Angular::shutdown()
{
	Log::alert("------------------------- Observer_Angular shutdown started  --------------------------------------------------");
	mRunning = false;
	toadlet::egg::System sys;
	while(!mDone) // since join doesn't seem to work correctly in NDK
		sys.msleep(10);


	mAccelData = NULL;
	mGyroData = NULL;
	mMagData = NULL;
	Log::alert("------------------------- Observer_Angular is donified --------------------------------------------------");
}

void Observer_Angular::reset()
{
	mMutex_data.lock();
	mCurRotMat.inject(createIdentity(3));
	mCurAttitude = extractEulerAngles(mCurRotMat);
	mCurVel.inject(Array2D<double>(3,1,0.0));

//	mGyroBias = Array2D<double>(3,1,0.0);

	mGyro.inject(Array2D<double>(3,1,0.0));
	mAccel.inject(Array2D<double>(3,1,0.0));
	mAccel[2][0] = GRAVITY;
//	mMagnometer = Array2D<double>(3,1,0.0);
	mMagnometer.inject(mMagDirNom);
	mMutex_data.unlock();
}

void Observer_Angular::run()
{
	mRunning = true;
	mDone = false;
	System sys;
	Time lastInnovationUpdateTime;
	Time lastGyroUpdateTime;
	Array2D<double> gyroSum(3,1,0.0);
	Array2D<double> magSum(3,1,0.0);
	Array2D<double> accelSum(3,1,0.0);
	bool accelProcessed = true; // "processed" means used to calculate the innovation term
	bool magProcessed = true;
	bool gyroProcessed = true;
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
		if(mNewGyroReady)
		{
			mMutex_data.lock();
			mMutex_cache.lock();
			mGyroData->lock();
			mGyro.inject(mGyroData->data);
			mGyroData->unlock();
			mMutex_cache.unlock();
			if(mDoingBurnIn && mBurnCount < 2000)
			{
				gyroSum += mGyro;
				magSum += mMagnometer;
				accelSum += mAccel;
				mBurnCount++;
				if(mBurnCount == 2000)
				{
					mDoingBurnIn = false;
					mGyroBias.inject(1.0/mBurnCount*gyroSum);
					Log::alert(+"Angular observer burn in done");
					printArray("\tGyro bias: \t",transpose(mGyroBias));

					mMutex_cache.lock();
					mGyroData->lock();
					lastGyroUpdateTime.setTime(mGyroData->timestamp);
					mGyroData->unlock();
					mMutex_cache.unlock();
				}
			}
			mNewGyroReady = false;
			mMutex_data.unlock();
			gyroProcessed = false;
		}
		if(mNewAccelReady)
		{
			mMutex_data.lock();
			mMutex_cache.lock();
			mAccelData->lock();
			mAccel.inject(mAccelData->data);
			mAccelData->unlock();
			mMutex_cache.unlock();
			mNewAccelReady = false;
			mMutex_data.unlock();
			accelProcessed = false;
		}
		if(mNewMagReady)
		{
			mMutex_data.lock();
			mMutex_cache.lock();
			mMagData->lock();
			mMagnometer.inject(mMagData->data);
			mMagData->unlock();
			mMutex_cache.unlock();
			mNewMagReady = false;
			mMutex_data.unlock();
			magProcessed = false;
		}

		if(!mDoingBurnIn && !accelProcessed && !magProcessed)
		{
			doInnovationUpdate(lastInnovationUpdateTime.getElapsedTimeUS()/1.0e6);
			lastInnovationUpdateTime.setTime();
			accelProcessed = true;
			magProcessed = true;
		}
		else
		{
			// this is to cover the case when we don't get an update for a long time
			double dt = lastInnovationUpdateTime.getElapsedTimeUS()/1.0e6;
			mInnovation = exp(-70.0*dt)*mInnovation;
		}

		if(!mDoingBurnIn && !gyroProcessed)
		{
			mMutex_cache.lock();
			double dt = Time::calcDiffNS(lastGyroUpdateTime, mGyroData->timestamp)/1.0e9;
			lastGyroUpdateTime.setTime(mGyroData->timestamp);
			mMutex_cache.unlock();
			doGyroUpdate(dt);
			gyroProcessed = true;
		}

		sys.usleep(500);
	}

	mDone = true;
	Log::alert("------------------ QuadLogger runner dead --------------------");
}

void Observer_Angular::doInnovationUpdate(double dt)
{
	mMutex_data.lock();
	// orthogonalize the directions (see Hua et al (2011) - Nonlinear attitude estimation with measurement decoupling and anti-windpu gyro-bias compensation)
	Array2D<double> uB = 1.0/norm2(mAccel)*mAccel;
	Array2D<double> uI = mAccelDirNom;
	Array2D<double> vB = cross(-1.0*mAccelDirNom, mMagnometer);
	vB = 1.0/norm2(vB)*vB;
	Array2D<double> vI = cross(-1.0*uI, mMagDirNom);
	vI = 1.0/norm2(vI)*vI;

	Array2D<double> transR = transpose(mCurRotMat);
	if(norm2(mAccel-mAccelDirNom*GRAVITY) < 2)
//	if( (norm2(mAccel)-GRAVITY) < 2)
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

	if(dt < 0.02)
		for(int i=0; i<mGyroBias.dim1(); i++)
			mGyroBias[i][0] += -dt*mGainI*mInnovation[i][0];

	String s1=String() + mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_GYRO_BIAS +"\t";
	for(int i=0; i<mGyroBias.dim1(); i++)
		s1 = s1+mGyroBias[i][0] + "\t";
	mMutex_data.unlock();

	String s2=String() + mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_OBSV_ANG_INNOVATION +"\t";
	for(int i=0; i<mInnovation.dim1(); i++)
		s2 = s2+mInnovation[i][0] + "\t";
	mMutex_data.unlock();

	mQuadLogger->addLine(s1,LOG_FLAG_OBSV_BIAS);
	mQuadLogger->addLine(s2,LOG_FLAG_OBSV_BIAS);
}

// Based on Hamel and Mahoney's nonlinear SO3 observer
void Observer_Angular::doGyroUpdate(double dt)
{
	if(dt > 0.05)
		return; // too long of a period to integrate over

	mMutex_data.lock();
	Array2D<double> gyro;
	mCurVel = mGyro - mGyroBias;
	gyro = mCurVel+mGainP*mInnovation;

	Array2D<double> gyro_x = convert_coordToso3(gyro);

	Array2D<double> A = createIdentity(3); // A is the amount of rotation that has occured
	double velMag = norm2(gyro);
	double s = sin(velMag*dt);
	double c = cos(velMag*dt);
	Array2D<double> gyro_x_sq = matmult(gyro_x,gyro_x);
	if( velMag > 1e-10 )
		A = createIdentity(3)+ s/velMag*gyro_x + (1-c)/velMag/velMag*gyro_x_sq;

	mCurRotMat = matmult(mCurRotMat,A);
	mCurAttitude = extractEulerAngles(mCurRotMat);

	Array2D<double> att = mCurAttitude.copy();

	Array2D<double> vel = mCurVel.copy();
	mMutex_data.unlock();

	for(int i=0; i<mListeners.size(); i++)
		mListeners[i]->onObserver_AngularUpdated(att, vel);
}

Array2D<double> Observer_Angular::convert_so3toCoord(Array2D<double> const &so3)
{
	Array2D<double> SO3(3,1);
	SO3[0][0] = so3[2][1];
	SO3[1][0] = so3[0][2];
	SO3[2][0] = so3[1][0];

	return SO3;
}

Array2D<double> Observer_Angular::convert_coordToso3(Array2D<double> const &SO3)
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
	mMutex_data.lock();
	att = mCurAttitude.copy();
	mMutex_data.unlock();

	return att;
}

Array2D<double> Observer_Angular::getCurVel()
{
	Array2D<double> vel;
	mMutex_data.lock();
	vel = mCurVel.copy();
	mMutex_data.unlock();
	return vel;
}

Array2D<double> Observer_Angular::getBias()
{
	Array2D<double> bias;
	mMutex_data.lock();
	bias = mGyroBias.copy();
	mMutex_data.unlock();
	return bias;
}

Array2D<double> Observer_Angular::getLastGyro()
{
	Array2D<double> lastGyro;
	mMutex_data.lock();
	lastGyro = mGyro.copy();
	mMutex_data.unlock();
	return lastGyro;
}

Array2D<double> Observer_Angular::getLastAccel()
{
	Array2D<double> lastAccel;
	mMutex_data.lock();
	lastAccel = mAccel.copy();
	mMutex_data.unlock();
	return lastAccel;
}

Array2D<double> Observer_Angular::getLastMagnometer()
{
	Array2D<double> lastMag;
	mMutex_data.lock();
	lastMag = mMagnometer.copy();
	mMutex_data.unlock();
	return lastMag;
}

void Observer_Angular::setWeights(double accelWeight, double magWeight)
{
	mMutex_data.lock();
	mAccelWeight = accelWeight;
	mMagWeight = magWeight;
	mMutex_data.unlock();
}

void Observer_Angular::addDirectionMeasurement(Array2D<double> const &dirMeas, Array2D<double> const &dirInertial, double weight)
{
	mMutex_data.lock();
	mExtraDirsMeasured.push_back(dirMeas.copy());
	mExtraDirsInertial.push_back(dirInertial.copy());
	mExtraDirsWeight.push_back(weight);
	mMutex_data.unlock();
}

void Observer_Angular::setYawZero()
{
	Array2D<double> temp;
	mMutex_data.lock();
	Array2D<double> rot = createRotMat(2, -mCurAttitude[2][0]);
	mCurRotMat = matmult(rot, mCurRotMat);
	mMagDirNom.inject(1.0/norm2(mMagnometer)*mMagnometer);
	temp = mMagDirNom.copy();
	mMutex_data.unlock();

	String str1 = String()+mStartTime.getElapsedTimeMS()+"\t" + LOG_ID_SET_YAW_ZERO + "\t";
	for(int i=0; i<temp.dim1(); i++)
		str1 = str1+temp[i][0]+"\t";
	mQuadLogger->addLine(str1,LOG_FLAG_PC_UPDATES);
}

void Observer_Angular::onNewCommObserverReset()
{
	reset();
	Log::alert("Observer reset");
	String str = String()+ mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_OBSV_ANG_RESET + "\t";
	mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
}

void Observer_Angular::onNewCommAttObserverGain(double gainP, double gainI, double accelWeight, double magWeight)
{
	setGainP(gainP);
	setGainI(gainI);
	setWeights(accelWeight, magWeight);

	{
		String s = "Observer gains updated: \t";
		s = s+gainP+"\t";
		s = s+gainI+"\t";
		s = s+accelWeight+"\t";
		s = s+magWeight+"\t";
		Log::alert(s);
	}
	String str = String()+ mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_OBSV_ANG_GAINS_UPDATED + "\t";
	str = str+gainP+"\t"+gainI+"\t";
	str = str+accelWeight+"\t"+magWeight;
	mQuadLogger->addLine(str,LOG_FLAG_PC_UPDATES);
}

void Observer_Angular::onNewCommNominalMag(Collection<float> const &nomMag)
{
	mMutex_data.lock();
	for(int i=0; i<3; i++)
		mMagDirNom[i][0] = nomMag[i];
	mMagDirNom = 1.0/norm2(mMagDirNom)*mMagDirNom;
	mMutex_data.unlock();

	{
		String s = "Nominal mag updated: \t";
		for(int i=0; i<nomMag.size(); i++)
			s = s+nomMag[i]+"\t";
		Log::alert(s);
	}
}

void Observer_Angular::onNewCommStateVicon(Collection<float> const &data)
{
	mMutex_data.lock();
	mYawVicon = -data[2];

	Array2D<double> R1 = createRotMat(2,-mCurAttitude[2][0]);
	Array2D<double> R2 = createRotMat(2,mYawVicon);
	mCurRotMat = matmult(R2, matmult(R1, mCurRotMat));
	mCurAttitude = extractEulerAngles(mCurRotMat);
	mMutex_data.unlock();
}

void Observer_Angular::onNewSensorUpdate(shared_ptr<SensorData> const &data)
{
	switch(data->type)
	{
		case SENSOR_DATA_TYPE_ACCEL:
			mMutex_cache.lock();
			mAccelData = static_pointer_cast<SensorDataVector>(data);
			mNewAccelReady = true;
			mMutex_cache.unlock();
			break;
		case SENSOR_DATA_TYPE_GYRO:
			mMutex_cache.lock();
			mGyroData = static_pointer_cast<SensorDataVector>(data);
			mNewGyroReady = true;
			mMutex_cache.unlock();
			break;
		case SENSOR_DATA_TYPE_MAG:
			mMutex_cache.lock();
			mMagData = static_pointer_cast<SensorDataVector>(data);
			mNewMagReady = true;
			mMutex_cache.unlock();
			break;
	}
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
