#include "Observer_Angular.h"

namespace ICSL{
namespace Quadrotor{
using namespace TNT;
using namespace ICSL::Constants;

Observer_Angular::Observer_Angular() :
	mGyroBias(3,1,0.0),
	mInnovation(3,1,0.0),
	mAccelDirNom(3,1,0.0),
	mMagDirNom(3,1,0.0),
	mCurVel(3,1,0.0),
	mRotCamToPhone(3,3,0.0),
	mRotPhoneToCam(3,3,0.0),
	mLastGoodAccel(3,1),
	mVisionInnovation(3,1,0.0)
{
	mNewAccelReady = mNewGyroReady = mNewMagReady = false;
	mRunning = false;;
	mDone = true;

	mGainP = 1;
	mGainI = 0.05; 
	mAccelWeight = 5;
	mMagWeight = 1;

	mQuadLogger = NULL;

	mBurnCount = 0;

	mAccelDirNom[2][0] = 1; // accel is the opposite direction as gravity
	mMagDirNom[0][0] = -21.2;
	mMagDirNom[1][0] = 13.4;
	mMagDirNom[2][0] = -35.3;
	mMagDirNom = 1.0/norm2(mMagDirNom)*mMagDirNom;

	mDoingBurnIn = true;

	mAccelData = mGyroData = mMagData = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

	mIsDoingIbvs = false;
	mLastTargetFindTime.setTimeMS(0);

	mRotCamToPhone = matmult(createRotMat(2,-0.5*(double)PI),
							 createRotMat(0,(double)PI));
	mRotPhoneToCam = transpose(mRotCamToPhone);

	mLastGoodAccel[0][0] = 0;
	mLastGoodAccel[1][0] = 0;
	mLastGoodAccel[2][0] = 1;
}

Observer_Angular::~Observer_Angular()
{
}

void Observer_Angular::initialize()
{
	reset();
}

void Observer_Angular::shutdown()
{
	Log::alert("------------------------- Observer_Angular shutdown started  --------------------------------------------------");
	mRunning = false;
	while(!mDone) // since join doesn't seem to work correctly in NDK
		System::msleep(10);


	mAccelData = NULL;
	mGyroData = NULL;
	mMagData = NULL;
	Log::alert("------------------------- Observer_Angular is donified --------------------------------------------------");
}

void Observer_Angular::reset()
{
	mMutex_data.lock();
	mCurVel.inject(Array2D<double>(3,1,0.0));

//	mGyroBias = Array2D<double>(3,1,0.0);

//	mGyro.inject(Array2D<double>(3,1,0.0));
//	mAccel.inject(Array2D<double>(3,1,0.0));
//	mAccel[2][0] = GRAVITY;
//	mMagnometer = Array2D<double>(3,1,0.0);
//	mMagnometer.inject(mMagDirNom);
	mMutex_data.unlock();
}

void Observer_Angular::run()
{
	mRunning = true;
	mDone = false;
	Time lastInnovationUpdateTime;
	Time lastGyroUpdateTime;
	Array2D<double> gyroSum(3,1,0.0);
	Array2D<double> magSum(3,1,0.0);
	Array2D<double> accelSum(3,1,0.0);
	bool accelProcessed = true; // "processed" means used to calculate the innovation term
	bool magProcessed = true;
	bool gyroProcessed = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);
	shared_ptr<DataVector<double>> gyroData, accelData, magData;
	gyroData = accelData = magData = NULL;
	Array2D<double> lastInnovation(3,1,0.0);
	double gyroDT;
	int burnTotal = 2000;
//	int burnTotal = 1000;
	while(mRunning)
	{
		if(mNewGyroReady)
		{
			mMutex_cache.lock();
			gyroData = mGyroData;
			mMutex_cache.unlock();

			mMutex_data.lock();
			if(mDoingBurnIn && mBurnCount < burnTotal)
			{
				if(gyroData != NULL)
				{
					gyroData->lock();
					gyroSum += gyroData->dataCalibrated;
					gyroData->unlock();
				}
				if(magData != NULL) {magData->lock(); magSum += magData->dataCalibrated; magData->unlock();}
				if(accelData != NULL) {accelData->lock(); accelSum += accelData->dataCalibrated; accelData->unlock();}
//				if(gyroData != NULL) {gyroData->lock(); gyroSum += gyroData->data; gyroData->unlock();}
//				if(magData != NULL) {magData->lock(); magSum += magData->data; magData->unlock();}
//				if(accelData != NULL) {accelData->lock(); accelSum += accelData->data; accelData->unlock();}
				mBurnCount++;
				if(mBurnCount == burnTotal)
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

					lastInnovationUpdateTime.setTime();
					mExtraDirsMeasured.clear();
					mExtraDirsInertial.clear();
					mExtraDirsWeight.clear();
				}
			}
			mMutex_data.unlock();
			mNewGyroReady = false;
			gyroProcessed = false;
		}
		if(mNewAccelReady)
		{
			mMutex_cache.lock();
			accelData = mAccelData;
			mMutex_cache.unlock();
			mNewAccelReady = false;
			accelProcessed = false;
		}
		if(mNewMagReady)
		{
			mMutex_cache.lock();
			magData = mMagData;
			mMutex_cache.unlock();
			mNewMagReady = false;
			magProcessed = false;
		}

		bool needInnovation = false;
		mMutex_data.lock();
		if(!mDoingBurnIn && accelData != NULL && magData != NULL && 
			( (!accelProcessed && !magProcessed) ))// || mExtraDirsMeasured.size() > 0))
			needInnovation = true;
		mMutex_data.unlock();
		if(needInnovation)
		{
			double dt = lastInnovationUpdateTime.getElapsedTimeNS()/1.0e9;
			lastInnovationUpdateTime.setTime();
			doInnovationUpdate(dt, accelData, magData);
			mMutex_data.lock();
			lastInnovation.inject(mInnovation);
			mMutex_data.unlock();
			accelProcessed = true;
			magProcessed = true;
		}
		else
		{
			// this is to cover the case when we don't get an update for a long time
			double dt = lastInnovationUpdateTime.getElapsedTimeUS()/1.0e6;
			mInnovation.inject(exp(-50.0*dt)*lastInnovation);
		}

		if(!mDoingBurnIn && !gyroProcessed)
		{
			gyroData->lock();
			gyroDT = Time::calcDiffNS(lastGyroUpdateTime, gyroData->timestamp)/1.0e9;
			lastGyroUpdateTime.setTime(gyroData->timestamp);
			gyroData->unlock();
			doGyroUpdate(gyroDT, gyroData);
			gyroProcessed = true;
		}

		mMutex_SO3Buffer.lock();
		while(mSO3Buffer.size() > 0 && mSO3Buffer.front()->timestamp.getElapsedTimeMS() > 0.5e3)
			mSO3Buffer.pop_front();
		mMutex_SO3Buffer.unlock();

		System::usleep(500);
	}

	mDone = true;
}

void Observer_Angular::doInnovationUpdate(double dt,
										  const shared_ptr<DataVector<double>> &accelData,
										  const shared_ptr<DataVector<double>> &magData)
{
	accelData->lock(); Array2D<double> accel = accelData->dataCalibrated.copy(); accelData->unlock();
	magData->lock(); Array2D<double> mag = magData->dataCalibrated.copy(); magData->unlock();
//	accelData->lock(); Array2D<double> accel = accelData->data.copy(); accelData->unlock();
//	magData->lock(); Array2D<double> mag = magData->data.copy(); magData->unlock();

	String logString1, logString2;
	Array2D<double> uB, uI, vB, vI, dMeas, dInertial;
	mMutex_data.lock();

//	if( abs(accel[2][0]-GRAVITY) < 6 )// && abs(norm2(accel)-GRAVITY) < 6 )
//		mLastGoodAccel.inject(accel);
//	else
//		accel.inject(mLastGoodAccel);
	if(abs(accel[2][0]-GRAVITY) > 6)
//		accel[2][0] = GRAVITY;
		accel[2][0] = mLastGoodAccel[2][0];
	else
		mLastGoodAccel.inject(accel);

	// orthogonalize the directions (see Hua et al (2011) - Nonlinear attitude estimation with measurement decoupling and anti-windpu gyro-bias compensation)
	uB = 1.0/norm2(accel)*accel;
	uI = mAccelDirNom;
	vB = cross(-1.0*mAccelDirNom, mag);
	vB = 1.0/norm2(vB)*vB;
	vI = cross(-1.0*uI, mMagDirNom);
	vI = 1.0/norm2(vI)*vI;

	SO3 transR = mCurAttitude.inv();
	mInnovation = mAccelWeight*cross(uB, transR*uI);
	mInnovation += mMagWeight*cross(vB, transR*vI); 
	mMutex_visionInnovation.lock();
	mInnovation[2][0] += mVisionInnovation[2][0];
	mMutex_visionInnovation.unlock();

	// add any extra measurements that may have come in
	// Right now this should just be coming from Vicon
	while(mExtraDirsMeasured.size() > 0)
	{
		double k = mExtraDirsWeight.back();
		dMeas = mExtraDirsMeasured.back();
		dInertial = mExtraDirsInertial.back();
		Array2D<double> dir = cross(dMeas, transR*dInertial);
		mInnovation += k*cross(dMeas, transR*dInertial);

		mExtraDirsWeight.pop_back();
		mExtraDirsMeasured.pop_back();
		mExtraDirsInertial.pop_back();
	}

	mGyroBias += dt*mGainI*mInnovation;

	for(int i=0; i<mGyroBias.dim1(); i++)
		logString1 = logString1+mGyroBias[i][0] + "\t";

	for(int i=0; i<mInnovation.dim1(); i++)
		logString2 = logString2+mInnovation[i][0] + "\t";
	mMutex_data.unlock();

	mQuadLogger->addEntry(Time(), LOG_ID_GYRO_BIAS, logString1, LOG_FLAG_OBSV_BIAS);
	mQuadLogger->addEntry(Time(), LOG_ID_OBSV_ANG_INNOVATION, logString2, LOG_FLAG_OBSV_BIAS);
}

// Based on Hamel and Mahoney's nonlinear SO3 observer
void Observer_Angular::doGyroUpdate(double dt, const shared_ptr<DataVector<double>> &gyroData)
{
	if(dt > 0.1)
		return; // too long of a period to integrate over

	mMutex_data.lock();
//	gyroData->lock();
	mCurVel.inject(gyroData->dataCalibrated - mGyroBias);
	Time gyroTime( gyroData->timestamp);
//	gyroData->unlock();
	Array2D<double> gyro = mCurVel+mGainP*mInnovation;

	SO3_LieAlgebra gyro_so3(gyro);
	SO3 A = gyro_so3.integrate(dt);
	mCurAttitude = mCurAttitude*A;

	shared_ptr<DataVector<double>> attData(new DataVector<double> );
	attData->type = DATA_TYPE_ATTITUDE;
	attData->timestamp.setTime(gyroTime);
	attData->data = mCurAttitude.getAnglesZYX();

	shared_ptr<DataVector<double>> velData(new DataVector<double> );
	velData->type = DATA_TYPE_ANGULAR_VEL;
	velData->timestamp.setTime(gyroTime);
	velData->data = mCurVel.copy();

	mMutex_SO3Buffer.lock();
	shared_ptr<SO3Data<double>> rotData(new SO3Data<double>());
	rotData->type = DATA_TYPE_SO3;
	rotData->timestamp.setTime(gyroTime);
	rotData->rotation = mCurAttitude;
	mSO3Buffer.push_back(rotData);
	mMutex_SO3Buffer.unlock();

	mMutex_data.unlock();

	for(int i=0; i<mListeners.size(); i++)
		mListeners[i]->onObserver_AngularUpdated(rotData, velData);

	String logStr=String()+Time::calcDiffMS(mStartTime,attData->timestamp)+"\t";
	for(int i=0; i<attData->data.dim1(); i++)
		logStr = logStr+attData->data[i][0] + "\t";
	for(int i=0; i<velData->data.dim1(); i++)
		logStr = logStr+velData->data[i][0]+"\t";
	if(mQuadLogger != NULL)
		mQuadLogger->addEntry(LOG_ID_CUR_ATT, logStr,LOG_FLAG_STATE);
}

Array2D<double> Observer_Angular::convert_so3toCoord(const Array2D<double> &so3)
{
	Array2D<double> SO3(3,1);
	SO3[0][0] = so3[2][1];
	SO3[1][0] = so3[0][2];
	SO3[2][0] = so3[1][0];

	return SO3;
}

Array2D<double> Observer_Angular::convert_coordToso3(const Array2D<double> &SO3)
{
	Array2D<double> so3(3,3,0.0);
	so3[1][2] = -(so3[2][1] = SO3[0][0]);
	so3[2][0] = -(so3[0][2] = SO3[1][0]);
	so3[0][1] = -(so3[1][0] = SO3[2][0]);

	return so3;
}

Array2D<double> Observer_Angular::extractEulerAngles(const Array2D<double> &rotMat)
{
	Array2D<double> euler(3,1);
	euler[0][0] = atan2(rotMat[2][1], rotMat[2][2]); // roll ... @TODO: this should be range checked
	euler[1][0] = atan2(-rotMat[2][0], sqrt(rotMat[0][0]*rotMat[0][0] + rotMat[1][0]*rotMat[1][0])); // pitch 
	euler[2][0] = atan2(rotMat[1][0], rotMat[0][0]);

	return euler;
}

SO3 Observer_Angular::estimateAttAtTime(const Time &t)
{
	mMutex_SO3Buffer.lock();
	SO3 att;
	if(mSO3Buffer.size() > 0)
		att = IData::interpolate(t, mSO3Buffer);
	// else att is the identity
	mMutex_SO3Buffer.unlock();

	return att;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread safe data accessors
////////////////////////////////////////////////////////////////////////////////////////////////////
SO3 Observer_Angular::getCurAttitude()
{
	SO3 att;
	mMutex_data.lock();
	att = mCurAttitude;
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
	Array2D<double> lastGyro(3,1,0.0);
	mMutex_cache.lock();
	if(mGyroData != NULL)
	{ mGyroData->lock(); lastGyro = mGyroData->data.copy(); mGyroData->unlock(); }
	mMutex_cache.unlock(); 
	return lastGyro;
}

Array2D<double> Observer_Angular::getLastAccel()
{
	Array2D<double> lastAccel(3,1,0.0);
	mMutex_cache.lock();
	if(mAccelData != NULL)
	{ mAccelData->lock(); lastAccel = mAccelData->dataCalibrated.copy(); mAccelData->unlock(); }
//	if(mAccelData != NULL)
//	{ mAccelData->lock(); lastAccel = mAccelData->data.copy(); mAccelData->unlock(); }
	mMutex_cache.unlock();
	return lastAccel;
}

Array2D<double> Observer_Angular::getLastMagnometer()
{
	Array2D<double> lastMag(3,1,0.0);
	mMutex_cache.lock();
	if(mMagData != NULL)
	{ mMagData->lock(); lastMag = mMagData->dataCalibrated.copy(); mMagData->unlock(); }
//	if(mMagData != NULL)
//	{ mMagData->lock(); lastMag = mMagData->data.copy(); mMagData->unlock(); }
	mMutex_cache.unlock();
	return lastMag;
}

void Observer_Angular::setWeights(double accelWeight, double magWeight)
{
	mMutex_data.lock();
	mAccelWeight = accelWeight;
	mMagWeight = magWeight;
	mMutex_data.unlock();
}

void Observer_Angular::addDirectionMeasurement(const Array2D<double> &dirMeas, const Array2D<double> &dirInertial, double weight)
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
	Array2D<double> curAngles = mCurAttitude.getAnglesZYX();
	double curYaw = curAngles[2][0];
	SO3 rot( createRotMat(2, -curYaw) );
//	mCurAttitude= rot*mCurAttitude;
	mCurAttitude *= rot;
	mMagData->lock(); Array2D<double> mag = mMagData->dataCalibrated; mMagData->unlock();
//	mMagData->lock(); Array2D<double> mag = mMagData->data; mMagData->unlock();
	mMagDirNom.inject(1.0/norm2(mag)*mag);
	temp = mMagDirNom.copy();
	mMutex_data.unlock();

	String str1;
	for(int i=0; i<temp.dim1(); i++)
		str1 = str1+temp[i][0]+"\t";
	mQuadLogger->addEntry(Time(), LOG_ID_SET_YAW_ZERO, str1, LOG_FLAG_PC_UPDATES);
}

void Observer_Angular::onNewSensorUpdate(const shared_ptr<IData> &data)
{
	switch(data->type)
	{
		case DATA_TYPE_ACCEL:
			mMutex_cache.lock();
			mAccelData = static_pointer_cast<DataVector<double>>(data);
			mNewAccelReady = true;
			mMutex_cache.unlock();
			break;
		case DATA_TYPE_GYRO:
			mMutex_cache.lock();
			mGyroData = static_pointer_cast<DataVector<double>>(data);
			mNewGyroReady = true;
			mMutex_cache.unlock();
			break;
		case DATA_TYPE_MAG:
			mMutex_cache.lock();
			mMagData = static_pointer_cast<DataVector<double>>(data);
			mNewMagReady = true;
			mMutex_cache.unlock();
			break;
	}
}

void Observer_Angular::onTargetFound(const shared_ptr<ImageTargetFindData> &data)
{
//Log::alert("I shouldn't be here (observer angular)");
//	if(data->target == NULL)
//		return;
//
//	mMutex_targetFindTime.lock();
//	mLastTargetFindTime.setTime();
//	mMutex_targetFindTime.unlock();
//
//	cv::Point2f p0 = data->target->squareData[0]->contour[0];
//	cv::Point2f p1 = data->target->squareData[0]->contour[1];
//	cv::Point2f p2 = data->target->squareData[0]->contour[2];
//
//	// long edge of target
//	Array2D<double> measDir1(3,1);
//	measDir1[0][0] = p1.x-p0.x;
//	measDir1[1][0] = p1.y-p0.y;
//	measDir1[2][0] = 0;
//	measDir1 = 1.0/norm2(measDir1)*measDir1;
//	measDir1 = matmult(mRotCamToPhone, measDir1);
//
//	Array2D<double> nomDir1(3,1);
//	nomDir1[0][0] = -1;
//	nomDir1[1][0] = 1;
//	nomDir1[2][0] = 0;
//	nomDir1 = 1.0/norm2(nomDir1)*nomDir1;
//
//	// short edge of target
//	Array2D<double> measDir2(3,1);
//	measDir2[0][0] = p2.x-p1.x;
//	measDir2[1][0] = p2.y-p1.y;
//	measDir2[2][0] = 0;
//	measDir2 = 1.0/norm2(measDir2)*measDir2;
//	measDir2 = matmult(mRotCamToPhone, measDir2);
//
//	Array2D<double> nomDir2(3,1);
//	nomDir2[0][0] = 1;
//	nomDir2[1][0] = 1;
//	nomDir2[2][0] = 0;
//	nomDir2 = 1.0/norm2(nomDir2)*nomDir2;
//
//	mMutex_data.lock();
//	mExtraDirsMeasured.push_back(measDir1.copy());
//	mExtraDirsMeasured.push_back(measDir2.copy());
//	mExtraDirsInertial.push_back(nomDir1.copy());
//	mExtraDirsInertial.push_back(nomDir2.copy());
//	mExtraDirsWeight.push_back(2*2);
//	mExtraDirsWeight.push_back(2*2);
//	mMutex_data.unlock();
}

// This function is way too long, I should probably split it up sometime
void Observer_Angular::onTargetFound2(const shared_ptr<ImageTargetFind2Data> &data)
{
	if(mNominalDirMap.size() == 0 && data->repeatRegions.size() + data->newRegions.size() < 5)
		return; // don't set my initial nominal angle yet

	mMutex_targetFindTime.lock();
	mLastTargetFindTime.setTime();
	mMutex_targetFindTime.unlock();

	//////////////////////////// Bookeeping ///////////////////////////////////////////
	// check for dead regions
	vector<size_t> deadKeys;
	unordered_map<size_t, shared_ptr<ActiveRegion>>::const_iterator regionIter;
	for(regionIter = mRegionMap.begin(); regionIter != mRegionMap.end(); regionIter++)
		if( regionIter->second->getLife() <= 0)
			deadKeys.push_back(regionIter->second->getId());
	sort(deadKeys.begin(), deadKeys.end());

	unordered_map<pair<size_t,size_t>, Array2D<double>, KeyHasher>::iterator angleIter;
	vector<unordered_map<pair<size_t,size_t>, Array2D<double>, KeyHasher>::iterator> removeIters;
	for(angleIter = mNominalDirMap.begin(); angleIter != mNominalDirMap.end(); angleIter++)
		if( binary_search(deadKeys.begin(),deadKeys.end(),angleIter->first.first) ||
			binary_search(deadKeys.begin(),deadKeys.end(),angleIter->first.second) )
			removeIters.push_back(angleIter);


	unordered_map<pair<size_t,size_t>, Time, KeyHasher>::iterator timeIter;
	vector<unordered_map<pair<size_t,size_t>, Time, KeyHasher>::iterator> removeTimeIters;
	for(timeIter = mNominalDirCreateTime.begin(); timeIter != mNominalDirCreateTime.end(); timeIter++)
		if( binary_search(deadKeys.begin(),deadKeys.end(),timeIter->first.first) ||
			binary_search(deadKeys.begin(),deadKeys.end(),timeIter->first.second) )
			removeTimeIters.push_back(timeIter);

	for(int i=0; i<removeIters.size(); i++)
		mNominalDirMap.erase( removeIters[i] );
	for(int i=0; i<removeTimeIters.size(); i++)
		mNominalDirCreateTime.erase( removeTimeIters[i] );
	for(int i=0; i<deadKeys.size(); i++)
		mRegionMap.erase( deadKeys[i] );

	// add new stuff
	vector<shared_ptr<ActiveRegion>> repeatRegions;
	repeatRegions.reserve(data->repeatRegions.size());
	vector<shared_ptr<ActiveRegion>> newRegions = data->newRegions;

	// need to make sure we actually know about all the repeat regions
	for(int i=0; i<data->repeatRegions.size(); i++)
		if( mRegionMap.count(data->repeatRegions[i]->getId()) > 0)
			repeatRegions.push_back(data->repeatRegions[i]);
		else
			newRegions.push_back(data->repeatRegions[i]);

	for(int i=0; i<newRegions.size(); i++)
		mRegionMap[newRegions[i]->getId()] = newRegions[i];

	// assemble everything into one vector
	vector<shared_ptr<ActiveRegion>> regions;
	regions.reserve(repeatRegions.size()+newRegions.size());
	regions.insert(regions.end(), repeatRegions.begin(), repeatRegions.end());
	regions.insert(regions.end(), newRegions.begin(), newRegions.end());

	// go through and see what pairs we already know about
	vector<pair<size_t,size_t>> repeatPairs, newPairs;
	vector<pair<cv::Point2f, cv::Point2f>> repeatPairPoints, newPairPoints;
	for(int i=0; i<regions.size(); i++)
		for(int j=i+1; j<regions.size(); j++)
		{
			int id1 = min(regions[i]->getId(), regions[j]->getId());
			int id2 = max(regions[i]->getId(), regions[j]->getId());

			pair<size_t,size_t> pr(id1, id2);
			cv::Point2f pt1 = mRegionMap[id1]->getFoundPos();
			cv::Point2f pt2 = mRegionMap[id2]->getFoundPos();
			if(mNominalDirMap.count(pr) > 0)
			{
				repeatPairs.push_back(pr);
				repeatPairPoints.push_back( pair<cv::Point2f, cv::Point2f>(pt1, pt2) );
			}
			else
			{
				double d = cv::norm(pt1-pt2);
				// avoid keeping pairs that are too close together
				if(d > 40)
				{
					newPairs.push_back(pr);
					newPairPoints.push_back( pair<cv::Point2f, cv::Point2f>(pt1, pt2) );
				}
			}
		}

	// For rotation compensation
	SO3 att = data->imageData->att;
	Array2D<double> euler = att.getAnglesZYX();
	SO3 R_x( createRotMat(0, euler[0][0]) );
	SO3 R_y( createRotMat(1, euler[1][0]) );
	SO3 R_z( createRotMat(2, euler[2][0]) );
//	SO3 attYX = R_x*R_y; // This is what Daewan had in his paper but I don't think it's right
	SO3 attYX = R_y*R_x; // R_y*R_x*R_x'*R_y'*R_z'*P = R_z'*P
	SO3 rotYX = attYX*SO3(mRotCamToPhone);
	double f = data->imageData->focalLength;
	double cx = data->imageData->center.x;
	double cy = data->imageData->center.y;

	//////////////////////////// add measurements for the pairs we've seen before ///////////////////////////////////////////
	cv::Point2f p1, p2;
	Array2D<double> dirMeas(3,1);
	Array2D<double> dirNom(3,1);
	Array2D<double> r1(3,1), r2(3,1);
	vector<Array2D<double>> dirMeasList(repeatPairs.size(),Array2D<double>(3,1));
	vector<Array2D<double>> dirNomList(repeatPairs.size(),Array2D<double>(3,1));
	vector<double> offsets(repeatPairs.size());
	for(int i=0; i<repeatPairs.size(); i++)
	{
		p1 = repeatPairPoints[i].first;
		p2 = repeatPairPoints[i].second;

		dirMeas[0][0] = p2.x-p1.x;
		dirMeas[1][0] = p2.y-p1.y;
		dirMeas[2][0] = 0;
		dirMeas.inject(1.0/norm2(dirMeas)*dirMeas);
		dirMeas.inject(matmult(mRotCamToPhone, dirMeas));

		dirNom.inject(mNominalDirMap[repeatPairs[i]]);

		dirMeasList[i].inject(dirMeas);
		dirNomList[i].inject(dirNom);

		// rotation compensation
		r1[0][0] = p1.x - cx;
		r1[1][0] = p1.y - cy;
		r1[2][0] = f;

		r2[0][0] = p2.x - cx;
		r2[1][0] = p2.y - cy;
		r2[2][0] = f;

		Array2D<double> delta = rotYX*(r2-r1);
		double curAngle = atan2(delta[1][0], delta[0][0]);
		double nomAngle = atan2(dirNom[1][0], dirNom[0][0]);
		while( curAngle-nomAngle >= PI) curAngle -= 2*PI;
		while( curAngle-nomAngle < -PI) curAngle += 2*PI;
		offsets[i] = curAngle-nomAngle;
	}

	// Find the median offset for outlier rejection
	double angleOffset = 0;
	if(offsets.size() > 0)
	{
		vector<double> tempOffsets = offsets;
		size_t medLoc = tempOffsets.size()/2;
//		nth_element(tempOffsets.begin(), tempOffsets.begin()+medLoc, tempOffsets.end());
		sort(tempOffsets.begin(), tempOffsets.end());
		double medOffset = tempOffsets[medLoc];

		// Now remove outliers
		int numOutliers = 0;
		int numInliers = 0;
		vector<Array2D<double>> tempDirMeasList, tempDirNomList;
		tempDirMeasList.swap(dirMeasList);
		tempDirNomList.swap(dirNomList);
		tempOffsets.clear();
		tempOffsets.swap(offsets);
		vector<double> lifeList;
		double totalLife = 0;
		int kingIndex = -1;
		int kingLife = -1;
		for(int i=0; i<tempOffsets.size(); i++)
		{
			if( abs(tempOffsets[i]-medOffset) < 0.1 )
			{
				double s = mNominalDirCreateTime[repeatPairs[i]].getElapsedTimeMS();
				double e = s;
//				double e = exp(-10*pow(tempOffsets[i]+euler[2][0],2));
				totalLife += e;
				lifeList.push_back(e);
				dirMeasList.push_back(tempDirMeasList[i].copy());
				dirNomList.push_back(tempDirNomList[i].copy());
				angleOffset += s*tempOffsets[i];
				offsets.push_back(offsets[i]);
				numInliers++;

				if(s > kingLife)
				{
					kingIndex = offsets.size()-1;
					kingLife = s;
				}
			}
			else // Remove this pair from history
			{
				mNominalDirMap.erase(repeatPairs[i]);
				mNominalDirCreateTime.erase(repeatPairs[i]);
				numOutliers++;
			}
		}

		angleOffset = offsets[kingIndex];

		if( abs(medOffset+euler[2][0]) < 0.2)
		{
			Array2D<double> dirMeas(3,1), dirNom(3,1,0.0);
			dirNom[1][0] = 1;
			dirNom[0][0] = 0;
			dirNom[0][0] = 0;

			Array2D<double> R = createRotMat_ZYX(-medOffset, euler[1][0], euler[0][0]);
			dirMeas = matmult(transpose(R), dirNom);

			double weight = 0.2;
			Array2D<double> innovation = weight*cross(dirMeas, att.inv()*dirNom);
			angleOffset = medOffset;


			mMutex_visionInnovation.lock();
			mVisionInnovation[0][0] = 0;
			mVisionInnovation[1][0] = 0;
			mVisionInnovation[2][0] = weight*(medOffset+euler[2][0]);
			mMutex_visionInnovation.unlock();
		}
		else
		{
			angleOffset = -euler[2][0];
			mMutex_visionInnovation.lock();
			mVisionInnovation[0][0] = 0;
			mVisionInnovation[1][0] = 0;
			mVisionInnovation[2][0] = 0;
			mMutex_visionInnovation.unlock();
		}
	}

	// If we don't have any repeats just use the current angle
	// estimate
	if(mNominalDirMap.size() == 0)
		angleOffset = -euler[2][0];
	else if(dirMeasList.size() == 0)
		angleOffset = -euler[2][0];

	/////////////////////////////////////////// set nominal directions for the known pairs ///////////////////////////////////////////
	Array2D<double> dir(3,1);
	SO3 rot2( createRotMat(2,angleOffset) );
	for(int i=0; i<newPairs.size(); i++)
	{
		p1 = newPairPoints[i].first;
		p2 = newPairPoints[i].second;
		
		// rotation compensation
		// Right now we are assuming that the points are on horizontal planes
		r1[0][0] = p1.x - cx;
		r1[1][0] = p1.y - cy;
		r1[2][0] = f;

		r2[0][0] = p2.x - cx;
		r2[1][0] = p2.y - cy;
		r2[2][0] = f;
		Array2D<double> delta =r2-r1;
		delta = 1.0/norm2(delta)*delta;
		dir = rot2.inv()*rotYX*delta;

		mNominalDirMap[newPairs[i]] = dir.copy();
		mNominalDirCreateTime[newPairs[i]] = data->imageData->timestamp;
	}
}

void Observer_Angular::onNewCommObserverReset()
{
	reset();
	Log::alert("Observer reset");
	String str = String();
	mQuadLogger->addEntry(Time(), LOG_ID_OBSV_ANG_RESET, str,LOG_FLAG_PC_UPDATES);
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
	String str;
	str = str+gainP+"\t"+gainI+"\t";
	str = str+accelWeight+"\t"+magWeight;
	mQuadLogger->addEntry(Time(), LOG_ID_OBSV_ANG_GAINS_UPDATED, str,LOG_FLAG_PC_UPDATES);
}

void Observer_Angular::onNewCommNominalMag(const Collection<float> &nomMag)
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

void Observer_Angular::onNewCommStateVicon(const Collection<float> &data)
{
	mMutex_targetFindTime.lock();
	Time lastTargetFindTime(mLastTargetFindTime);
	mMutex_targetFindTime.unlock();
	if(/*mIsDoingIbvs &&*/ lastTargetFindTime.getElapsedTimeMS() < 1e3)
		return;

	Array2D<double> nomDir(3,1);
	nomDir[0][0] = 1;
	nomDir[1][0] = 0;
	nomDir[2][0] = 0;
	nomDir = 1.0/norm2(nomDir)*nomDir;

	double viconYaw = -data[2];
	mMutex_data.lock();
	Array2D<double> curAngles = mCurAttitude.getAnglesZYX();
	mMutex_data.unlock();

	// Most accurate would be to rotate nomDir by the full Vicon
	// attitude, but that would make my roll and pitch
	// artificially more accurate. Instead, I'm using the current 
	// roll and pitch estimate such that they will cancel out when
	// I go to calculated the innovation and am left with (mostly) 
	// just yaw.
	Array2D<double> R = createRotMat_ZYX(viconYaw, curAngles[1][0], curAngles[0][0]);
	Array2D<double> measDir = matmult(transpose(R),nomDir);

	mMutex_data.lock();
	mExtraDirsMeasured.push_back(measDir.copy());
	mExtraDirsInertial.push_back(nomDir.copy());
	mExtraDirsWeight.push_back(5);
	mMutex_data.unlock();

//	mMutex_data.lock();
//	double yawVicon = -data[2];
//
//	Array2D<double> curAngles = mCurAttitude.getAnglesZYX();
//	double curYaw = curAngles[2][0];
//	SO3 R1(createRotMat(2,-curYaw));
//	SO3 R2(createRotMat(2,yawVicon));
////	mCurAttitude = R2*R1*mCurAttitude;
//	mCurAttitude *= R2*R1;
//	mMutex_data.unlock();
}

};
};
