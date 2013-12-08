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

	mDataLogger = NULL;

	mBurnCount = 0;

	mAccelDirNom[2][0] = 1; // accel is the opposite direction as gravity
	mMagDirNom[0][0] = -21.2;
	mMagDirNom[1][0] = 13.4;
	mMagDirNom[2][0] = -35.3;
	normalize(mMagDirNom);

	mDoingBurnIn = true;

	mAccelData = mGyroData = mMagData = NULL;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
	mThreadNiceValue = 0;

	mIsDoingIbvs = false;
	mLastObjectTrackedTime.setTimeMS(0);

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
	setpriority(PRIO_PROCESS, 0, mThreadNiceValue);
	int nice = getpriority(PRIO_PROCESS, 0);
	Log::alert(String()+"Observer_Angular nice value: "+nice);

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

//	String logString1, logString2;
	Array2D<double> uB, uI, vB, vI, dMeas, dInertial;
	mMutex_data.lock();

	if(abs(accel[2][0]-GRAVITY) > 4)
		accel[2][0] = mLastGoodAccel[2][0];
	else
		mLastGoodAccel.inject(accel);

	// orthogonalize the directions (see Hua et al (2011) - Nonlinear attitude estimation with measurement decoupling and anti-windpu gyro-bias compensation)
//	uB = 1.0/norm2(accel)*accel;
//	uI = mAccelDirNom;
//	vB = cross(-1.0*mAccelDirNom, mag);
//	vB = 1.0/norm2(vB)*vB;
//	vI = cross(-1.0*uI, mMagDirNom);
//	vI = 1.0/norm2(vI)*vI;
	normalize(accel);
	normalize(mag);
	uB = accel;
	uI = mAccelDirNom;
	vB = cross(-1.0*mAccelDirNom, mag);
	vI = cross(-1.0*uI, mMagDirNom);
	normalize(vB);
	normalize(vI);

	SO3 transR = mCurAttitude.inv();
	mInnovation = mAccelWeight*cross(uB, transR*uI);
	if(mIsDoingIbvs) // while using Vicon ignore the magnometer
		mInnovation += mMagWeight*cross(vB, transR*vI); 
	mMutex_visionInnovation.lock();
	mInnovation += mVisionInnovation;
	mVisionInnovation = 0.5*mVisionInnovation; // add some decay in case it doesn't get updated
	mMutex_visionInnovation.unlock();

	// add any extra measurements that may have come in
	// Right now this should just be coming from Vicon
//	while(mExtraDirsMeasured.size() > 0)
//	{
//		double k = mExtraDirsWeight.back();
//		dMeas = mExtraDirsMeasured.back();
//		dInertial = mExtraDirsInertial.back();
//		Array2D<double> dir = cross(dMeas, transR*dInertial);
//		mInnovation += k*cross(dMeas, transR*dInertial);
//
//		mExtraDirsWeight.pop_back();
//		mExtraDirsMeasured.pop_back();
//		mExtraDirsInertial.pop_back();
//	}

	mGyroBias[0][0] += dt*mGainI*mInnovation[0][0];
	mGyroBias[1][0] += dt*mGainI*mInnovation[1][0];
	mGyroBias[2][0] += dt*mGainI*mInnovation[2][0];

	mDataLogger->addEntry(LOG_ID_GYRO_BIAS, mGyroBias, LOG_FLAG_OBSV_BIAS);
	mDataLogger->addEntry(LOG_ID_OBSV_ANG_INNOVATION, mInnovation, LOG_FLAG_OBSV_BIAS);
	mMutex_data.unlock();
}

// Based on Hamel and Mahony's nonlinear SO3 observer
void Observer_Angular::doGyroUpdate(double dt, const shared_ptr<DataVector<double>> &gyroData)
{
	if(dt > 0.1)
	{
		Log::alert(String()+"angular observer long dt: " + dt);
		return; // too long of a period to integrate over
	}

	mMutex_data.lock();
	mCurVel.inject(gyroData->dataCalibrated - mGyroBias);
	Time gyroTime( gyroData->timestamp);
	Array2D<double> gyro = mCurVel+mGainP*mInnovation;

	SO3_LieAlgebra gyro_so3(gyro);
	SO3 A = gyro_so3.integrate(dt);
	mCurAttitude = mCurAttitude*A;

	shared_ptr<DataVector<double>> velData(new DataVector<double> );
	velData->type = DATA_TYPE_ANGULAR_VEL;
	velData->timestamp.setTime(gyroTime);
	velData->dataRaw = mCurVel.copy();
	velData->dataCalibrated = mCurVel.copy();

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

	mDataLogger->addEntry(LOG_ID_CUR_ATT, rotData, velData, LOG_FLAG_STATE);
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
	{ mGyroData->lock(); lastGyro = mGyroData->dataCalibrated.copy(); mGyroData->unlock(); }
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
	double curYaw = curAngles[0][0];
	SO3 rot( createRotMat(2, -curYaw) );
//	mCurAttitude= rot*mCurAttitude;
	mCurAttitude *= rot;
	mMagData->lock(); Array2D<double> mag = mMagData->dataCalibrated; mMagData->unlock();
//	mMagData->lock(); Array2D<double> mag = mMagData->data; mMagData->unlock();
	mMagDirNom.inject(1.0/norm2(mag)*mag);
	temp = mMagDirNom.copy();
	mMutex_data.unlock();

	mDataLogger->addEntry(LOG_ID_SET_YAW_ZERO, temp, LOG_FLAG_PC_UPDATES);
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

// This function is way too long, I should probably split it up sometime
void Observer_Angular::onObjectsTracked(const shared_ptr<ObjectTrackerData> &data)
{
//	if(mNominalDirMap.size() == 0 && data->repeatObjects.size() + data->newObjects.size() < 10)
//		return; // don't set my initial nominal angle yet
//
//	mMutex_targetFindTime.lock();
//	mLastObjectTrackedTime.setTime();
//	mMutex_targetFindTime.unlock();
//
//	//////////////////////////// Bookeeping ///////////////////////////////////////////
//	// check for dead objects
//	vector<size_t> deadKeys;
//	unordered_map<size_t, shared_ptr<TrackedObject>>::const_iterator objectIter;
//	for(objectIter = mObjectMap.begin(); objectIter != mObjectMap.end(); objectIter++)
//		if( !objectIter->second->isAlive() )
//			deadKeys.push_back(objectIter->second->getId());
//	sort(deadKeys.begin(), deadKeys.end());
//
//	unordered_map<pair<size_t,size_t>, Array2D<double>, KeyHasher>::iterator angleIter;
//	vector<unordered_map<pair<size_t,size_t>, Array2D<double>, KeyHasher>::iterator> removeIters;
//	for(angleIter = mNominalDirMap.begin(); angleIter != mNominalDirMap.end(); angleIter++)
//		if( binary_search(deadKeys.begin(),deadKeys.end(),angleIter->first.first) ||
//			binary_search(deadKeys.begin(),deadKeys.end(),angleIter->first.second) )
//			removeIters.push_back(angleIter);
//
//
//	unordered_map<pair<size_t,size_t>, Time, KeyHasher>::iterator timeIter;
//	vector<unordered_map<pair<size_t,size_t>, Time, KeyHasher>::iterator> removeTimeIters;
//	for(timeIter = mNominalDirCreateTime.begin(); timeIter != mNominalDirCreateTime.end(); timeIter++)
//		if( binary_search(deadKeys.begin(),deadKeys.end(),timeIter->first.first) ||
//			binary_search(deadKeys.begin(),deadKeys.end(),timeIter->first.second) )
//			removeTimeIters.push_back(timeIter);
//
//	for(int i=0; i<removeIters.size(); i++)
//		mNominalDirMap.erase( removeIters[i] );
//	for(int i=0; i<removeTimeIters.size(); i++)
//		mNominalDirCreateTime.erase( removeTimeIters[i] );
//	for(int i=0; i<deadKeys.size(); i++)
//		mObjectMap.erase( deadKeys[i] );
//
//	// add new stuff
//	vector<shared_ptr<TrackedObject>> repeatObjects;
//	repeatObjects.reserve(data->repeatObjects.size());
//	vector<shared_ptr<TrackedObject>> newObjects = data->newObjects;
//
//	// need to make sure we actually know about all the repeat objects
//	for(int i=0; i<data->repeatObjects.size(); i++)
//		if( mObjectMap.count(data->repeatObjects[i]->getId()) > 0)
//			repeatObjects.push_back(data->repeatObjects[i]);
//		else
//			newObjects.push_back(data->repeatObjects[i]);
//
//	for(int i=0; i<newObjects.size(); i++)
//		mObjectMap[newObjects[i]->getId()] = newObjects[i];
//
//	// assemble everything into one vector
//	vector<shared_ptr<TrackedObject>> objects;
//	objects.reserve(repeatObjects.size()+newObjects.size());
//	objects.insert(objects.end(), repeatObjects.begin(), repeatObjects.end());
//	objects.insert(objects.end(), newObjects.begin(), newObjects.end());
//
//	// go through and see what pairs we already know about
//	vector<pair<size_t,size_t>> repeatPairs, newPairs;
//	vector<pair<cv::Point2f, cv::Point2f>> repeatPairPoints, newPairPoints;
//	for(int i=0; i<objects.size(); i++)
//		for(int j=i+1; j<objects.size(); j++)
//		{
//			int id1 = min(objects[i]->getId(), objects[j]->getId());
//			int id2 = max(objects[i]->getId(), objects[j]->getId());
//
//			pair<size_t,size_t> pr(id1, id2);
//			cv::Point2f pt1 = mObjectMap[id1]->getLocation();
//			cv::Point2f pt2 = mObjectMap[id2]->getLocation();
//			if(mNominalDirMap.count(pr) > 0)
//			{
//				repeatPairs.push_back(pr);
//				repeatPairPoints.push_back( pair<cv::Point2f, cv::Point2f>(pt1, pt2) );
//			}
//			else
//			{
//				double d = cv::norm(pt1-pt2);
//				// avoid keeping pairs that are too close together
//				if(d > 20)
//				{
//					newPairs.push_back(pr);
//					newPairPoints.push_back( pair<cv::Point2f, cv::Point2f>(pt1, pt2) );
//				}
//			}
//		}
//
//	// For rotation compensation
//	SO3 att = data->imageData->att;
//	Array2D<double> curAngles = att.getAnglesZYX();
//	SO3 R_x( createRotMat(0, curAngles[2][0]) );
//	SO3 R_y( createRotMat(1, curAngles[1][0]) );
//	SO3 R_z( createRotMat(2, curAngles[0][0]) );
//	SO3 attYX = R_y*R_x;
//	SO3 rotYX = attYX*SO3(mRotCamToPhone);
//	double f = data->imageData->focalLength;
//	double cx = data->imageData->center.x;
//	double cy = data->imageData->center.y;
//
//	//////////////////////////// add measurements for the pairs we've seen before ///////////////////////////////////////////
//	cv::Point2f p1, p2;
//	Array2D<double> dirMeas(3,1);
//	Array2D<double> dirNom(3,1);
//	Array2D<double> r1(3,1), r2(3,1);
////	vector<Array2D<double>> dirMeasList(repeatPairs.size(),Array2D<double>(3,1));
////	vector<Array2D<double>> dirNomList(repeatPairs.size(),Array2D<double>(3,1));
//	vector<double> offsets(repeatPairs.size());
//	for(int i=0; i<repeatPairs.size(); i++)
//	{
//		p1 = repeatPairPoints[i].first;
//		p2 = repeatPairPoints[i].second;
//
//		dirMeas[0][0] = p2.x-p1.x;
//		dirMeas[1][0] = p2.y-p1.y;
//		dirMeas[2][0] = 0;
//		dirMeas.inject(1.0/norm2(dirMeas)*dirMeas);
//		dirMeas.inject(matmult(mRotCamToPhone, dirMeas));
//
//		dirNom.inject(mNominalDirMap[repeatPairs[i]]);
//
////		dirMeasList[i].inject(dirMeas);
////		dirNomList[i].inject(dirNom);
//
//		// rotation compensation
//		r1[0][0] = p1.x - cx;
//		r1[1][0] = p1.y - cy;
//		r1[2][0] = f;
//
//		r2[0][0] = p2.x - cx;
//		r2[1][0] = p2.y - cy;
//		r2[2][0] = f;
//
//		Array2D<double> delta = rotYX*(r2-r1);
//		double curAngle = atan2(delta[1][0], delta[0][0]);
//		double nomAngle = atan2(dirNom[1][0], dirNom[0][0]);
//		while( curAngle-nomAngle >= PI) curAngle -= 2*PI;
//		while( curAngle-nomAngle < -PI) curAngle += 2*PI;
//		offsets[i] = curAngle-nomAngle;
//	}
//
//	// Find the median offset for outlier rejection
//	double angleOffset = 0;
//	if(offsets.size() > 10)
//	{
//		vector<double> tempOffsets = offsets;
//		size_t medLoc = tempOffsets.size()/2;
////		nth_element(tempOffsets.begin(), tempOffsets.begin()+medLoc, tempOffsets.end());
//		sort(tempOffsets.begin(), tempOffsets.end());
//		double medOffset = tempOffsets[medLoc];
//		if(!mIsDoingIbvs)
//			medOffset = -curAngles[0][0];
//
//		// Now remove outliers
//		int numOutliers = 0;
//		int numInliers = 0;
//		tempOffsets.clear();
//		tempOffsets.swap(offsets);
//		vector<int> killList;
//		for(int i=0; i<tempOffsets.size(); i++)
//		{
//			if( abs(tempOffsets[i]-medOffset) < 0.05)
//			{
//				offsets.push_back(offsets[i]);
//				numInliers++;
//			}
//			else // Remove this pair from history
//			{
//				killList.push_back(i);
//				numOutliers++;
//			}
//		}
//
//		angleOffset = medOffset;
//
//		if( abs(medOffset+curAngles[0][0]) < 0.1 &&
//			(float)numOutliers / numInliers < 1)
//		{
//			Array2D<double> dirMeas(3,1), dirNom(3,1,0.0);
//			dirNom[1][0] = 1;
//			dirNom[0][0] = 0;
//			dirNom[0][0] = 0;
//
//			Array2D<double> R = createRotMat_ZYX(-angleOffset, curAngles[1][0], curAngles[2][0]);
//			dirMeas = matmult(transpose(R), dirNom);
//
//			double weight = 5*2;
//			Array2D<double> innovation = weight*cross(dirMeas, att.inv()*dirNom);
//			angleOffset = medOffset;
//
//			if(mIsDoingIbvs)
//			{
//				mMutex_visionInnovation.lock();
////				mVisionInnovation.inject(innovation);
//				mVisionInnovation[0][0] = 0;
//				mVisionInnovation[1][0] = 0;
//				mVisionInnovation[2][0] = -0.5*2*2*(curAngles[0][0]+medOffset);//innovation[2][0];
//				mMutex_visionInnovation.unlock();
//			}
//
//			for(int i=0; i<killList.size(); i++)
//			{
//				mNominalDirMap.erase(repeatPairs[killList[i]]);
//				mNominalDirCreateTime.erase(repeatPairs[killList[i]]);
//			}
//		}
//		else
//		{
//Log::alert(String()+(int)mStartTime.getElapsedTimeMS()+" -- Bad bunch: " + numOutliers + " vs. " +  numInliers + " and " + abs(medOffset+curAngles[0][0]));
//			for(int i=0; i<repeatPairs.size(); i++)
//			{
//				mNominalDirMap.erase(repeatPairs[i]);
//				mNominalDirCreateTime.erase(repeatPairs[i]);
//			}
//			angleOffset = -curAngles[0][0];
//			mMutex_visionInnovation.lock();
//			mVisionInnovation[0][0] = 0;
//			mVisionInnovation[1][0] = 0;
//			mVisionInnovation[2][0] = 0;
//			mMutex_visionInnovation.unlock();
//
//			for(int i=0; i<repeatPairs.size(); i++)
//			{
//				mNominalDirMap.erase(repeatPairs[i]);
//				mNominalDirCreateTime.erase(repeatPairs[i]);
//			}
//		}
//	}
//	else
//	{
//		angleOffset = -curAngles[0][0];
//		mMutex_visionInnovation.lock();
//		mVisionInnovation[0][0] = 0;
//		mVisionInnovation[1][0] = 0;
//		mVisionInnovation[2][0] = 0;
//		mMutex_visionInnovation.unlock();
//	}
//
//	// If we don't have any repeats just use the current angle
//	// estimate
//	if(mNominalDirMap.size() == 0 && mIsDoingIbvs)
//		angleOffset = -curAngles[0][0];
//
//	/////////////////////////////////////////// set nominal directions for the known pairs ///////////////////////////////////////////
//	Array2D<double> dir(3,1);
//	SO3 rot2( createRotMat(2,angleOffset) );
//	for(int i=0; i<newPairs.size(); i++)
//	{
//		p1 = newPairPoints[i].first;
//		p2 = newPairPoints[i].second;
//		
//		// rotation compensation
//		// Right now we are assuming that the points are on horizontal planes
//		r1[0][0] = p1.x - cx;
//		r1[1][0] = p1.y - cy;
//		r1[2][0] = f;
//
//		r2[0][0] = p2.x - cx;
//		r2[1][0] = p2.y - cy;
//		r2[2][0] = f;
//		Array2D<double> delta =r2-r1;
//		delta = 1.0/norm2(delta)*delta;
//		dir = rot2.inv()*rotYX*delta;
//
//		mNominalDirMap[newPairs[i]] = dir.copy();
//		mNominalDirCreateTime[newPairs[i]] = data->imageData->timestamp;
//	}
//
//	mDataLogger->addEntry(LOG_ID_VISION_INNOVATION, mVisionInnovation, LOG_FLAG_OBSV_BIAS);
}

void Observer_Angular::onNewCommObserverReset()
{
	reset();
	Log::alert("Observer reset");
	mDataLogger->addEntry(LOG_ID_OBSV_ANG_RESET, LOG_FLAG_PC_UPDATES);
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

	Array2D<double> vals(4,1);
	vals[0][0] = gainP;
	vals[1][0] = gainI;
	vals[2][0] = accelWeight;
	vals[3][0] = magWeight;
	mDataLogger->addEntry(LOG_ID_OBSV_ANG_GAINS_UPDATED, vals,LOG_FLAG_PC_UPDATES);
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

//void Observer_Angular::onObserver_TranslationalImageProcessed(const shared_ptr<ImageTranslationData> &data)
//{
//	if(data->goodPoints.size() < 10 || !mIsDoingIbvs)
//		return;
//
//	vector<shared_ptr<TrackedObject>> curObjects = data->goodObjects;
//	vector<cv::Point2f> curPoints(curObjects.size());
//
//	// adjust for roll and pitch
//	// also rotate to phone coords
//	SO3 att = data->objectTrackingData->imageData->att;
//	Array2D<double> curAngles = att.getAnglesZYX();
//	SO3 R_x( createRotMat(0, curAngles[2][0]) );
//	SO3 R_y( createRotMat(1, curAngles[1][0]) );
//	SO3 R_z( createRotMat(2, curAngles[0][0]) );
//	SO3 attYX = R_y*R_x;
//	SO3 rotYX = attYX*SO3(mRotCamToPhone);
//	Array2D<double> p(3,1);
//	double f = data->objectTrackingData->imageData->focalLength;
//	cv::Point2f center = data->objectTrackingData->imageData->center;
//	for(int i=0; i<curObjects.size(); i++)
//	{
//		p[0][0] = curObjects[i]->getLocation().x - center.x;
//		p[1][0] = curObjects[i]->getLocation().y - center.y;
//		p[2][0] = f;
//		p = rotYX*p;
//		curPoints[i].x = p[0][0];
//		curPoints[i].y = p[1][0];
//		curPoints[i] += data->imageOffset;
//	}
//
//
//	vector<cv::Point2f> nomPoints = data->nominalPoints;
//
//	vector<cv::Point2f> curDeltaList, nomDeltaList;
//	vector<double> distList, ageList;
//	for(int i=0; i<curPoints.size(); i++)
//		for(int j=i+1; j<curPoints.size(); j++)
//		{
//			cv::Point2f curDelta = curPoints[j]-curPoints[i];
//			if(curDelta.x > 30 && curDelta.y > 30)
//				continue;
//
//			cv::Point2f nomDelta = nomPoints[j]-nomPoints[i];
//
//			curDeltaList.push_back(curDelta);
//			nomDeltaList.push_back(nomDelta);
//			distList.push_back(norm(curDelta));
//			ageList.push_back(curObjects[i]->getAge()+curObjects[j]->getAge());
//		}
//
//	// Pick out only the furthest apart
//	int numEntries = min(100, (int)curDeltaList.size());
////	vector<int> distOrder(distList.size());
////	for(int i=0; i<distOrder.size(); i++)
////		distOrder[i] = i;
////	sort(distOrder.begin(), distOrder.end(), [&](int i1, int i2){return distList[i1] > distList[i2];});
////	nth_element(distOrder.begin(), distOrder.begin()+numEntries-1, distOrder.end(), [&](int i1, int i2){return distList[i1] > distList[i2];});
//
//	vector<int> ageOrder(ageList.size());
//	for(int i=0; i<ageOrder.size(); i++)
//		ageOrder[i] = i;
//	sort(ageOrder.begin(), ageOrder.end(), [&](int i1, int i2){return ageList[i1] > ageList[i2];});
//
//	vector<int> sortOrder;
////	sortOrder.swap(distOrder);
//	sortOrder.swap(ageOrder);
//
//	Array2D<double> A_T(numEntries,2), B_T(numEntries,2);
//	for(int i=0; i<numEntries; i++)
//	{
//		A_T[i][0] = curDeltaList[sortOrder[i]].x;
//		A_T[i][1] = curDeltaList[sortOrder[i]].y;
//               
//		B_T[i][0] = nomDeltaList[sortOrder[i]].x;
//		B_T[i][1] = nomDeltaList[sortOrder[i]].y;
//	}
//
//	JAMA::LU<double> lu_B_T(B_T);
//	Array2D<double> C_T = lu_B_T.solve(A_T);
//	Array2D<double> C = transpose(submat(C_T,0,1,0,1));
//
//	// need to orthogonalize
//	JAMA::SVD<double> svd_C(C);
//	Array2D<double> U, S, V;
//	svd_C.getU(U);
//	svd_C.getV(V);
//
//	// Based on the Polar decomposition C = UP
//	Array2D<double> rot = matmult(U, transpose(V));
//	
//	// Need to orthogonalize R
//	double angle = acos(rot[0][0]);
//
//	mMutex_visionInnovation.lock();
//	mVisionInnovation[0][0] = 0;
//	mVisionInnovation[1][0] = 0;
//	mVisionInnovation[2][0] = -0.5*(curAngles[0][0] + angle)*2*2*2*2*2*2;
//	mMutex_visionInnovation.unlock();
//}

void Observer_Angular::onNewCommStateVicon(const Collection<float> &data)
{
	mMutex_targetFindTime.lock();
	Time lastTargetFindTime(mLastObjectTrackedTime);
	mMutex_targetFindTime.unlock();
	if(mIsDoingIbvs)// && lastTargetFindTime.getElapsedTimeMS() < 1e3)
		return;

	Array2D<double> nomDir(3,1);
	nomDir[0][0] = 1;
	nomDir[1][0] = 0;
	nomDir[2][0] = 0;
	nomDir = 1.0/norm2(nomDir)*nomDir;

	double viconYaw = -data[2];
	mMutex_data.lock();
	Array2D<double> curAngles = mCurAttitude.getAnglesZYX();
	double curYaw = curAngles[0][0];
	mMutex_data.unlock();

	mMutex_visionInnovation.lock();
	mVisionInnovation[0][0] = 0;
	mVisionInnovation[1][0] = 0;
	mVisionInnovation[2][0] = -0.1*2*2*2*2*2*(curYaw-viconYaw);
	mMutex_visionInnovation.unlock();

	mDataLogger->addEntry(LOG_ID_USE_VICON_YAW, LOG_FLAG_PC_UPDATES);
}

void Observer_Angular::onNewCommUseIbvs(bool useIbvs)
{
	mIsDoingIbvs = true;

	mMutex_cache.lock();
	Array2D<double> mag = mMagData->dataCalibrated.copy();
	mMutex_cache.unlock();

	mMutex_data.lock();
	SO3 att = mCurAttitude;
	mag = att*mag;
	normalize(mag);
	mMagDirNom.inject(mag);
	mMutex_data.unlock();
}

};
};
