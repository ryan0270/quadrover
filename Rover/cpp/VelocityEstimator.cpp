#include "VelocityEstimator.h"

namespace ICSL {
namespace Quadrotor {
//using namespace std;
using namespace TNT;
//using namespace ICSL::Constants;

VelocityEstimator::VelocityEstimator() : 
	mRotPhoneToCam(3,3,0.0),
	mRotCamToPhone(3,3,0.0)
{
	mDone = true;
	mRunning = false;

	mNewImageDataAvailable = false;
	mNewRegionDataAvailable = false;
	mLastImageFeatureData = NULL;
	mLastRegionData = NULL;

	mMeasCov = 2*pow(5,2);
	mProbNoCorr = 0.1;

	mLastDelayTimeUS = 0;
}

VelocityEstimator::~VelocityEstimator()
{
}

void VelocityEstimator::shutdown()
{
	Log::alert("------------------------- VelocityEstimator shutdown started");
	mRunning = false;
	while(mDone != true)
		System::msleep(10);
	Log::alert("------------------------- VelocityEstimator shutdown done");
}

void VelocityEstimator::initialize()
{
	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
	mThreadNiceValue = 0;
}

void VelocityEstimator::run()
{
	mDone = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);
	setpriority(PRIO_PROCESS, 0, mThreadNiceValue);
	int nice = getpriority(PRIO_PROCESS, 0);
	Log::alert(String()+"VelocityEstimator nice value: "+nice);

	// Keep these in memory to reduce the time it takes to 
	// allocate
	vector<cv::Point2f> oldPoints, curPoints;
	vector<Array2D<double>> mDeltaList, SDeltaList;
	vector<pair<Array2D<double>, Array2D<double>>> priorDistList;
	vector<Array2D<double>> SdInvmdList, SaInvList, SaList;
	Array2D<double> C(50,50);
	vector<Array2D<double>> LvList, q1HatList, AjList;

	shared_ptr<ImageFeatureData> oldImageFeatureData, curImageFeatureData;
	oldImageFeatureData = curImageFeatureData = NULL;
	shared_ptr<ImageRegionData> oldRegionData, curRegionData;
	oldRegionData = curRegionData = NULL;
	Time procTimer;
	double procTime, delayTime;
	Array2D<double> velEst(3,1,0.0);
	double heightEst = 0.05;;
	String logString;
	float measCov, probNoCorr;
	Time curTime;
	while(mRunning)
	{
		bool success = false;
		if(mNewImageDataAvailable)
		{
			procTimer.setTime();

			mMutex_data.lock();
			oldImageFeatureData = curImageFeatureData;
			curImageFeatureData = mLastImageFeatureData;
			mNewImageDataAvailable = false;
			mMutex_data.unlock();

			curTime.setTime(curImageFeatureData->imageData->timestamp);
			
			if(oldImageFeatureData != NULL && oldImageFeatureData->featurePoints.size() > 5 && curImageFeatureData->featurePoints.size() > 5)
			{
				mMutex_params.lock();
				measCov = mMeasCov;
				probNoCorr = mProbNoCorr;
				mMutex_params.unlock();

				success = doVelocityEstimate(oldImageFeatureData, curImageFeatureData, velEst, heightEst,
							measCov, probNoCorr, oldPoints, curPoints, mDeltaList, SDeltaList, priorDistList,
							SdInvmdList, SaInvList, SaList, C, LvList, q1HatList, AjList);
			}

			procTime = procTimer.getElapsedTimeNS()/1.0e9;
		}
		else if(mNewRegionDataAvailable) // we will do only one of these on any given pass
		{
			procTimer.setTime();

			mMutex_data.lock();
			oldRegionData = curRegionData;
			curRegionData = mLastRegionData;
			mNewRegionDataAvailable = false;
			mMutex_data.unlock();

			curTime.setTime(curRegionData->imageData->timestamp);
			
			if(oldRegionData != NULL && oldRegionData->regionCentroids.size() > 5 &&
			   curRegionData->regionCentroids.size() > 5)
			{
				mMutex_params.lock();
				// HACK: These should have their own, separate variables
				measCov = 4*mMeasCov;
				probNoCorr = 0.1*mProbNoCorr;
				mMutex_params.unlock();

				success = doVelocityEstimate(oldRegionData, curRegionData, velEst, heightEst, measCov, probNoCorr);
			}

			procTime = procTimer.getElapsedTimeNS()/1.0e9;
		}

		if(success)
		{
			shared_ptr<DataVector<double>> velData(new DataVector<double>());
			velData->dataRaw = velEst.copy();
			velData->dataCalibrated = velEst.copy();
			velData->type = DATA_TYPE_MAP_VEL;
			velData->timestamp.setTime(curTime);

			shared_ptr<Data<double>> heightData(new Data<double>());
			heightData->dataRaw = heightEst;
			heightData->dataCalibrated = heightEst;
			heightData->type = DATA_TYPE_MAP_HEIGHT;
			heightData->timestamp.setTime(curTime);

			for(int i=0; i<mListeners.size(); i++)
				mListeners[i]->onVelocityEstimator_newEstimate(velData, heightData);

			delayTime = curTime.getElapsedTimeNS()/1.0e9;
			mMutex_data.lock();
			mLastDelayTimeUS = delayTime*1.0e6;
			mMutex_data.unlock();

			if(mDataLogger != NULL)
			{
				mDataLogger->addEntry(LOG_ID_MAP_VEL, velEst, LOG_FLAG_CAM_RESULTS);
				mDataLogger->addEntry(LOG_ID_MAP_HEIGHT, heightEst, LOG_FLAG_CAM_RESULTS);
				mDataLogger->addEntry(LOG_ID_MAP_VEL_CALC_TIME, procTime, LOG_FLAG_CAM_RESULTS);
				mDataLogger->addEntry(LOG_ID_OPTIC_FLOW_VELOCITY_DELAY, delayTime, LOG_FLAG_CAM_RESULTS);
			}
		}

		System::msleep(1);
	}

	mDone = true;
}

// See eqn 98 in the Feb 25, 2013 notes and Bayesian velocity paper
bool VelocityEstimator::doVelocityEstimate(const shared_ptr<ImageFeatureData> oldFeatureData,
										   const shared_ptr<ImageFeatureData> curFeatureData,
										   Array2D<double> &velEst, 
										   double &heightEst,
										   double visionMeasCov,
										   double probNoCorr,
										   vector<cv::Point2f> &oldPoints,
										   vector<cv::Point2f> &curPoints,
										   vector<Array2D<double>> &mDeltaList,
										   vector<Array2D<double>> &SDeltaList,
										   vector<pair<Array2D<double>, Array2D<double>>> &priorDistList,
										   vector<Array2D<double>> &SdInvmdList,
										   vector<Array2D<double>> &SaInvList,
										   vector<Array2D<double>> &SaList,
										   Array2D<double> &C,
										   vector<Array2D<double>> &LvList,
										   vector<Array2D<double>> &q1HatList,
										   vector<Array2D<double>> &AjList) const
{
	int N1 = oldFeatureData->featurePoints.size();
	int N2 = curFeatureData->featurePoints.size();

	Time oldTime = oldFeatureData->imageData->timestamp;
	Time curTime = curFeatureData->imageData->timestamp;
	double dt = Time::calcDiffNS(oldTime, curTime)/1.0e9;

	cv::Point2f center;
	center.x = curFeatureData->imageData->center.x;
	center.y = curFeatureData->imageData->center.y;
	float focalLength = curFeatureData->imageData->focalLength;

	// Get relevant state data
	Array2D<double> oldState = mObsvTranslational->estimateStateAtTime(oldTime);
	SO3 attOld = oldFeatureData->imageData->att;

	Array2D<double> curState = mObsvTranslational->estimateStateAtTime(curTime);
	Array2D<double> curErrCov = mObsvTranslational->estimateErrCovAtTime(curTime);
	SO3 attCur = curFeatureData->imageData->att;

	if(N1 == 0 || N2 == 0)
	{
		Log::alert("Velocity estimator failed to get the state");
		return false;
	}
	else if(oldFeatureData->featurePoints.size() == 0 ||
	   		curFeatureData->featurePoints.size() == 0)
	{
		heightEst = curState[2][0];
		velEst = submat(curState,3,5,0,0);
		return false;
	}

	SO3 attChange = attCur*attOld.inv();
	Array2D<double> omega = 1.0/dt*attChange.log().toVector();

	// Ignore this case since it means we're probably sitting on the ground
	if(oldState[2][0] <= 0)
		return false; 
	curState = submat(curState,0,5,0,0);
	curErrCov = submat(curErrCov,0,5,0,5);

	// Rotate data to cam coords
	omega.inject( matmult( mRotPhoneToCam, omega ));
	curState.inject(matmult( mRotPhoneToCam2, curState));
	curErrCov.inject(matmult( mRotPhoneToCam2, matmult(curErrCov, mRotCamToPhone2)));

	// get prior distributions
//	vector<cv::Point2f> oldPoints(oldFeatureData->featurePoints.size());
	oldPoints.clear(); oldPoints.resize(N1);
	for(int i=0; i<oldPoints.size(); i++)
		oldPoints[i] = oldFeatureData->featurePoints[i]-center;

//	vector<cv::Point2f> curPoints(curFeatureData->featurePoints.size());
	curPoints.clear(); curPoints.resize(N2);
	for(int i=0; i<curPoints.size(); i++)
		curPoints[i] = curFeatureData->featurePoints[i]-center;
	Array2D<double> mv = submat(curState,3,5,0,0);
	Array2D<double> Sv = submat(curErrCov,3,5,3,5);
	double mz = max(-curState[2][0], 0.130);
	double varz = curErrCov[2][2];

	double camOffset = 0;
	mz -= camOffset;

	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = visionMeasCov;
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

//	priorDistList = calcPriorDistributions(oldPoints, mv, Sv, mz, varz, focalLength, dt, omega);
	calcPriorDistributions(mDeltaList, SDeltaList, priorDistList, oldPoints, mv, Sv, mz, varz, focalLength, dt, omega);

	// Make soft point correspondences
//	Array2D<double> C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv, probNoCorr);
	calcCorrespondence(C, priorDistList, curPoints, Sn, SnInv, SdInvmdList, SaInvList, SaList, probNoCorr);

	double numMatches = 0;
	for(int i=0; i<N1; i++)
		for(int j=0; j<N2; j++)
			numMatches += C[i][j];
	mDataLogger->addEntry(LOG_ID_MAP_NUM_MATCHES, numMatches, LOG_FLAG_CAM_RESULTS);
	
	// Find map vel
	Array2D<double> vel(3,1), covVel(3,3);
	double z;
//	computeMAPEstimate(vel, covVel, z, oldPoints, curPoints, C, mv, Sv, mz, varz, Sn, focalLength, dt, omega);
	computeMAPEstimate(vel, covVel, z, oldPoints, curPoints, C, mv, Sv,
						LvList, q1HatList, AjList, mz, varz, Sn, focalLength, dt, omega);
	z += camOffset;

	vel = matmult(mRotCamToPhone, vel);
	covVel = matmult(mRotCamToPhone, matmult(covVel, mRotPhoneToCam));

	velEst.inject(vel);
	heightEst = z;

	return true;
}

// TODO: combine this with the above to be one function that just operates on locations
bool VelocityEstimator::doVelocityEstimate(const shared_ptr<ImageRegionData> oldRegionData,
										   const shared_ptr<ImageRegionData> curRegionData,
										   Array2D<double> &velEst, 
										   double &heightEst,
										   double visionMeasCov,
										   double probNoCorr) const
{
	Time oldTime = oldRegionData->imageData->timestamp;
	Time curTime = curRegionData->imageData->timestamp;
	double dt = Time::calcDiffNS(oldTime, curTime)/1.0e9;

	cv::Point2f center;
	center.x = curRegionData->imageData->center.x;
	center.y = curRegionData->imageData->center.y;
	float focalLength = curRegionData->imageData->focalLength;

	// Get relevant state data
	Array2D<double> oldState = mObsvTranslational->estimateStateAtTime(oldTime);
	SO3 attOld = oldRegionData->imageData->att;

	Array2D<double> curState = mObsvTranslational->estimateStateAtTime(curTime);
	Array2D<double> curErrCov = mObsvTranslational->estimateErrCovAtTime(curTime);
	SO3 attCur = curRegionData->imageData->att;

	if(oldState.dim1() == 0 || curState.dim1() == 0)
	{
		Log::alert("Velocity estimator failed to get the state");
		return false;
	}

Log::alert("Velocity estimation from regions temporarily disabled");
heightEst = curState[2][0];
velEst = submat(curState,3,5,0,0);
return false;

//	SO3 attChange = attCur*attOld.inv();
//	double theta;
//	Array2D<double> axis;
//	attChange.getAngleAxis(theta, axis);
//	Array2D<double> omega = theta/dt*axis;
//
//	// Ignore this case since it means we're probably sitting on the ground
//	if(oldState[2][0] <= 0)
//		return false; 
//	curState = submat(curState,0,5,0,0);
//	curErrCov = submat(curErrCov,0,5,0,5);
//
//	// Rotate data to cam coords
//	omega.inject( matmult( mRotPhoneToCam, omega ));
//	Array2D<double> mRotPhoneToCam2 = blkdiag(mRotPhoneToCam, mRotPhoneToCam);
//	Array2D<double> mRotCamToPhone2 = blkdiag(mRotCamToPhone, mRotCamToPhone);
//	curState = matmult( mRotPhoneToCam2, curState);
//	curErrCov = matmult( mRotPhoneToCam2, matmult(curErrCov, mRotCamToPhone2));
//
//	// get prior distributions
//	vector<cv::Point2f> oldPoints = oldRegionData->regionCentroids;
//	vector<cv::Point2f> curPoints = curRegionData->regionCentroids;
//
//	Array2D<double> mv = submat(curState,3,5,0,0);
//	Array2D<double> Sv = submat(curErrCov,3,5,3,5);
//	double mz = max(-curState[2][0], 0.130);
//	double sz = sqrt(curErrCov[2][2]);
//
//	double camOffset = 0;
//	mz -= camOffset;
//
//	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
//	Sn[0][0] = Sn[1][1] = visionMeasCov;
//	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];
//
//	vector<pair<Array2D<double>, Array2D<double>>> priorDistList(oldPoints.size());
//	priorDistList = calcPriorDistributions(oldPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);
//
//	// Make soft point correspondences
//	Array2D<double> C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv, probNoCorr);
//
//	double numMatches = 0;
//	for(int i=0; i<C.dim1()-1; i++)
//		for(int j=0; j<C.dim2()-1; j++)
//			numMatches += C[i][j];
//	mDataLogger->addEntry(LOG_ID_MAP_NUM_MATCHES, numMatches, LOG_FLAG_CAM_RESULTS);
//	
//	// Find map vel
//	Array2D<double> vel(3,1), covVel(3,3);
//	double z;
//	computeMAPEstimate(vel, covVel, z, oldPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
//	z += camOffset;
//
//	vel = matmult(mRotCamToPhone, vel);
//	covVel = matmult(mRotCamToPhone, matmult(covVel, mRotPhoneToCam));
//
//	velEst.inject(vel);
//	heightEst = z;
//
//	return true;
}

void VelocityEstimator::onFeaturesFound(const shared_ptr<ImageFeatureData> &data)
{
	mMutex_data.lock();
	mLastImageFeatureData = data;
	mNewImageDataAvailable = true;
	mMutex_data.unlock();
}

void VelocityEstimator::onRegionsFound(const shared_ptr<ImageRegionData> &data)
{
	mMutex_data.lock();
	mLastRegionData= data;
	mNewRegionDataAvailable = true;
	mMutex_data.unlock();
}

vector<pair<Array2D<double>, Array2D<double>>> VelocityEstimator::calcPriorDistributions(
												const vector<cv::Point2f> &points, 
												const Array2D<double> &mv,
												const Array2D<double> &Sv, 
												double mz, double varz, 
												double focalLength, double dt, 
												const Array2D<double> &omega)
{
	vector<Array2D<double>> mDeltaList(points.size()), SDeltaList(points.size());
	vector<pair<Array2D<double>, Array2D<double>>> priorDistList(mDeltaList.size());

	calcPriorDistributions(mDeltaList, SDeltaList, priorDistList, points, mv, Sv, mz, varz, focalLength, dt, omega);

	return priorDistList;
}

void VelocityEstimator::calcPriorDistributions(vector<Array2D<double>> &mDeltaList,
											   vector<Array2D<double>> &SDeltaList,
											   vector<pair<Array2D<double>, Array2D<double>>> &priorDistList,
											   const vector<cv::Point2f> &points, 
											   const Array2D<double> &mv,
											   const Array2D<double> &Sv, 
											   double mz, double varz, 
											   double focalLength, double dt, 
											   const Array2D<double> &omega)
{
	double mvx = mv[0][0];
	double mvy = mv[1][0];
	double mvz = mv[2][0];
	double svx = sqrt(Sv[0][0]);
	double svy = sqrt(Sv[1][1]);
	double svz = sqrt(Sv[2][2]);
	double sz = sqrt(varz);
	double f = focalLength;
	double fInv = 1.0/f;

	// delta_x = q_x*v_z*dt-f*v_x*dt
//	vector<Array2D<double>> mDeltaList(points.size()), SDeltaList(points.size());
	mDeltaList.clear(); mDeltaList.resize(points.size());
	SDeltaList.clear(); SDeltaList.resize(points.size());
	double x, y;
	Array2D<double> mDelta(2,1), SDelta(2,2,0.0);
	for(int i=0; i<points.size(); i++)
	{
		x = points[i].x;
		y = points[i].y;

		mDelta[0][0] = x*mvz*dt-f*mvx*dt;
		mDelta[1][0] = y*mvz*dt-f*mvy*dt;

		SDelta[0][0] = pow(x*svz*dt,2)+pow(f*svx*dt, 2);
		SDelta[1][1] = pow(y*svz*dt,2)+pow(f*svy*dt, 2);

		mDeltaList[i] = mDelta.copy();
		SDeltaList[i] = SDelta.copy();
	}

	// calc distribution of Z^-1
	double mz1Inv = 1.0/mz;
	double mz2Inv = 1.0/mz/mz;
	double log_sz = log(sz);
	double log_mz = log(mz);
	double del1LnOld = DBL_MAX;
	double del2LnOld = DBL_MAX;
	double del1Ln = 0;
	double del2Ln = 0;
	double del1, del2;
	double fact;
	int stopK = 0;
	for(int k=1; k<2; k++)
	{
		fact = fact2ln(k);
		del1Ln = 2*k*log_sz+fact-(2*k+1)*log_mz;
		del2Ln = log(2*k+1)+2*k*log_sz+fact-(2*k+2)*log_mz;
		if(del1Ln > del1LnOld || del2Ln > del2LnOld)
			break;

		del1 = exp(del1Ln);
		del2 = exp(del2Ln);

		mz1Inv = mz1Inv+del1;
		del1LnOld = del1Ln;

		mz2Inv = mz2Inv+del2;
		del2LnOld = del2Ln;

		stopK = k;
	}
	if(stopK == 1 || mz2Inv < mz1Inv*mz1Inv)
	{
		mz1Inv = 1.0/mz;
		mz2Inv = 1.0/mz/mz+sz*sz/pow(mz,4);
	}

	// E(Zinv^2) should always be >= E(Z)^2
	if(mz2Inv < mz1Inv*mz1Inv)
		Log::alert(String()+"crap, mz2Inv < mz1Inv*mz1Inv");

	// calc distribution moments
//	vector<pair<Array2D<double>, Array2D<double>>> priorDistList(mDeltaList.size());
	priorDistList.clear(); priorDistList.resize(points.size());
	Array2D<double> md(2,1), Sd(2,2,0.0), Lw(2,3);
	for(int i=0; i<mDeltaList.size(); i++)
	{
		mDelta = mDeltaList[i];
		SDelta = SDeltaList[i];
		md = mDelta*mz1Inv;

		// Shift the mean to so the distribution is for the location of q2 instead of q2-q1-Lw*w
		x = points[i].x;
		y = points[i].y;

		Lw[0][0] = fInv*x*y; 		Lw[0][1] = -(f+fInv*x*x); 	Lw[0][2] = y;
		Lw[1][0] = f+fInv*y*y;		Lw[1][1] = -fInv*x*y;		Lw[1][2] = -x;
		md = md+dt*matmult(Lw, omega);

		md[0][0] += x;
		md[1][0] += y;

		Sd[0][0] = (pow(mDelta[0][0],2)+SDelta[0][0])*mz2Inv-pow(mDelta[0][0],2)*pow(mz1Inv,2);
		Sd[1][1] = (pow(mDelta[1][0],2)+SDelta[1][1])*mz2Inv-pow(mDelta[1][0],2)*pow(mz1Inv,2);

		Sd[0][1] = Sd[1][0] = x*y*pow(dt,2)*pow(svz,2)*mz2Inv + mDelta[0][0]*mDelta[1][0]*(mz2Inv-pow(mz1Inv,2));

		priorDistList[i] = pair<Array2D<double>, Array2D<double>>(md.copy(), Sd.copy());
	}

//	return priorDistList;
}

Array2D<double> VelocityEstimator::calcCorrespondence(const vector<pair<Array2D<double>, Array2D<double>>> &priorDistList,
													  const vector<cv::Point2f> &curPointList,
													  const Array2D<double> &Sn,
													  const Array2D<double> &SnInv,
													  float probNoCorr)
{
	int N1 = priorDistList.size();
	int N2 = curPointList.size();
	Array2D<double> C;
	vector<Array2D<double>> SdInvmdList(N1), SaInvList(N1), SaList(N1);
	calcCorrespondence(C, priorDistList, curPointList, Sn, SnInv, SdInvmdList, SaInvList, SaList, probNoCorr);

	return C;
}

void  VelocityEstimator::calcCorrespondence(Array2D<double> &C,
											const vector<pair<Array2D<double>, Array2D<double>>> &priorDistList,
											const vector<cv::Point2f> &curPointList,
											const Array2D<double> &Sn,
											const Array2D<double> &SnInv,
											vector<Array2D<double>> &SdInvmdList,
											vector<Array2D<double>> &SaInvList,
											vector<Array2D<double>> &SaList,
											float probNoCorr)
{
	int N1 = priorDistList.size();
	int N2 = curPointList.size();

	if(N1 == 0 || N2 == 0)
	{
		for(int i=0; i<C.dim1(); i++)
			for(int j=0; j<C.dim2(); j++)
				C[i][j] = 0;
		return;
	}

	// Precompute some things
//	vector<Array2D<double>> SdInvmdList(N1), SaInvList(N1), SaList(N1);
	SdInvmdList.clear(); SdInvmdList.resize(N1);	
	SaInvList.clear(); SaInvList.resize(N1);	
	SaList.clear(); SaList.resize(N1);	
	vector<double> fBList(N1), coeffList(N1), xRangeList(N1), yRangeList(N1);
	Array2D<double> eye2 = createIdentity((double)2.0);
	double det_Sn = Sn[0][0]*Sn[1][1] - Sn[0][1]*Sn[1][0];
	Array2D<double> md(2,1), Sd(2,2), SdInv(2,2), Sa(2,2), SaInv(2,2);
	Array2D<double> S(2,2), V(2,2,0.0), D(2,2,0.0);
	double den;
	double fB, det_Sd, det_Sa, coeff, xrange, yrange;
	double theta1, theta2, r1, r2;
	double eigMid, eigOffset;
	for(int i=0; i<N1; i++)
	{
		md.inject(priorDistList[i].first);
		Sd.inject(priorDistList[i].second);

		den = Sd[0][0]*Sd[1][1]-Sd[0][1]*Sd[1][0];
		SdInv[0][0] = Sd[1][1]/den;
		SdInv[0][1] = -Sd[0][1]/den;
		SdInv[1][0] = SdInv[0][1];
		SdInv[1][1] = Sd[0][0]/den;
		
		SaInv.inject(SdInv+SnInv);
		den = SaInv[0][0]*SaInv[1][1]-SaInv[0][1]*SaInv[1][0];
		Sa[0][0] = SaInv[1][1]/den;
		Sa[0][1] = -SaInv[0][1]/den;
		Sa[1][0] = Sa[0][1];
		Sa[1][1] = SaInv[0][0]/den;

//		fB = matmultS(transpose(md),matmult(SdInv,md));
		fB = SdInv[0][0]*md[0][0]*md[0][0] + 2.0*SdInv[0][1]*md[0][0]*md[1][0] + SdInv[1][1]*md[1][0]*md[1][0];
		det_Sd = Sd[0][0]*Sd[1][1] - Sd[0][1]*Sd[1][0];
		det_Sa = Sa[0][0]*Sa[1][1] - Sa[0][1]*Sa[1][0];
		coeff = sqrt(det_Sa)/2.0/PI/sqrt(det_Sd*det_Sn);

		S.inject(Sd+Sn);
		eigMid = 0.5*(S[0][0]+S[1][1]);
		eigOffset = 0.5*sqrt( (S[0][0]-S[1][1])*(S[0][0]-S[1][1]) + 4*S[0][1]*S[0][1] );
		D[0][0] = eigMid-eigOffset;
		D[1][1] = eigMid+eigOffset;
		if(S[0][1] == 0)
		{
			if(S[0][0] > S[1][1])
			{
				V[0][0] = 1;
				V[1][0] = 0;
				V[0][1] = 0;
				V[1][1] = 1;
			}
			else
			{
				V[0][0] = 0;
				V[1][0] = 1;
				V[0][1] = 1;
				V[1][1] = 0;
			}
		}
		else
		{
			V[0][0] = D[0][0]-S[1][1];
			V[1][0] = S[0][1];
			V[0][1] = V[1][0];
			V[1][1] = -V[0][0];
		}

		theta1 = atan2(V[1][0], V[0][0]);
		theta2 = atan2(V[1][1], V[1][0]);
		r1 = 3.0*sqrt(D[0][0]);
		r2 = 3.0*sqrt(D[1][1]);

		while(theta1 < -PI/2.0)	theta1 += PI;
		while(theta1 > PI/2.0) 	theta1 -= PI;
		while(theta2 < 0) 		theta2 += PI;
		while(theta2 > PI) 		theta2 -= PI;

		xrange = max( abs(r1*cos(theta1)), abs(r2*cos(theta2)) );
		yrange = max( abs(r1*sin(theta1)), abs(r2*sin(theta2)) );
//		xrange = max( r1*cos(theta1), r2*cos(theta2) );
//		yrange = max( r1*sin(theta1), r2*sin(theta2) );

		SdInvmdList[i] = matmult(SdInv, md);
		SaInvList[i] = SaInv.copy();
		SaList[i] = Sa.copy();
		fBList[i] = fB;
		coeffList[i] = coeff;
		xRangeList[i] = xrange;
		yRangeList[i] = yrange;
	}

//	Array2D<double> C(N1+1, N2+1);
	if(C.dim1() < N1+1 || C.dim2() < N2+1)
	{
		int N = max(C.dim1(), max(C.dim2(), max(N1+1, N2+1)));
		C = Array2D<double>(N, N);
		Log::alert(String()+"Reallocating velocity estimator correspondence matrix: " + C.dim1() + "x" + C.dim2());
	}
	vector<pair<int, int>> chad;
	double x, y, fC, f;
	for(int j=0; j<N2; j++)
	{
		x = curPointList[j].x;
		y = curPointList[j].y;
		for(int i=0; i<N1; i++)
		{
			md.inject(priorDistList[i].first);
			if(abs(x-md[0][0]) < xRangeList[i] && abs(y-md[1][0]) < yRangeList[i] )
				chad.push_back(make_pair(i, j));
			else
				C[i][j] = 0;
		}
	}

	double peakCoeff = 0;
	Array2D<double> mq(2,1), SnInvmq(2,1), ma(2,1), temp1(2,1);
	for(int idx=0; idx<chad.size(); idx++)
	{
		int i=chad[idx].first;
		int j=chad[idx].second;
		mq[0][0] = curPointList[j].x;
		mq[1][0] = curPointList[j].y;

		SnInvmq[0][0] = SnInv[0][0]*mq[0][0]; // assumes Sn diagonal
		SnInvmq[1][0] = SnInv[1][1]*mq[1][0];

//		double fC = matmultS(transpose(mq),matmult(SnInv,mq));
		fC = SnInv[0][0]*mq[0][0]*mq[0][0] + SnInv[1][1]*mq[1][0]*mq[1][0]; // assumes Sn is diagonal
		
		md.inject(priorDistList[i].first);
//		ma = matmult(SaList[i],SdInvmdList[i]+SnInvmq);
		temp1.inject(SdInvmdList[i]+SnInvmq);
		ma[0][0] = SaList[i][0][0]*temp1[0][0] + SaList[i][0][1]*temp1[1][0];
		ma[1][0] = SaList[i][1][0]*temp1[0][0] + SaList[i][1][1]*temp1[1][0];
//		double f = -1.0*matmultS(transpose(ma),matmult(SaInvList[i],ma))+fBList[i]+fC;
		SaInv.inject(SaInvList[i]);
		f = -1.0*(SaInv[0][0]*ma[0][0]*ma[0][0] + 2.0*SaInv[0][1]*ma[0][0]*ma[1][0] + SaInv[1][1]*ma[1][0]*ma[1][0])
			+ fBList[i] + fC;
		C[i][j] = coeffList[i]*exp(-0.5*f);
		peakCoeff = max(peakCoeff,coeffList[i]);
	}

	for(int j=0; j<N2; j++)
		C[N1][j] = probNoCorr;

	C[N1][N2] = 0;

	// Scale colums to unit sum
	double colSum;
	for(int j=0; j<N2; j++)
	{
		colSum = 0;
		for(int i=0; i<N1+1; i++)
			colSum += C[i][j];

		for(int i=0; i<N1+1; i++)
			if(C[i][j] != 0)
				C[i][j] /= colSum;
	}

	// Now check if any of the rows sum to over 1
	double rowSum;
	for(int i=0; i<N1; i++)
	{
		rowSum = 0;
		for(int j=0; j<N2; j++)
			rowSum += C[i][j];

		if(rowSum > 1)
		{
			for(int j=0; j<N2; j++)
				if(C[i][j] != 0)
					C[i][j] /= rowSum;

			C[i][N2] = 0;
		}
		else
			C[i][N2] = 1-rowSum;
	}

	// and, finally, reset the virtual point correspondence so columns sum to 1 again
	for(int j=0; j<N2; j++)
	{
		double colSum = 0;
		for(int i=0; i<N1; i++)
			colSum += C[i][j];
		C[N1][j] = 1-colSum;
	}
}

void VelocityEstimator::computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
						const vector<cv::Point2f> &prevPoints,
						const vector<cv::Point2f> &curPoints, 
						const Array2D<double> &C, // correspondence matrix
						const Array2D<double> &mv, // velocity mean
						const Array2D<double> &Sv, // velocity covariance
						double mz, // height mean
						double vz, // height variance
						const Array2D<double> &Sn, // feature measurement covariance
						double focalLength, double dt, const Array2D<double> &omega)
{
	int N1 = prevPoints.size();
	int N2 = curPoints.size();
	vector<Array2D<double>> LvList(N1), q1HatList(N1), AjList(N2);
	computeMAPEstimate(velMAP, covVel, heightMAP, prevPoints, curPoints, C, mv, Sv, LvList, q1HatList,
					   AjList, mz, vz, Sn, focalLength, dt, omega, -1);
}

void VelocityEstimator::computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
						const vector<cv::Point2f> &prevPoints,
						const vector<cv::Point2f> &curPoints, 
						const Array2D<double> &C, // correspondence matrix
						const Array2D<double> &mv, // velocity mean
						const Array2D<double> &Sv, // velocity covariance
						double mz, // height mean
						double vz, // height variance
						const Array2D<double> &Sn, // feature measurement covariance
						double focalLength, double dt, const Array2D<double> &omega,
						int maxPointCnt)
{
	int N1 = prevPoints.size();
	int N2 = curPoints.size();
	vector<Array2D<double>> LvList(N1), q1HatList(N1), AjList(N2);
	computeMAPEstimate(velMAP, covVel, heightMAP, prevPoints, curPoints, C, mv, Sv, LvList, q1HatList,
					   AjList, mz, vz, Sn, focalLength, dt, omega, maxPointCnt);
}

void VelocityEstimator::computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
						const vector<cv::Point2f> &prevPoints,
						const vector<cv::Point2f> &curPoints, 
						const Array2D<double> &C, // correspondence matrix
						const Array2D<double> &mv, // velocity mean
						const Array2D<double> &Sv, // velocity covariance
						vector<Array2D<double>> &LvList, // temp variable
						vector<Array2D<double>> &q1HatList,
						vector<Array2D<double>> &AjList,
						double mz, // height mean
						double vz, // height variance
						const Array2D<double> &Sn, // feature measurement covariance
						double focalLength, double dt, const Array2D<double> &omega)
{
	computeMAPEstimate(velMAP, covVel, heightMAP, prevPoints, curPoints, C, mv, Sv, LvList, q1HatList,
					   AjList, mz, vz, Sn, focalLength, dt, omega, -1);
}

void VelocityEstimator::computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
						const vector<cv::Point2f> &prevPoints,
						const vector<cv::Point2f> &curPoints, 
						const Array2D<double> &C, // correspondence matrix
						const Array2D<double> &mv, // velocity mean
						const Array2D<double> &Sv, // velocity covariance
						vector<Array2D<double>> &LvList, // temp variable
						vector<Array2D<double>> &q1HatList,
						vector<Array2D<double>> &AjList,
						double mz, // height mean
						double vz, // height variance
						const Array2D<double> &Sn, // feature measurement covariance
						double focalLength, double dt, const Array2D<double> &omega,
						int maxPointCnt)
{
	int N1 = prevPoints.size();
	int N2 = curPoints.size();
	double f = focalLength;
	double fInv = 1.0/focalLength;

	if(N1 == 0 || N2 == 0)
	{
		velMAP = mv.copy();
		heightMAP = mz;
		return;
	}

	JAMA::Cholesky<double> chol_Sv(Sv), chol_Sn(Sn);
	Array2D<double> SvInv = chol_Sv.solve(createIdentity((double)3.0));
	Array2D<double> SnInv = chol_Sn.solve(createIdentity((double)2.0));

	double sz = sqrt(vz);

	///////////////////////////////////////////////////////////////
	// Build up constant matrices
	///////////////////////////////////////////////////////////////
//	vector<Array2D<double>> LvList(N1), q1HatList(N1);
	LvList.clear(); LvList.resize(N1);
	q1HatList.clear(); q1HatList.resize(N1);
	Array2D<double> Lv(2,3), Lw(2,3), q1(2,1);
	double x, y;
	for(int i=0; i<N1; i++)
	{
		x = prevPoints[i].x;
		y = prevPoints[i].y;

		Lv[0][0] = -f; Lv[0][1] = 0;  Lv[0][2] = x;
		Lv[1][0] = 0;  Lv[1][1] = -f; Lv[1][2] = y;
		LvList[i] = Lv.copy();

		Lw[0][0] = fInv*x*y; 		Lw[0][1] = -(f+fInv*x*x); 	Lw[0][2] = y;
		Lw[1][0] = f+fInv*y*y;		Lw[1][1] = -fInv*x*y;		Lw[1][2] = -x;

		q1[0][0] = x;
		q1[1][0] = y;
		q1HatList[i] = q1+dt*matmult(Lw, omega);
	}

//	vector<Array2D<double>> AjList(N2);
	AjList.clear(); AjList.resize(N2);
	Array2D<double> Aj(2,3);
	for(int j=0; j<N2; j++)
	{
		for(int i=0; i<Aj.dim1(); i++)
			for(int j=0; j<Aj.dim2(); j++)
				Aj[i][j] = 0;
		for(int i=0; i<N1; i++)
		{
			if(C[i][j] > 0.01)
			{
//				Aj += C[i][j]*LvList[i];
				Aj[0][0] += C[i][j]*LvList[i][0][0];
				Aj[0][2] += C[i][j]*LvList[i][0][2];
				Aj[1][1] += C[i][j]*LvList[i][1][1];
				Aj[1][2] += C[i][j]*LvList[i][1][2];
			}
		}
		Aj = dt*Aj;

		AjList[j] = Aj.copy();
	}

	double s0 = 0;
	Array2D<double> s1_T(1,3,0.0), S2(3,3,0.0);
	Array2D<double> q2(2,1), ds1_T(1,3), dS2(3,3);
	Array2D<double> temp1(2,1), temp2(2,3);
	double ds0;
	for(int j=0; j<N2; j++)
	{
		// this 0.01 is arbitrary, just to avoid
		// doing calcs that will have little impact on
		// the final estimate
		if( (1-C[N1][j]) > 0.01)
		{
			Array2D<double> q2(2,1);
			q2[0][0] = curPoints[j].x;
			q2[1][0] = curPoints[j].y;
			temp1[0][0] = temp1[1][0] = 0;
			for(int i=0; i<N1; i++)
			{
				if(C[i][j] > 0.01)
				{
					q1[0][0] = prevPoints[i].x;
					q1[1][0] = prevPoints[i].y;
	
					temp1 += C[i][j]*(q2-q1HatList[i]);
				}
			}

			Aj.inject(AjList[j]);
	
//			temp2 = matmult(SnInv, Aj);
			temp2[0][0] = SnInv[0][0]*Aj[0][0]; temp2[0][1] = 0; 					temp2[0][2] = SnInv[0][0]*Aj[0][2];
			temp2[1][0] = 0;					temp2[1][1] = SnInv[1][1]*Aj[1][1];	temp2[1][2] = SnInv[1][1]*Aj[1][2];
			temp2 = (1.0-C[N1][j])*temp2;

			ds0 = (1.0-C[N1][j])*(SnInv[0][0]*temp1[0][0]*temp1[0][0] + SnInv[1][1]*temp1[1][0]*temp1[1][0]); // assumes Sn diagnoal

//			ds1_T.inject(matmult(transpose(temp1), temp2));
			ds1_T[0][0] = temp1[0][0]*temp2[0][0];
			ds1_T[0][1] = temp1[1][0]*temp2[1][1];
			ds1_T[0][2] = temp1[0][0]*temp2[0][2]+temp1[1][0]*temp2[1][2];
			
//			Array2D<double>	dS2   = matmult(transpose(Aj), temp2);
			dS2[0][0] = Aj[0][0]*temp2[0][0]; 	dS2[0][1] = 0; 						dS2[0][2] = Aj[0][0]*temp2[0][2];
			dS2[1][0] = 0; 						dS2[1][1] = Aj[1][1]*temp2[1][1];	dS2[1][2] = Aj[1][1]*temp2[1][2];
			dS2[2][0] = Aj[0][2]*temp2[0][0];	dS2[2][1] = Aj[1][2]*temp2[1][1];	dS2[2][2] = Aj[0][2]*temp2[0][2]+Aj[1][2]*temp2[1][2]; 

			s0   += ds0;
			s1_T += ds1_T;
			S2   += dS2;

		}
	}

	Array2D<double> s1 = transpose(s1_T);

	// For easy evaluation of the objective function
	auto scoreFunc = [&](const Array2D<double> &vel, double z){ return -0.5*(
							s0
							-2.0/z*matmultS(s1_T, vel)
							+1.0/z/z*matmultS(transpose(vel), matmult(S2, vel))
							+matmultS( transpose(vel-mv), matmult(SvInv, vel-mv))
							+pow(z-mz,2)/vz
							);};

	// unique solution for optimal vel, given z
	auto solveVel =  [&](double z){
		if( maxPointCnt > 0)
		{
			s1 = ((double)min(maxPointCnt,N1))/N1*s1;
			S2 = ((double)min(maxPointCnt,N1))/N1*S2;
		}
		Array2D<double> temp1 = 1.0/z*s1+matmult(SvInv,mv);
		Array2D<double> temp2 = 1.0/z/z*S2+SvInv;
		JAMA::Cholesky<double> chol_temp2(temp2);
		return chol_temp2.solve(temp1);
	};

	///////////////////////////////////////////////////////////////
	// Find a good starting point
	///////////////////////////////////////////////////////////////
	double scoreBest = -0xFFFFFF;
	Array2D<double> velBest(3,1), velTemp(3,1);
	double zBest, score;
	double interval = 6.0*sz/10.0;
	for(double zTemp=mz-3*sz; zTemp < mz+3.1*sz; zTemp += interval )
	{
		velTemp.inject(solveVel(zTemp));
		score = scoreFunc(velTemp, zTemp);
		if(score > scoreBest)
		{
			scoreBest = score;
			velBest.inject(velTemp);
			zBest = zTemp;
		}
	}

	///////////////////////////////////////////////////////////////
	// Line search to find the optimum
	// Golden ratio algorithm (from Luenberger book)
	///////////////////////////////////////////////////////////////
	double zL= zBest-interval;
	double zR= zBest+interval;
	Array2D<double> velL = solveVel(zL);
	Array2D<double> velR = solveVel(zR);
	Array2D<double> vel1, vel2;
	double z1, z2;
	double score1, score2;
	double offset;
	double range = zR-zL;
	while(range > 1e-3)
	{
		offset = 0.618*range;
		z1 = zR-offset;
		z2 = zL+offset;
		vel1 = solveVel(z1);
		vel2 = solveVel(z2);
		score1 = scoreFunc(vel1, z1);
		score2 = scoreFunc(vel2, z2);
		if( score1 < score2 ) // keep right region
		{
			zL = z1;
			velL = vel1;
		}
		else
		{
			zR = z2;
			velR = vel2;
		}

		range = zR-zL;
	}

	velMAP = 0.5*(velL+velR);
	heightMAP = 0.5*(zL+zR);

	// Compute distribution
//	Array2D<double> covTemp = 1.0/heightMAP/heightMAP*min(1,N1)/N1*S2+SvInv;
//	JAMA::Cholesky<double> chol_covTemp(covTemp);
//	Array2D<double> covTempInv = chol_covTemp.solve(createIdentity((double)3.0));
//	covVel = matmult(transpose(covTempInv), matmult(1.0/heightMAP/heightMAP*S2, covTempInv));
}

void VelocityEstimator::onNewCommVelEstMeasCov(float measCov)
{
	mMutex_params.lock();
	mMeasCov = measCov;
	mMutex_params.unlock();

	Log::alert(String()+"Vision Meas Cov set to " + measCov);
}

void VelocityEstimator::onNewCommVelEstProbNoCorr(float probNoCorr)
{
	mMutex_params.lock();
	mProbNoCorr = probNoCorr;
	mMutex_params.unlock();

	Log::alert(String()+"Probability of no correspondence set to "+probNoCorr);
}

void VelocityEstimator::setRotPhoneToCam(const TNT::Array2D<double> &rot)
{
	mRotPhoneToCam.inject(rot);
	mRotCamToPhone.inject(transpose(rot));

	mRotPhoneToCam2 = blkdiag(mRotPhoneToCam, mRotPhoneToCam);
	mRotCamToPhone2 = blkdiag(mRotCamToPhone, mRotCamToPhone);
}

} // namespace Rover
} // namespace ICSL
