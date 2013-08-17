#include "VelocityEstimator.h"

namespace ICSL {
namespace Quadrotor {
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

VelocityEstimator::VelocityEstimator() : 
	mRotPhoneToCam(3,3,0.0),
	mRotCamToPhone(3,3,0.0)
{
	mDone = true;
	mRunning = false;

	mNewImageDataAvailable = false;
	mLastImageFeatureData = NULL;

	mMeasCov = 2*5*5;
	mProbNoCorr = 0.1;
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
}

void VelocityEstimator::run()
{
	mDone = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	shared_ptr<ImageFeatureData> oldImageFeatureData, curImageFeatureData;
	oldImageFeatureData = curImageFeatureData = NULL;
	Time procTimer;
	double procTime, delayTime;
	Array2D<double> velEst(3,1,0.0);
	double heightEst = 0.05;;
	String logString;
	float measCov, probNoCorr;
	while(mRunning)
	{
		if(mNewImageDataAvailable)
		{
			procTimer.setTime();
			oldImageFeatureData = curImageFeatureData;
			curImageFeatureData = mLastImageFeatureData;
			mNewImageDataAvailable = false;
			
			if(oldImageFeatureData != NULL && oldImageFeatureData->featurePoints.size() > 5 && curImageFeatureData->featurePoints.size() > 5)
			{
				mMutex_params.lock();
				measCov = mMeasCov;
				probNoCorr = mProbNoCorr;
				mMutex_params.unlock();
				curImageFeatureData->lock();
				doVelocityEstimate(oldImageFeatureData, curImageFeatureData, velEst, heightEst, measCov, probNoCorr);
				curImageFeatureData->unlock();

				shared_ptr<DataVector<double> > velData(new DataVector<double>());
				velData->data = velEst.copy();
				velData->type = DATA_TYPE_MAP_VEL;
				curImageFeatureData->lock();
				velData->timestamp.setTime(curImageFeatureData->imageData->timestamp);
				curImageFeatureData->unlock();

				shared_ptr<Data<double> > heightData(new Data<double>());
				heightData->data = heightEst;
				heightData->type = DATA_TYPE_MAP_HEIGHT;
				curImageFeatureData->lock();
				heightData->timestamp.setTime(curImageFeatureData->imageData->timestamp);
				curImageFeatureData->unlock();

				for(int i=0; i<mListeners.size(); i++)
					mListeners[i]->onVelocityEstimator_newEstimate(velData, heightData);

				procTime = procTimer.getElapsedTimeNS()/1.0e9;
				curImageFeatureData->lock();
				delayTime = curImageFeatureData->imageData->timestamp.getElapsedTimeNS()/1.0e9;
				curImageFeatureData->unlock();
				mMutex_data.lock();
				mLastDelayTimeUS = delayTime*1.0e6;
				mMutex_data.unlock();

				if(mQuadLogger != NULL)
				{
					logString = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_MAP_VEL+ "\t";
					for(int i=0; i<velEst.dim1(); i++)
						logString = logString + velEst[i][0] + "\t";
					mQuadLogger->addLine(logString,LOG_FLAG_CAM_RESULTS);

					logString = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_MAP_HEIGHT + "\t" + heightEst;
					mQuadLogger->addLine(logString,LOG_FLAG_CAM_RESULTS);

					logString = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_MAP_VEL_CALC_TIME + "\t"+procTime;
					mQuadLogger->addLine(logString,LOG_FLAG_CAM_RESULTS);

					logString = String()+mStartTime.getElapsedTimeMS() + "\t" + LOG_ID_OPTIC_FLOW_VELOCITY_DELAY + "\t"+ delayTime;
					mQuadLogger->addLine(logString,LOG_FLAG_CAM_RESULTS);
				}
			}
		}

		System::msleep(1);
	}

	mDone = true;
}

// See eqn 98 in the Feb 25, 2013 notes
void VelocityEstimator::doVelocityEstimate(shared_ptr<ImageFeatureData> const &oldFeatureData,
										   shared_ptr<ImageFeatureData> const &curFeatureData,
										   Array2D<double> &velEst, 
										   double &heightEst,
										   double visionMeasCov,
										   double probNoCorr) const
{
//Log::alert("=======================================================");
	Time oldTime = oldFeatureData->imageData->timestamp;
	Time curTime = curFeatureData->imageData->timestamp;
	double dt = Time::calcDiffNS(oldTime, curTime)/1.0e9;

	float scale = 1.0; // for a 640x480 image
	if(curFeatureData->imageData->image->cols == 320)
		scale = 0.5;
	cv::Point2f center;
	center.x = scale*curFeatureData->imageData->centerX_640x480;
	center.y = scale*curFeatureData->imageData->centerY_640x480;
	float focalLength = scale*curFeatureData->imageData->focalLength_640x480;

	// Get relevant state data
	Array2D<double> oldState = mObsvTranslational->estimateStateAtTime(oldTime);
//	Array2D<double> oldErrCov = mObsvTranslational->estimateErrCovAtTime(oldTime);
	Array2D<double> eulOld = submat(oldState,0,2,0,0);
	Array2D<double> attOld = createRotMat_ZYX( eulOld[2][0], eulOld[1][0], eulOld[0][0] );

	Array2D<double> curState = mObsvTranslational->estimateStateAtTime(curTime);
	Array2D<double> curErrCov = mObsvTranslational->estimateErrCovAtTime(curTime);
	Array2D<double> eulCur = submat(curState,0,2,0,0);
	Array2D<double> attCur = createRotMat_ZYX( eulCur[2][0], eulCur[1][0], eulCur[0][0] );

	Array2D<double> attChange = matmult(transpose(attCur), attOld);
	Array2D<double> omega = logSO3(attChange, dt);

	// Rotate data to cam coords
	omega.inject( matmult( mRotPhoneToCam, omega ));
	Array2D<double> mRotPhoneToCam2 = blkdiag(mRotPhoneToCam, mRotPhoneToCam);
	Array2D<double> mRotCamToPhone2 = blkdiag(mRotCamToPhone, mRotCamToPhone);
	curState = matmult( mRotPhoneToCam2, curState);
	curErrCov = matmult( mRotPhoneToCam2, matmult(curErrCov, mRotCamToPhone2));

	// get prior distributions
	vector<cv::Point2f> oldPoints(oldFeatureData->featurePoints.size());
	for(int i=0; i<oldPoints.size(); i++)
		oldPoints[i] = oldFeatureData->featurePoints[i]-center;

	vector<cv::Point2f> curPoints(curFeatureData->featurePoints.size());
	for(int i=0; i<curPoints.size(); i++)
		curPoints[i] = curFeatureData->featurePoints[i]-center;

//while(curPoints.size() > 10)
//	curPoints.pop_back();

	Array2D<double> mv = submat(curState,3,5,0,0);
	Array2D<double> Sv = submat(curErrCov,3,5,3,5);
	double mz = -curState[2][0];
	double sz = sqrt(curErrCov[2][2]);

	double camOffset = 0.05;
	mz -= camOffset;
	mz = max(mz, 0.130);

	Array2D<double> Sn(2,2,0.0), SnInv(2,2,0.0);
	Sn[0][0] = Sn[1][1] = visionMeasCov;
	SnInv[0][0] = SnInv[1][1] = 1.0/Sn[0][0];

	vector<pair<Array2D<double>, Array2D<double> > > priorDistList(oldPoints.size());
	priorDistList = calcPriorDistributions(oldPoints, mv, Sv, mz, sz*sz, focalLength, dt, omega);
	Array2D<double> C = calcCorrespondence(priorDistList, curPoints, Sn, SnInv, probNoCorr);

	double numMatches = 0;
	for(int i=0; i<C.dim1()-1; i++)
		for(int j=0; j<C.dim2()-1; j++)
			numMatches += C[i][j];
	{
		String str = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_MAP_NUM_MATCHES+"\t"+numMatches;
		mQuadLogger->addLine(str, LOG_FLAG_CAM_RESULTS);
	}
	
	Array2D<double> vel(3,1), covVel(3,3);
	double z;
	computeMAPEstimate(vel, covVel, z, oldPoints, curPoints, C, mv, Sv, mz, sz*sz, Sn, focalLength, dt, omega);
	z += camOffset;

	vel = matmult(mRotCamToPhone, vel);
	covVel = matmult(mRotCamToPhone, matmult(covVel, mRotPhoneToCam));

	velEst.inject(vel);
	heightEst = z;
}

void VelocityEstimator::onFeaturesFound(shared_ptr<ImageFeatureData> const &data)
{
	mLastImageFeatureData = data;
	mNewImageDataAvailable = true;
}

vector<pair<Array2D<double>, Array2D<double> > > VelocityEstimator::calcPriorDistributions(vector<cv::Point2f> const &points, 
							Array2D<double> const &mv, Array2D<double> const &Sv, 
							double const &mz, double const &varz, 
							double const &focalLength, double const &dt, 
							Array2D<double> const &omega)
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

//printArray("mv:\t", mv);
//printArray("Sv:\n",Sv);
//printArray("w:\t",omega);
//Log::alert(String()+"mz:\t"+mz);
//Log::alert(String()+"vz:\t"+varz);
//Log::alert(String()+"dt:\t"+dt);

	// delta_x = q_x*v_z*dt-f*v_x*dt
	vector<Array2D<double> > mDeltaList(points.size()), SDeltaList(points.size());
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
	double del1LnOld = 0xFFFFFFFF;
	double del2LnOld = 0xFFFFFFFF;
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
	// This is needed since Zinv isn't actually normally distributed
	// so the subsequent calculations can sometimes cause problems.
//	mz2Inv = max(mz2Inv, mz1Inv*mz1Inv);
	if(mz2Inv < mz1Inv*mz1Inv)
		Log::alert(String()+"crap, mz2Inv >= mz1Inv*mz1Inv");

//Log::alert(String()+"mz1Inv: "+mz1Inv);
//Log::alert(String()+"mz2Inv: "+mz2Inv);

	// calc distribution moments
	vector<pair<Array2D<double>, Array2D<double> > > priorDistList(mDeltaList.size());
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

		priorDistList[i] = pair<Array2D<double>, Array2D<double> >(md.copy(), Sd.copy());

//if(i == 0)
//{
//	printArray("md:\t", md);
//	printArray("chad dist:\n", Sd);
//}
	}

	return priorDistList;
}

Array2D<double> VelocityEstimator::calcCorrespondence(vector<pair<Array2D<double>, Array2D<double> > > const &priorDistList,
													  vector<cv::Point2f> const &curPointList,
													  Array2D<double> const &Sn,
													  Array2D<double> const &SnInv,
													  float const &probNoCorr)
{
	int N1 = priorDistList.size();
	int N2 = curPointList.size();

	if(N1 == 0 || N2 == 0)
		return Array2D<double>();

	// Precompute some things
	vector<Array2D<double> > SdInvmdList(N1), SaInvList(N1), SaList(N1);
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
		V[0][0] = D[0][0]-S[1][1];
		V[1][0] = S[0][1];
		V[0][1] = V[1][0];
		V[1][1] = -V[0][0];

		theta1 = atan2(V[1][0], V[0][0]);
		theta2 = atan2(V[1][1], V[1][0]);
		r1 = 3.0*sqrt(D[0][0]);
		r2 = 3.0*sqrt(D[1][1]);

		while(theta1 < -PI/2.0)	theta1 += PI;
		while(theta1 > PI/2.0) 	theta1 -= PI;
		while(theta2 < 0) 		theta2 += PI;
		while(theta2 > PI) 		theta2 -= PI;

		xrange = max( r1*cos(theta1), r2*cos(theta2) );
		yrange = max( r1*sin(theta1), r2*sin(theta2) );

		SdInvmdList[i] = matmult(SdInv, md);
		SaInvList[i] = SaInv.copy();
		SaList[i] = Sa.copy();
		fBList[i] = fB;
		coeffList[i] = coeff;
		xRangeList[i] = xrange;
		yRangeList[i] = yrange;
	}

	Array2D<double> C(N1+1, N2+1);
	vector<pair<int, int> > chad;
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

//	{
//		String str = String()+mStartTime.getElapsedTimeMS()+"\t"+LOG_ID_MAP_PEAK_PSD_VALUE+"\t"+peakCoeff;
//		mQuadLogger->addLine(str, LOG_FLAG_CAM_RESULTS);
//	}
//	double thresh = probNoCorr*peakCoeff;
	for(int j=0; j<N2; j++)
		C[N1][j] = probNoCorr;

	C[N1][N2] = 0;

	// Scale colums to unit sum
	for(int j=0; j<N2; j++)
	{
		double colSum = 0;
		for(int i=0; i<N1+1; i++)
			colSum += C[i][j];

		for(int i=0; i<N1+1; i++)
			if(C[i][j] != 0)
				C[i][j] /= colSum;
	}

	// Now check if any of the rows sum to over 1
	for(int i=0; i<N1; i++)
	{
		double rowSum = 0;
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

	return C;
}

void VelocityEstimator::computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<double> const &C, // correspondence matrix
						Array2D<double> const &mv, // velocity mean
						Array2D<double> const &Sv, // velocity covariance
						double const &mz, // height mean
						double const &vz, // height variance
						Array2D<double> const &Sn, // feature measurement covariance
						double const &focalLength, double const &dt, Array2D<double> const &omega)
{
	computeMAPEstimate(velMAP, covVel, heightMAP, prevPoints, curPoints, C, mv, Sv, mz, vz, Sn, focalLength, dt, omega, -1);
}

void VelocityEstimator::computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<double> const &C, // correspondence matrix
						Array2D<double> const &mv, // velocity mean
						Array2D<double> const &Sv, // velocity covariance
						double const &mz, // height mean
						double const &vz, // height variance
						Array2D<double> const &Sn, // feature measurement covariance
						double const &focalLength, double const &dt, Array2D<double> const &omega,
						int maxPointCnt)
{
	int N1 = prevPoints.size();
	int N2 = curPoints.size();
	double f = focalLength;
	double fInv = 1.0/focalLength;

	if(N1 == 0 || N2 == 0)
	{
		velMAP = Array2D<double>(3,1,0.0);
		heightMAP = 0;
		return;
	}

	JAMA::Cholesky<double> chol_Sv(Sv), chol_Sn(Sn);
	Array2D<double> SvInv = chol_Sv.solve(createIdentity((double)3.0));
	Array2D<double> SnInv = chol_Sn.solve(createIdentity((double)2.0));

	double sz = sqrt(vz);

	///////////////////////////////////////////////////////////////
	// Build up constant matrices
	///////////////////////////////////////////////////////////////
	vector<Array2D<double> > LvList(N1), q1HatList(N1);
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

	vector<Array2D<double> > AjList(N2);
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
		if( (1-C[N1][j]) > 0.01)
		{
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

//			Array2D<double>	ds1_T = matmult(transpose(temp1), temp2);
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
	auto scoreFunc = [&](Array2D<double> const &vel, double const &z){ return -0.5*(
							s0
							-2.0/z*matmultS(s1_T, vel)
							+1.0/z/z*matmultS(transpose(vel), matmult(S2, vel))
							+matmultS( transpose(vel-mv), matmult(SvInv, vel-mv))
							+pow(z-mz,2)/vz
							);};

	// unique solution for optimal vel, given z
	auto solveVel =  [&](double const &z){
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
		z2 = zL-offset;
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
}

void VelocityEstimator::onNewCommVelEstMeasCov(float const &measCov)
{
	mMutex_params.lock();
	mMeasCov = measCov;
	mMutex_params.unlock();

	Log::alert(String()+"Vision Meas Cov set to " + measCov);
}

void VelocityEstimator::onNewCommVelEstProbNoCorr(float const &probNoCorr)
{
	mMutex_params.lock();
	mProbNoCorr = probNoCorr;
	mMutex_params.unlock();

	Log::alert(String()+"Probability of no correspondence set to "+probNoCorr);
}

} // namespace Rover
} // namespace ICSL
