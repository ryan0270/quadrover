#include "TrackedObject.h"
#include <Observer_Angular.h>
#include <Observer_Translational.h>

namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;
using namespace toadlet::egg;

Observer_Angular *TrackedObject::mObsvAngular = NULL;
Observer_Translational *TrackedObject::mObsvTranslation = NULL;

size_t TrackedObject::lastID = 0;
std::mutex TrackedObject::mutex_lastID;

TrackedObject::TrackedObject() :
	mExpectedPos(2,1,0.0),
	mPosCov(2,2,0.0)
{
	mType = TrackedObjectType::UNKNOWN;
	mutex_lastID.lock(); mId = lastID++; mutex_lastID.unlock();

	// make sure this is pos def
	mPosCov[0][0] = mPosCov[1][1] = 1;

	mIsAlive = true;
}

void TrackedObject::markFound(shared_ptr<TrackedObject> &match)
{
	mLastFoundTime.setTime(match->mLastFoundTime);
	mLocation = match->mLocation;
	mHistory.push_back(pair<Time, cv::Point2f>(mLastFoundTime, mLocation));
}

void TrackedObject::kill()
{
	mIsAlive = false;
}

void TrackedObject::rebirth()
{
	mutex_lastID.lock(); mId = lastID++; mutex_lastID.unlock();
	pair<Time, cv::Point2f> h = mHistory.back();
	mHistory.clear();
	mHistory.push_back(h);
}

// Hmm, should I change this so it does incremental updates instead of updating as
// one big step from the last found time
void TrackedObject::updatePositionDistribution(double focalLength, const cv::Point2f &center, const Time &curTime)
{
	double dt = Time::calcDiffNS(mLastFoundTime, curTime)/1.0e9; 

	Array2D<double> curState = mObsvTranslation->estimateStateAtTime(curTime);
	Array2D<double> curErrCov = mObsvTranslation->estimateErrCovAtTime(curTime);
	Array2D<double> mv = submat(curState,3,5,0,0);
	Array2D<double> Sv = submat(curErrCov,3,5,3,5);
	double mz = curState[2][0];
	double varz= curErrCov[2][2];

	SO3 curAtt = mObsvAngular->estimateAttAtTime(curTime);
	SO3 prevAtt = mObsvAngular->estimateAttAtTime(mLastFoundTime);
	SO3 attChange = curAtt*prevAtt.inv();
	Array2D<double> omega = 1.0/dt*attChange.log().toVector();

	const double mvx = mv[0][0];
	const double mvy = mv[1][0];
	const double mvz = mv[2][0];
	const double svx = sqrt(Sv[0][0]);
	const double svy = sqrt(Sv[1][1]);
	const double svz = sqrt(Sv[2][2]);
	const double sz = sqrt(varz);
	const double f = focalLength;
	const double fInv = 1.0/f;

	// calc distribution of Z^-1
	double mz1Inv = 1.0/mz;
	double mz2Inv = 1.0/mz/mz;
	double log_sz = log(sz);
	double log_mz = log(mz);
	double del1LnOld = DBL_MAX;//0xFFFFFFFF;
	double del2LnOld = DBL_MAX;//0xFFFFFFFF;
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

	double x = mLocation.x-center.x;
	double y = mLocation.y-center.y;

	// delta_x = q_x*v_z*dt-f*v_x*dt
	Array2D<double> mDelta(2,1), SDelta(2,2);
	mDelta[0][0] = x*mvz*dt-f*mvx*dt;
	mDelta[1][0] = y*mvz*dt-f*mvy*dt;

	SDelta[0][0] = pow(x*svz*dt,2)+pow(f*svx*dt, 2);
	SDelta[1][1] = pow(y*svz*dt,2)+pow(f*svy*dt, 2);
	SDelta[0][1] = SDelta[1][0] = 0;

	// calc distribution moments
	mExpectedPos = mDelta*mz1Inv;

	// Shift the mean to so the distribution is for the location of q2 instead of q2-q1-Lw*w
	Array2D<double> Lw(2,3);
	Lw[0][0] = fInv*x*y; 		Lw[0][1] = -(f+fInv*x*x); 	Lw[0][2] = y;
	Lw[1][0] = f+fInv*y*y;		Lw[1][1] = -fInv*x*y;		Lw[1][2] = -x;
	mExpectedPos = mExpectedPos+dt*matmult(Lw, omega);

	mExpectedPos[0][0] += x;
	mExpectedPos[1][0] += y;

	// Calculate the covariance
	mPosCov[0][0] = (pow(mDelta[0][0],2)+SDelta[0][0])*mz2Inv-pow(mDelta[0][0],2)*pow(mz1Inv,2);
	mPosCov[1][1] = (pow(mDelta[1][0],2)+SDelta[1][1])*mz2Inv-pow(mDelta[1][0],2)*pow(mz1Inv,2);

	mPosCov[0][1] = mPosCov[1][0] = x*y*pow(dt,2)*pow(svz,2)*mz2Inv + mDelta[0][0]*mDelta[1][0]*(mz2Inv-pow(mz1Inv,2));

	// offset the point back to original coords
	mExpectedPos[0][0] += center.x;
	mExpectedPos[1][0] += center.y;
}

// TODO: This should operate on S1 and S2 (the covariances of each point being matched)
// instead of assuming a constant Sn for curObjectList
Array2D<double> TrackedObject::calcCorrespondence(const vector<shared_ptr<TrackedObject>> &prevObjectList,
												 const vector<shared_ptr<TrackedObject>> &curObjectList,
												 const Array2D<double> &Sn1,
												 const Array2D<double> &SnInv1,
												 double probNoCorr)
{
	int N1 = prevObjectList.size();
	int N2 = curObjectList.size();

	if(N1 == 0 || N2 == 0)
		return Array2D<double>();

	// To account for measurement noise both in the previous object and the current object
	Array2D<double> Sn = 2.0*Sn1;
	Array2D<double> SnInv = 0.5*SnInv1;

	// Precompute some things
	vector<Array2D<double>> SdInvmdList(N1), SaInvList(N1), SaList(N1);
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
		md.inject(prevObjectList[i]->mExpectedPos);
		Sd.inject(prevObjectList[i]->mPosCov);

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

		if(coeff / (coeff+probNoCorr) < 0.5)
			prevObjectList[i]->kill();

		S.inject(Sd+Sn);
		eigMid = 0.5*(S[0][0]+S[1][1]);
		eigOffset = 0.5*sqrt( (S[0][0]-S[1][1])*(S[0][0]-S[1][1]) + 4*S[0][1]*S[0][1] );
		D[0][0] = eigMid+eigOffset;
		D[1][1] = eigMid-eigOffset;
		if(S[0][1] == 0)
		{
			if(S[0][0] > S[1][1])
			{
				V[0][0] = 1; V[1][0] = 0;
				V[0][1] = 0; V[1][1] = 1;
			}
			else
			{
				V[0][0] = 0; V[1][0] = 1;
				V[0][1] = 1; V[1][1] = 0;
			}
		}
		else
		{
			V[0][0] = D[0][0]-S[1][1];	V[1][0] = S[0][1];
			V[0][1] = V[1][0];			V[1][1] = -V[0][0];
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

		SdInvmdList[i] = matmult(SdInv, md);
		SaInvList[i] = SaInv.copy();
		SaList[i] = Sa.copy();
		fBList[i] = fB;
		coeffList[i] = coeff;
		xRangeList[i] = xrange;
		yRangeList[i] = yrange;
	}

	Array2D<double> C(N1+1, N2+1);
	vector<pair<int, int>> chad;
	double x, y, fC, f;
	for(int j=0; j<N2; j++)
	{
		x = curObjectList[j]->mLocation.x;
		y = curObjectList[j]->mLocation.y;
		for(int i=0; i<N1; i++)
		{
			if(curObjectList[j]->mType != prevObjectList[i]->mType)
			{
				C[i][j] = 0;
				continue;
			}
			md.inject(prevObjectList[i]->mExpectedPos);
			if(abs(x-md[0][0]) < xRangeList[i] && abs(y-md[1][0]) < yRangeList[i] )
				chad.push_back(make_pair(i, j));
			else
				C[i][j] = 0;
		}
	}

	Array2D<double> mq(2,1), SnInvmq(2,1), ma(2,1), temp1(2,1);
	for(int idx=0; idx<chad.size(); idx++)
	{
		int i=chad[idx].first;
		int j=chad[idx].second;
		mq[0][0] = curObjectList[j]->mLocation.x;
		mq[1][0] = curObjectList[j]->mLocation.y;

		SnInvmq[0][0] = SnInv[0][0]*mq[0][0]; // assumes Sn diagonal
		SnInvmq[1][0] = SnInv[1][1]*mq[1][0];
//		double fC = matmultS(transpose(mq),matmult(SnInv,mq));
		fC = SnInv[0][0]*mq[0][0]*mq[0][0] + SnInv[1][1]*mq[1][0]*mq[1][0]; // assumes Sn is diagonal
//		md.inject(prevObjectList[i]->mExpectedPos);
//		ma = matmult(SaList[i],SdInvmdList[i]+SnInvmq);
		temp1.inject(SdInvmdList[i]+SnInvmq);
		ma[0][0] = SaList[i][0][0]*temp1[0][0] + SaList[i][0][1]*temp1[1][0];
		ma[1][0] = SaList[i][1][0]*temp1[0][0] + SaList[i][1][1]*temp1[1][0];
//		double f = -1.0*matmultS(transpose(ma),matmult(SaInvList[i],ma))+fBList[i]+fC;
		SaInv.inject(SaInvList[i]);
		f = -1.0*(SaInv[0][0]*ma[0][0]*ma[0][0] + 2.0*SaInv[0][1]*ma[0][0]*ma[1][0] + SaInv[1][1]*ma[1][0]*ma[1][0])
			+ fBList[i] + fC;
		C[i][j] = coeffList[i]*exp(-0.5*f);
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

	// nothing else to do here since the last colum and row have to be zeros
	if(probNoCorr == 0)
		return C;

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
		colSum = 0;
		for(int i=0; i<N1; i++)
			colSum += C[i][j];
		C[N1][j] = 1-colSum;
	}

//printArray("C:\n",C);

	return C;
}

Array2D<double> TrackedObject::calcCorrespondence2(const vector<shared_ptr<TrackedObject>> &prevObjectList,
												 const vector<shared_ptr<TrackedObject>> &curObjectList,
												 double probNoCorr)
{
	int N1 = prevObjectList.size();
	int N2 = curObjectList.size();

	if(N1 == 0 || N2 == 0)
		return Array2D<double>();

	// Precompute some things
	vector<Array2D<double>> SdList(N1), SdInvList(N1), SdInvmdList(N1);
	vector<double> fBList(N1), det_SdList(N1);
	Array2D<double> eye2 = createIdentity((double)2.0);
	Array2D<double> md(2,1), Sd(2,2), SdInv(2,2);
	double den;
	double fB, det_Sd;
	for(int i=0; i<N1; i++)
	{
		md.inject(prevObjectList[i]->mExpectedPos);
		Sd.inject(prevObjectList[i]->mPosCov);

		den = Sd[0][0]*Sd[1][1]-Sd[0][1]*Sd[1][0];
		SdInv[0][0] = Sd[1][1]/den;
		SdInv[0][1] = -Sd[0][1]/den;
		SdInv[1][0] = SdInv[0][1];
		SdInv[1][1] = Sd[0][0]/den;

//		fB = matmultS(transpose(md),matmult(SdInv,md));
		fB = SdInv[0][0]*md[0][0]*md[0][0] + 2.0*SdInv[0][1]*md[0][0]*md[1][0] + SdInv[1][1]*md[1][0]*md[1][0];
		det_Sd = Sd[0][0]*Sd[1][1] - Sd[0][1]*Sd[1][0];

//		if(coeff / (coeff+probNoCorr) < 0.5)
//			prevObjectList[i]->kill();


		SdList[i] = Sd.copy();
		SdInvList[i] = SdInv.copy();
		SdInvmdList[i] = matmult(SdInv, md);
		fBList[i] = fB;
		det_SdList[i] = det_Sd;
	}

	vector<Array2D<double>> SaList(N1,Array2D<double>(2,2)), SaInvmdList(N1,Array2D<double>(2,2));
	Array2D<double> C(N1+1, N2+1,0.0);
	Array2D<double> S(2,2), Sn(2,2), SnInv(2,2), Sa(2,2), SaInv(2,2);
	Array2D<double> ma(2,1), mq(2,1), SnInvmq(2,1);
	Array2D<double> V(2,2), D(2,2);
	double det_Sa, coeff;
	double x, y, fC, f, det_Sn, det_SnInv;
	double theta1, theta2, r1, r2;
	double eigMid, eigOffset;
	double xrange, yrange;
	vector<double> maxSurvivability(N1);
	for(int j=0; j<N2; j++)
	{
		x = curObjectList[j]->mLocation.x;
		y = curObjectList[j]->mLocation.y;
		Sn.inject(curObjectList[j]->mPosCov);

		det_Sn = Sn[0][0]*Sn[1][1]-Sn[0][1]*Sn[1][0];
		det_SnInv =1.0/det_Sn;
		SnInv[0][0] = det_SnInv*Sn[1][1];
		SnInv[0][1] = -det_SnInv*Sn[0][1];
		SnInv[1][0] = SnInv[0][1];
		SnInv[1][1] = det_SnInv*Sn[0][0];
		for(int i=0; i<N1; i++)
		{
			if(curObjectList[j]->mType != prevObjectList[i]->mType)
			{
				C[i][j] = 0;
				continue;
			}
			Sd.inject(SdList[i]);
			S.inject(Sd+Sn);
			eigMid = 0.5*(S[0][0]+S[1][1]);
			eigOffset = 0.5*sqrt( (S[0][0]-S[1][1])*(S[0][0]-S[1][1]) + 4*S[0][1]*S[0][1] );
			D[0][0] = eigMid+eigOffset;
			D[1][1] = eigMid-eigOffset;
			if(S[0][1] == 0)
			{
				if(S[0][0] > S[1][1])
				{
					V[0][0] = 1; V[1][0] = 0;
					V[0][1] = 0; V[1][1] = 1;
				}
				else
				{
					V[0][0] = 0; V[1][0] = 1;
					V[0][1] = 1; V[1][1] = 0;
				}
			}
			else
			{
				V[0][0] = D[0][0]-S[1][1];	V[1][0] = S[0][1];
				V[0][1] = V[1][0];			V[1][1] = -V[0][0];
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

			md.inject(prevObjectList[i]->mExpectedPos);
			if(abs(x-md[0][0]) > xrange || abs(y-md[1][0]) > yrange )
				C[i][j] = 0;
			else
			{
				mq[0][0] = curObjectList[j]->mLocation.x;
				mq[1][0] = curObjectList[j]->mLocation.y;

				SnInvmq.inject(matmult(SnInv, mq));

				fC = matmultS(transpose(mq),matmult(SnInv,mq));
				SaInv.inject(SdInvList[i]+SnInv);
				den = SaInv[0][0]*SaInv[1][1]-SaInv[0][1]*SaInv[1][0];
				Sa[0][0] = SaInv[1][1]/den;
				Sa[0][1] = -SaInv[0][1]/den;
				Sa[1][0] = Sa[0][1];
				Sa[1][1] = SaInv[0][0]/den;

				ma = matmult(Sa,SdInvmdList[i]+SnInvmq);
//				temp1.inject(SdInvmdList[i]+SnInvmq);
//				ma[0][0] = Sa[0][0]*temp1[0][0] + Sa[0][1]*temp1[1][0];
//				ma[1][0] = Sa[1][0]*temp1[0][0] + Sa[1][1]*temp1[1][0];

				f = -1.0*matmultS(transpose(ma),matmult(SaInv,ma))+fBList[i]+fC;
//				f = -1.0*(SaInv[0][0]*ma[0][0]*ma[0][0] + 2.0*SaInv[0][1]*ma[0][0]*ma[1][0] + SaInv[1][1]*ma[1][0]*ma[1][0])
//					+ fBList[i] + fC;

				det_Sd = det_SdList[i];
				det_Sa = Sa[0][0]*Sa[1][1] - Sa[0][1]*Sa[1][0];
				coeff = sqrt(det_Sa)/2.0/PI/sqrt(det_Sd*det_Sn);

				maxSurvivability[i] = max(maxSurvivability[i], coeff/probNoCorr);

				C[i][j] = coeff*exp(-0.5*f);
			}
		}
	}

	// Kill off the prev objects that can't survive anymore
	for(int i=0; i<N1; i++)
		if(maxSurvivability[i] < 0.5)
			prevObjectList[i]->kill();

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

	// nothing else to do here since the last colum and row have to be zeros
	if(probNoCorr == 0)
		return C;

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
		colSum = 0;
		for(int i=0; i<N1; i++)
			colSum += C[i][j];
		C[N1][j] = 1-colSum;
	}

	return C;
}

const Time &TrackedObject::getCreateTime() const
{
	return mHistory[0].first;
}

TrackedPoint::TrackedPoint() : TrackedObject()
{
	mType = TrackedObjectType::POINT;
}

TrackedPoint::TrackedPoint(const Time & time, const cv::Point2f &point) : TrackedPoint()
{
	mLocation = point;
	mExpectedPos[0][0] = point.x;
	mExpectedPos[1][0] = point.y;
	mHistory.push_back(pair<Time, cv::Point2f>(time, point));
	mLastFoundTime.setTime(time);
}

TrackedRegion::TrackedRegion() : TrackedObject()
{
	mType = TrackedObjectType::REGION;
}

TrackedRegion::TrackedRegion(const Time &time,
							 const vector<cv::Point2f> &contour,
							 const cv::Point2f &point,
							 const cv::Moments &mom) : TrackedRegion()
{
	mLocation = point;
	mExpectedPos[0][0] = point.x;
	mExpectedPos[1][0] = point.y;
	mHistory.push_back(pair<Time, cv::Point2f>(time, point));
	mLastFoundTime.setTime(time);

	mContour = contour;
	mMoments = mom;
}

void TrackedRegion::markFound(shared_ptr<TrackedObject> &match)
{
	TrackedObject::markFound(match);
	mContour = static_pointer_cast<TrackedRegion>(match)->mContour;
	mMoments = static_pointer_cast<TrackedRegion>(match)->mMoments;
}

}
}
