#include "ActiveRegion.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;
using namespace toadlet::egg;

float ActiveRegion::MAX_LIFE=40;
size_t ActiveRegion::lastID = 0;
std::mutex ActiveRegion::mutex_lastID;

ActiveRegion::ActiveRegion() :
	mExpectedPos(2,1,0.0),
	mPosCov(2,2,0.0),
	mPrincipalAxes(2,2,0.0),
	mPrincipalAxesEigVal(2,0.0)
{
	mLife = 10;
	mutex_lastID.lock(); mId = lastID++; mutex_lastID.unlock();

	// make sure this is pos def
	mPosCov[0][0] = mPosCov[1][1] = 1;
}

ActiveRegion::ActiveRegion(std::vector<cv::Point> points) : ActiveRegion()
{
	mContour = points;
	mMoments = cv::moments(points);
	mFoundPos.x = mMoments.m10/mMoments.m00;
	mFoundPos.y = mMoments.m01/mMoments.m00;
	mExpectedPos[0][0] = mFoundPos.x;
	mExpectedPos[1][0] = mFoundPos.y;
	calcPrincipalAxes();
}

void ActiveRegion::copyData(const ActiveRegion &ao)
{
	mContour.assign(ao.mContour.begin(), ao.mContour.end());
	mMoments = ao.mMoments;
	mPrevFoundPos = mFoundPos;
	mFoundPos = ao.mFoundPos;
	mPrincipalAxes.inject(ao.mPrincipalAxes);
	mPrincipalAxesEigVal[0] = ao.mPrincipalAxesEigVal[0];
	mPrincipalAxesEigVal[1] = ao.mPrincipalAxesEigVal[1];
	for(int i=0; i<ao.mNeighbors.size(); i++)
		addNeighbor(ao.mNeighbors[i], true);
}

void ActiveRegion::markFound(const Time &time)
{
	mLastFoundTime.setTime(time);
}

void ActiveRegion::addLife(float val)
{
	mLife = min(MAX_LIFE, mLife+val);
}

void ActiveRegion::takeLife(float val)
{
	mLife -= val;

	if(mLife <= 0)
		kill();
}

void ActiveRegion::kill()
{
	mLife = 0;
	// need to copy this list first
	// since when we go to remove neighbors
	// it will modify my list
	vector<shared_ptr<ActiveRegion>> nList = mNeighbors;
	for(int i=0; i<mNeighbors.size(); i++)
		mNeighbors[i]->removeNeigbor(mId, false);

	mNeighbors.clear();
}

void ActiveRegion::addNeighbor(shared_ptr<ActiveRegion> n, bool doTwoWay)
{
	if(n->mLife <= 0)
	{
		Log::alert("dud");
		return;
	}
	if(n->mId == mId)
	{
		Log::alert("adding myself");
		return;
	}

	bool found = false;
	for(int i=0; i<mNeighbors.size(); i++)
		if(n->mId == mNeighbors[i]->mId)
		{
			found = true;
			break;
		}

	if(!found)
	{
		mNeighbors.push_back(n);
//Log::alert(String()+"Connecting " + mId + " and "+n->mId);
	}

	if(doTwoWay)
		n->addNeighbor(shared_from_this(), false);

	// check for duds
	bool haveDud = false;
	for(int i=0; i<mNeighbors.size(); i++)
		if(!mNeighbors[i]->isAlive())
		{
			haveDud = true;
			break;
		}
	
	if(haveDud)// || mNeighbors.size() > 50)
	{
		String str = "\n";
		Log::alert("------------------------- Have a dud -------------------------");
		for(int i=0; i<mNeighbors.size(); i++)
		{
			str = str+mNeighbors[i]->mId+"\t";
			str = str+mNeighbors[i]->mLife+"\t";
			str = str+"\n";
		}
		Log::alert(str);
		Log::alert("//////////////////////////////////////////////////");
	}
}

void ActiveRegion::removeNeigbor(int nid, bool doTwoWay)
{
	vector<shared_ptr<ActiveRegion>>::iterator iter;
	iter = mNeighbors.begin();
	bool found = false;
	while(iter != mNeighbors.end() && !found)
	{
		if((*iter)->mId == nid)
		{
			shared_ptr<ActiveRegion> n = *iter;
			mNeighbors.erase(iter);
			if(doTwoWay)
				n->removeNeigbor(mId, false);
			found = true;
		}
		iter++;
	}
}

// from http://en.wikipedia.org/wiki/Image_moment
void ActiveRegion::calcPrincipalAxes()
{
	Array2D<double> cov(2,2);
	cov[0][0] = mMoments.mu20/mMoments.m00;
	cov[0][1] = mMoments.mu11/mMoments.m00;
	cov[1][0] = cov[0][1];
	cov[1][1] = mMoments.mu02/mMoments.m00;

	double eigMid = 0.5*(cov[0][0]+cov[1][1]);
	double eigOffset = 0.5*sqrt( (cov[0][0]-cov[1][1])*(cov[0][0]-cov[1][1]) + 4*cov[0][1]*cov[0][1] );
	mPrincipalAxesEigVal[0] = eigMid+eigOffset;
	mPrincipalAxesEigVal[1] = eigMid-eigOffset;
	if(cov[0][1] == 0)
	{
		if(cov[0][0] > cov[1][1])
		{
			mPrincipalAxes[0][0] = 1;
			mPrincipalAxes[1][0] = 0;
			mPrincipalAxes[0][1] = 0;
			mPrincipalAxes[1][1] = 1;
		}
		else
		{
			mPrincipalAxes[0][0] = 0;
			mPrincipalAxes[1][0] = 1;
			mPrincipalAxes[0][1] = 1;
			mPrincipalAxes[1][1] = 0;
		}
	}
	else
	{
		mPrincipalAxes[0][0] = mPrincipalAxesEigVal[0]-cov[1][1];
		mPrincipalAxes[1][0] = cov[0][1];
		mPrincipalAxes[0][1] = mPrincipalAxes[1][0];
		mPrincipalAxes[1][1] = -mPrincipalAxes[0][0];
	}

	double scale0 = norm2(submat(mPrincipalAxes,0,1,0,0));
	double scale1 = norm2(submat(mPrincipalAxes,0,1,1,1));
	mPrincipalAxes[0][0] /= scale0;
	mPrincipalAxes[1][0] /= scale0;
	mPrincipalAxes[0][1] /= scale1;
	mPrincipalAxes[1][1] /= scale1;
	mPrincipalAxesEigVal[0] /= scale0;
	mPrincipalAxesEigVal[1] /= scale1;
}

// Hmm, should I change this so it does incremental updates instead of updating as
// one big step from the last found time
void ActiveRegion::updatePositionDistribution(const Array2D<double> &mv, const Array2D<double> &Sv, 
									double mz, double varz, 
									double focalLength, const cv::Point2f &center,
									const Array2D<double> &omega,
									const Time &curTime)
{
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

	double x = mFoundPos.x-center.x;
	double y = mFoundPos.y-center.y;
	double dt = Time::calcDiffNS(mLastFoundTime, curTime)/1.0e9;

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

// TODO: This should operate on S1 and S2 (the covariances of each point being matched
// instead of assuming a constant Sn for curObjectList
Array2D<double> ActiveRegion::calcCorrespondence(const vector<shared_ptr<ActiveRegion>> &prevObjectList,
												 const vector<shared_ptr<ActiveRegion>> &curObjectList,
												 const Array2D<double> &Sn1,
												 const Array2D<double> &SnInv1,
												 double varxi_ratio,
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

		S.inject(Sd+Sn);
		eigMid = 0.5*(S[0][0]+S[1][1]);
		eigOffset = 0.5*sqrt( (S[0][0]-S[1][1])*(S[0][0]-S[1][1]) + 4*S[0][1]*S[0][1] );
		D[0][0] = eigMid+eigOffset;
		D[1][1] = eigMid-eigOffset;
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
		x = curObjectList[j]->mFoundPos.x;
		y = curObjectList[j]->mFoundPos.y;
		for(int i=0; i<N1; i++)
		{
			md.inject(prevObjectList[i]->mExpectedPos);
			if(abs(x-md[0][0]) < xRangeList[i] && abs(y-md[1][0]) < yRangeList[i] )
				chad.push_back(make_pair(i, j));
			else
				C[i][j] = 0;
		}
	}

	double dxi, pxi;
//	double pxi_coeff = 1.0/sqrt(2.0*PI*varxi);
	Array2D<double> mq(2,1), SnInvmq(2,1), ma(2,1), temp1(2,1);
	double scale = 1e6;
	for(int idx=0; idx<chad.size(); idx++)
	{
		int i=chad[idx].first;
		int j=chad[idx].second;
		mq[0][0] = curObjectList[j]->mFoundPos.x;
		mq[1][0] = curObjectList[j]->mFoundPos.y;

		SnInvmq[0][0] = SnInv[0][0]*mq[0][0]; // assumes Sn diagonal
		SnInvmq[1][0] = SnInv[1][1]*mq[1][0];

//		double fC = matmultS(transpose(mq),matmult(SnInv,mq));
		fC = SnInv[0][0]*mq[0][0]*mq[0][0] + SnInv[1][1]*mq[1][0]*mq[1][0]; // assumes Sn is diagonal
		
		md.inject(prevObjectList[i]->mExpectedPos);
//		ma = matmult(SaList[i],SdInvmdList[i]+SnInvmq);
		temp1.inject(SdInvmdList[i]+SnInvmq);
		ma[0][0] = SaList[i][0][0]*temp1[0][0] + SaList[i][0][1]*temp1[1][0];
		ma[1][0] = SaList[i][1][0]*temp1[0][0] + SaList[i][1][1]*temp1[1][0];
//		double f = -1.0*matmultS(transpose(ma),matmult(SaInvList[i],ma))+fBList[i]+fC;
		SaInv.inject(SaInvList[i]);
		f = -1.0*(SaInv[0][0]*ma[0][0]*ma[0][0] + 2.0*SaInv[0][1]*ma[0][0]*ma[1][0] + SaInv[1][1]*ma[1][0]*ma[1][0])
			+ fBList[i] + fC;

		// shape distribution
		double varxi = pow(varxi_ratio*prevObjectList[i]->getArea(),2);
		double pxi_coeff = 1.0/sqrt(2.0*PI*varxi);
		dxi = calcShapeDistance(prevObjectList[i], curObjectList[j]);
		pxi = pxi_coeff*exp(-0.5*dxi*dxi/varxi);
		C[i][j] = scale*pxi*coeffList[i]*exp(-0.5*f);
	}

	for(int j=0; j<N2; j++)
		C[N1][j] = scale*probNoCorr;

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

	// nothing else to do here since the last colum and row have to be zeros
	if(probNoCorr == 0)
		return C;

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

//printArray("C:\n",C);

	return C;
}

double ActiveRegion::calcShapeDistance(const shared_ptr<ActiveRegion> &ao1, const shared_ptr<ActiveRegion> &ao2)
{
	return abs(ao1->mMoments.m00 - ao2->mMoments.m00);
// Adapted from OpenCV's matchShapes (but I use central moments instead of Hu moments)
//	double c1, c2, c3;
//	c1 = c2 = c3 = 0;
//	int sma, smb;
//	for(int k=0; k<7; k++)
//	{
//		double ama = abs( ao1->centralMoms[k] );
//		double amb = abs( ao2->centralMoms[k] );
//		if(ama < 1.e-5 || amb < 1.e-5)
//			continue;
//		if(ao1->centralMoms[k] > 0)
//			sma = 1;
//		else if(ao1->centralMoms[k] < 0)
//			sma = -1;
//		else
//			sma = 0;
//		if(ao2->centralMoms[k] > 0)
//			smb = 1;
//		else if(ao2->centralMoms[k] < 0)
//			smb = -1;
//		else
//			smb = 0;
//
//		ama = sma*log10(ama);
//		amb = smb*log10(amb);
//		c1 += abs(-1./ama+1./amb);
////		c2 += abs(-ama+amb);
////		c3 = max(c3, abs((ama-amb)/ama));
//	}
//
//	return c1;
}

}
}
