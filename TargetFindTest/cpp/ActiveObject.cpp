#include "ActiveObject.h"

namespace ICSL{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

unsigned long ActiveObject::lastID = 0;


ActiveObject::ActiveObject() : expectedPos(2,1,0.0), posCov(2,2,0.0)
{
	life = 10;
	this->id = lastID++;

	// make sure this is pos def
	posCov[0][0] = posCov[1][1] = 1;
}

ActiveObject::ActiveObject(std::vector<cv::Point> points) : ActiveObject()
{
	contour = points;
	mom = cv::moments(points);
	lastCenter.x = mom.m10/mom.m00;
	lastCenter.y = mom.m01/mom.m00;
	expectedPos[0][0] = lastCenter.x;
	expectedPos[1][0] = lastCenter.y;
	cv::HuMoments(mom, huMom);
	centralMoms[0] = mom.mu20;
	centralMoms[1] = mom.mu11;
	centralMoms[2] = mom.mu02;
	centralMoms[3] = mom.mu30;
	centralMoms[4] = mom.mu21;
	centralMoms[5] = mom.mu12;
//		centralMoms[6] = mom.mu03;
	centralMoms[6] = mom.m00;
}

void ActiveObject::updatePosition(const Array2D<double> &mv, const Array2D<double> &Sv, 
									double mz, double varz, 
									double focalLength, const cv::Point2f &center,
									const Array2D<double> &omega,
									const Time &curTime)
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

	double x = lastCenter.x-center.x;
	double y = lastCenter.y-center.y;
	double dt = Time::calcDiffNS(lastFoundTime, curTime)/1.0e9;

	// delta_x = q_x*v_z*dt-f*v_x*dt
	Array2D<double> mDelta(2,1), SDelta(2,2);
	mDelta[0][0] = x*mvz*dt-f*mvx*dt;
	mDelta[1][0] = y*mvz*dt-f*mvy*dt;

	SDelta[0][0] = pow(x*svz*dt,2)+pow(f*svx*dt, 2);
	SDelta[1][1] = pow(y*svz*dt,2)+pow(f*svy*dt, 2);

	// calc distribution moments
	expectedPos = mDelta*mz1Inv;

	// Shift the mean to so the distribution is for the location of q2 instead of q2-q1-Lw*w
	Array2D<double> Lw(2,3);
	Lw[0][0] = fInv*x*y; 		Lw[0][1] = -(f+fInv*x*x); 	Lw[0][2] = y;
	Lw[1][0] = f+fInv*y*y;		Lw[1][1] = -fInv*x*y;		Lw[1][2] = -x;
	expectedPos = expectedPos+dt*matmult(Lw, omega);

	expectedPos[0][0] += x;
	expectedPos[1][0] += y;

	// Calculate the ocvariance
	posCov[0][0] = (pow(mDelta[0][0],2)+SDelta[0][0])*mz2Inv-pow(mDelta[0][0],2)*pow(mz1Inv,2);
	posCov[1][1] = (pow(mDelta[1][0],2)+SDelta[1][1])*mz2Inv-pow(mDelta[1][0],2)*pow(mz1Inv,2);

	posCov[0][1] = posCov[1][0] = x*y*pow(dt,2)*pow(svz,2)*mz2Inv + mDelta[0][0]*mDelta[1][0]*(mz2Inv-pow(mz1Inv,2));

	// offset the point back to original coords
	expectedPos[0][0] += center.x;
	expectedPos[1][0] += center.y;
}

// TODO: This should operate on S1 and S2 (the covariances of each point being matched
// instead of assuming a constant Sn for curObjectList
Array2D<double> ActiveObject::calcCorrespondence(const vector<shared_ptr<ActiveObject>> &prevObjectList,
												 const vector<shared_ptr<ActiveObject>> &curObjectList,
												 const Array2D<double> &Sn,
												 const Array2D<double> &SnInv,
												 double varxi,
												 float probNoCorr)
{
	int N1 = prevObjectList.size();
	int N2 = curObjectList.size();

	if(N1 == 0 || N2 == 0)
		return Array2D<double>();

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
		md.inject(prevObjectList[i]->expectedPos);
		Sd.inject(prevObjectList[i]->posCov);

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
		x = curObjectList[j]->lastCenter.x;
		y = curObjectList[j]->lastCenter.y;
		for(int i=0; i<N1; i++)
		{
			md.inject(prevObjectList[i]->expectedPos);
			if(abs(x-md[0][0]) < xRangeList[i] && abs(y-md[1][0]) < yRangeList[i] )
				chad.push_back(make_pair(i, j));
			else
				C[i][j] = 0;
		}
	}

	double dxi, pxi;
	double pxi_coeff = 1.0/sqrt(2.0*PI*varxi);
	Array2D<double> mq(2,1), SnInvmq(2,1), ma(2,1), temp1(2,1);
	double scale = 1e6;
	for(int idx=0; idx<chad.size(); idx++)
	{
		int i=chad[idx].first;
		int j=chad[idx].second;
		mq[0][0] = curObjectList[j]->lastCenter.x;
		mq[1][0] = curObjectList[j]->lastCenter.y;

		SnInvmq[0][0] = SnInv[0][0]*mq[0][0]; // assumes Sn diagonal
		SnInvmq[1][0] = SnInv[1][1]*mq[1][0];

//		double fC = matmultS(transpose(mq),matmult(SnInv,mq));
		fC = SnInv[0][0]*mq[0][0]*mq[0][0] + SnInv[1][1]*mq[1][0]*mq[1][0]; // assumes Sn is diagonal
		
		md.inject(prevObjectList[i]->expectedPos);
//		ma = matmult(SaList[i],SdInvmdList[i]+SnInvmq);
		temp1.inject(SdInvmdList[i]+SnInvmq);
		ma[0][0] = SaList[i][0][0]*temp1[0][0] + SaList[i][0][1]*temp1[1][0];
		ma[1][0] = SaList[i][1][0]*temp1[0][0] + SaList[i][1][1]*temp1[1][0];
//		double f = -1.0*matmultS(transpose(ma),matmult(SaInvList[i],ma))+fBList[i]+fC;
		SaInv.inject(SaInvList[i]);
		f = -1.0*(SaInv[0][0]*ma[0][0]*ma[0][0] + 2.0*SaInv[0][1]*ma[0][0]*ma[1][0] + SaInv[1][1]*ma[1][0]*ma[1][0])
			+ fBList[i] + fC;

		// shape distribution
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

	return C;
}

// Adapted from OpenCV's matchShapes (but I use central moments instead of Hu moments)
double ActiveObject::calcShapeDistance(const shared_ptr<ActiveObject> &ao1, const shared_ptr<ActiveObject> &ao2)
{
return abs(ao1->mom.m00 - ao2->mom.m00);
	double c1, c2, c3;
	c1 = c2 = c3 = 0;
	int sma, smb;
	for(int k=0; k<7; k++)
	{
		double ama = abs( ao1->centralMoms[k] );
		double amb = abs( ao2->centralMoms[k] );
		if(ama < 1.e-5 || amb < 1.e-5)
			continue;
		if(ao1->centralMoms[k] > 0)
			sma = 1;
		else if(ao1->centralMoms[k] < 0)
			sma = -1;
		else
			sma = 0;
		if(ao2->centralMoms[k] > 0)
			smb = 1;
		else if(ao2->centralMoms[k] < 0)
			smb = -1;
		else
			smb = 0;

		ama = sma*log10(ama);
		amb = smb*log10(amb);
		c1 += abs(-1./ama+1./amb);
//		c2 += abs(-ama+amb);
//		c3 = max(c3, abs((ama-amb)/ama));
	}

	return c1;
}

}
