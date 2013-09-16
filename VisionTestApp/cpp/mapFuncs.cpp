#include "mapFuncs.h"
//#include <tbb/parallel_for.h>
//#include <tbb/mutex.h>

#include <iostream>

namespace ICSL {
namespace Rover {

DTYPE rs0, rs1, rs2, rs3, rs4, rs5;

vector<pair<Array2D<DTYPE>, Array2D<DTYPE> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
							Array2D<DTYPE> const &mv, Array2D<DTYPE> const &Sv, 
							DTYPE const &mz, DTYPE const &varz, 
							DTYPE const &focalLength, DTYPE const &dt, 
							Array2D<DTYPE> const &omega)
{
	DTYPE mvx = mv[0][0];
	DTYPE mvy = mv[1][0];
	DTYPE mvz = mv[2][0];
	DTYPE svx = sqrt(Sv[0][0]);
	DTYPE svy = sqrt(Sv[1][1]);
	DTYPE svz = sqrt(Sv[2][2]);
	DTYPE sz = sqrt(varz);
	DTYPE f = focalLength;
	DTYPE fInv = 1.0/f;

	// delta_x = q_x*v_z*dt-f*v_x*dt
	vector<Array2D<DTYPE> > mDeltaList(points.size()), SDeltaList(points.size());
	DTYPE x, y;
	Array2D<DTYPE> mDelta(2,1), SDelta(2,2,0.0);
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
	DTYPE mz1Inv = 1.0/mz;
	DTYPE mz2Inv = 1.0/mz/mz;
	DTYPE log_sz = log(sz);
	DTYPE log_mz = log(mz);
	DTYPE del1LnOld = 0xFFFFFFFF;
	DTYPE del2LnOld = 0xFFFFFFFF;
	DTYPE del1Ln = 0;
	DTYPE del2Ln = 0;
	DTYPE del1, del2;
	DTYPE fact;
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

	// calc distribution moments
	vector<pair<Array2D<DTYPE>, Array2D<DTYPE> > > priorDistList(mDeltaList.size());
	Array2D<DTYPE> md(2,1), Sd(2,2,0.0), Lw(2,3);
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

		priorDistList[i] = pair<Array2D<DTYPE>, Array2D<DTYPE> >(md.copy(), Sd.copy());
	}

	return priorDistList;
}

Array2D<DTYPE> calcCorrespondence(vector<pair<Array2D<DTYPE>, Array2D<DTYPE> > > const &priorDistList, vector<cv::Point2f> const &curPointList, Array2D<DTYPE> const &Sn, Array2D<DTYPE> const &SnInv)
{
	int N1 = priorDistList.size();
	int N2 = curPointList.size();

	if(N1 == 0 || N2 == 0)
		return Array2D<DTYPE>();

Time start;
	// Precompute some things
	vector<Array2D<DTYPE> > SdInvmdList(N1), SaInvList(N1), SaList(N1);
	vector<DTYPE> fBList(N1), coeffList(N1), xRangeList(N1), yRangeList(N1);
	Array2D<DTYPE> eye2 = createIdentity((DTYPE)2.0);
	DTYPE det_Sn = Sn[0][0]*Sn[1][1] - Sn[0][1]*Sn[1][0];
	Array2D<DTYPE> md(2,1), Sd(2,2), SdInv(2,2), Sa(2,2), SaInv(2,2);
	Array2D<DTYPE> S(2,2), V(2,2,0.0), D(2,2,0.0);
	DTYPE den;
	DTYPE fB, det_Sd, det_Sa, coeff, xrange, yrange;
	DTYPE theta1, theta2, r1, r2;
	DTYPE eigMid, eigOffset;
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

rs0 += start.getElapsedTimeNS()/1.0e9; start.setTime();
	Array2D<DTYPE> C(N1+1, N2+1);
	vector<pair<int, int> > chad;
	DTYPE x, y, fC, f;
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

	DTYPE peakCoeff = 0;
	Array2D<DTYPE> mq(2,1), SnInvmq(2,1), ma(2,1), temp1(2,1);
	for(int idx=0; idx<chad.size(); idx++)
	{
		int i=chad[idx].first;
		int j=chad[idx].second;
		mq[0][0] = curPointList[j].x;
		mq[1][0] = curPointList[j].y;

		SnInvmq[0][0] = SnInv[0][0]*mq[0][0]; // assumes Sn diagonal
		SnInvmq[1][0] = SnInv[1][1]*mq[1][0];

//		DTYPE fC = matmultS(transpose(mq),matmult(SnInv,mq));
		fC = SnInv[0][0]*mq[0][0]*mq[0][0] + SnInv[1][1]*mq[1][0]*mq[1][0]; // assumes Sn is diagonal
		
		md.inject(priorDistList[i].first);
//		ma = matmult(SaList[i],SdInvmdList[i]+SnInvmq);
		temp1.inject(SdInvmdList[i]+SnInvmq);
		ma[0][0] = SaList[i][0][0]*temp1[0][0] + SaList[i][0][1]*temp1[1][0];
		ma[1][0] = SaList[i][1][0]*temp1[0][0] + SaList[i][1][1]*temp1[1][0];
//		DTYPE f = -1.0*matmultS(transpose(ma),matmult(SaInvList[i],ma))+fBList[i]+fC;
		SaInv.inject(SaInvList[i]);
		f = -1.0*(SaInv[0][0]*ma[0][0]*ma[0][0] + 2.0*SaInv[0][1]*ma[0][0]*ma[1][0] + SaInv[1][1]*ma[1][0]*ma[1][0])
			+ fBList[i] + fC;
		C[i][j] = coeffList[i]*exp(-0.5*f);
		peakCoeff = max(peakCoeff,coeffList[i]);
	}

rs1 += start.getElapsedTimeNS()/1.0e9; start.setTime();
	DTYPE probNoCorr = 0.1*peakCoeff;
	for(int j=0; j<N2; j++)
		C[N1][j] = probNoCorr;

	C[N1][N2] = 0;

	// Scale colums to unit sum
	for(int j=0; j<N2; j++)
	{
		DTYPE colSum = 0;
		for(int i=0; i<N1+1; i++)
			colSum += C[i][j];

		for(int i=0; i<N1+1; i++)
			if(C[i][j] != 0)
				C[i][j] /= colSum;
	}

	// Now check if any of the rows sum to over 1
	for(int i=0; i<N1; i++)
	{
		DTYPE rowSum = 0;
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
		DTYPE colSum = 0;
		for(int i=0; i<N1; i++)
			colSum += C[i][j];
		C[N1][j] = 1-colSum;
	}

rs2 += start.getElapsedTimeNS()/1.0e9; start.setTime();
	return C;
}

void computeMAPEstimate(Array2D<DTYPE> &velMAP /*out*/, Array2D<DTYPE> &covVel /*out*/, DTYPE &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<DTYPE> const &C, // correspondence matrix
						Array2D<DTYPE> const &mv, // velocity mean
						Array2D<DTYPE> const &Sv, // velocity covariance
						DTYPE const &mz, // height mean
						DTYPE const &vz, // height variance
						Array2D<DTYPE> const &Sn, // feature measurement covariance
						DTYPE const &focalLength, DTYPE const &dt, Array2D<DTYPE> const &omega)
{
	computeMAPEstimate(velMAP, covVel, heightMAP, prevPoints, curPoints, C, mv, Sv, mz, vz, Sn, focalLength, dt, omega, -1);
}

void computeMAPEstimate(Array2D<DTYPE> &velMAP /*out*/, Array2D<DTYPE> &covVel /*out*/, DTYPE &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<DTYPE> const &C, // correspondence matrix
						Array2D<DTYPE> const &mv, // velocity mean
						Array2D<DTYPE> const &Sv, // velocity covariance
						DTYPE const &mz, // height mean
						DTYPE const &vz, // height variance
						Array2D<DTYPE> const &Sn, // feature measurement covariance
						DTYPE const &focalLength, DTYPE const &dt, Array2D<DTYPE> const &omega,
						int maxPointCnt)
{
Time start;
	int N1 = prevPoints.size();
	int N2 = curPoints.size();
	DTYPE f = focalLength;
	DTYPE fInv = 1.0/focalLength;

	JAMA::Cholesky<DTYPE> chol_Sv(Sv), chol_Sn(Sn);
	Array2D<DTYPE> SvInv = chol_Sv.solve(createIdentity((DTYPE)3.0));
	Array2D<DTYPE> SnInv = chol_Sn.solve(createIdentity((DTYPE)2.0));

	DTYPE sz = sqrt(vz);

	///////////////////////////////////////////////////////////////
	// Build up constant matrices
	///////////////////////////////////////////////////////////////
	vector<Array2D<DTYPE> > LvList(N1), q1HatList(N1);
	Array2D<DTYPE> Lv(2,3), Lw(2,3), q1(2,1);
	DTYPE x, y;
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

//rs0 += start.getElapsedTimeNS()/1.0e9; start.setTime();
	vector<Array2D<DTYPE> > AjList(N2);
	Array2D<DTYPE> Aj(2,3);
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

//rs1 += start.getElapsedTimeNS()/1.0e9; start.setTime();
	DTYPE s0 = 0;
	Array2D<DTYPE> s1_T(1,3,0.0), S2(3,3,0.0);
	Array2D<DTYPE> q2(2,1), ds1_T(1,3), dS2(3,3);
	Array2D<DTYPE> temp1(2,1), temp2(2,3);
	DTYPE ds0;
	for(int j=0; j<N2; j++)
	{
		if( (1-C[N1][j]) > 1e-2)
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

//			Array2D<DTYPE>	ds1_T = matmult(transpose(temp1), temp2);
			ds1_T[0][0] = temp1[0][0]*temp2[0][0];
			ds1_T[0][1] = temp1[1][0]*temp2[1][1];
			ds1_T[0][2] = temp1[0][0]*temp2[0][2]+temp1[1][0]*temp2[1][2];
			
//			Array2D<DTYPE>	dS2   = matmult(transpose(Aj), temp2);
			dS2[0][0] = Aj[0][0]*temp2[0][0]; 	dS2[0][1] = 0; 						dS2[0][2] = Aj[0][0]*temp2[0][2];
			dS2[1][0] = 0; 						dS2[1][1] = Aj[1][1]*temp2[1][1];	dS2[1][2] = Aj[1][1]*temp2[1][2];
			dS2[2][0] = Aj[0][2]*temp2[0][0];	dS2[2][1] = Aj[1][2]*temp2[1][1];	dS2[2][2] = Aj[0][2]*temp2[0][2]+Aj[1][2]*temp2[1][2]; 

			s0   += ds0;
			s1_T += ds1_T;
			S2   += dS2;
		}
	}

//rs2 += start.getElapsedTimeNS()/1.0e9; start.setTime();
	Array2D<DTYPE> s1 = transpose(s1_T);

	// For easy evaluation of the objective function
	auto scoreFunc = [&](Array2D<DTYPE> const &vel, DTYPE const &z){ return -0.5*(
							s0
							-2.0/z*matmultS(s1_T, vel)
							+1.0/z/z*matmultS(transpose(vel), matmult(S2, vel))
							+matmultS( transpose(vel-mv), matmult(SvInv, vel-mv))
							+pow(z-mz,2)/vz
							);};

	// unique solution for optimal vel, given z
	auto solveVel =  [&](DTYPE const &z){
		if( maxPointCnt > 0)
		{
			s1 = ((DTYPE)min(maxPointCnt,N1))/N1*s1;
			S2 = ((DTYPE)min(maxPointCnt,N1))/N1*S2;
		}
		Array2D<DTYPE> temp1 = 1.0/z*s1+matmult(SvInv,mv);
		Array2D<DTYPE> temp2 = 1.0/z/z*S2+SvInv;
		JAMA::Cholesky<DTYPE> chol_temp2(temp2);
		return chol_temp2.solve(temp1);
	};

	///////////////////////////////////////////////////////////////
	// Find a good starting point
	///////////////////////////////////////////////////////////////
	DTYPE scoreBest = -0xFFFFFF;
	Array2D<DTYPE> velBest(3,1), velTemp(3,1);
	DTYPE zBest, score;
	DTYPE interval = 6.0*sz/10.0;
	for(DTYPE zTemp=mz-3*sz; zTemp < mz+3.1*sz; zTemp += interval )
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

//rs3 += start.getElapsedTimeNS()/1.0e9; start.setTime();
	///////////////////////////////////////////////////////////////
	// Line search to find the optimum
	///////////////////////////////////////////////////////////////
	DTYPE zL= zBest-interval;
	DTYPE zR= zBest+interval;
	Array2D<DTYPE> velL = solveVel(zL);
	Array2D<DTYPE> velR = solveVel(zR);
	Array2D<DTYPE> vel1, vel2;
	DTYPE z1, z2;
	DTYPE score1, score2;
	DTYPE offset;
	DTYPE range = zR-zL;
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

	// Compute distribution
//	Array2D<DTYPE> covTemp = 1.0/heightMAP/heightMAP*min(1,N1)/N1*S2+SvInv;
//	JAMA::Cholesky<DTYPE> chol_covTemp(covTemp);
//	Array2D<DTYPE> covTempInv = chol_covTemp.solve(createIdentity((DTYPE)3.0));
//	covVel = matmult(transpose(covTempInv), matmult(1.0/heightMAP/heightMAP*S2, covTempInv));

//printArray("	mv:\t",mv);
//printArray("velMAP:\t",velMAP);
////printArray("Sv:\n", Sv);
////printArray("S2:\n", S2);
////printArray("covTemp:\n",covTemp);
////printArray("covVel:\n", covVel);
////printArray("velMAP:\t", velMAP);
//int chad = 0;

//rs4 += start.getElapsedTimeNS()/1.0e9; start.setTime();
}

}
}
