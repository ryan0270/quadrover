#include "mapFuncs.h"
#include <tbb/parallel_for.h>
#include <tbb/mutex.h>

#include <iostream>

namespace ICSL {
namespace Rover {

vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
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

	// delta_x = q_x*v_z*dt-f*v_x*dt
	vector<Array2D<double> > mDeltaList(points.size()), SDeltaList(points.size());
//	for(int i=0; i<points.size(); i++)
	tbb::parallel_for(0, (int)points.size(), [&](int i)
	{
		double x = points[i].x;
		double y = points[i].y;

		Array2D<double> mDelta(2,1), SDelta(2,2,0.0);
		mDelta[0][0] = x*mvz*dt-f*mvx*dt;
		mDelta[1][0] = y*mvz*dt-f*mvy*dt;

		SDelta[0][0] = pow(x*svz*dt,2)+pow(f*svx*dt, 2);
		SDelta[1][1] = pow(y*svz*dt,2)+pow(f*svy*dt, 2);

		mDeltaList[i] = mDelta.copy();
		SDeltaList[i] = SDelta.copy();
	}
	);

	// calc distribution of Z^-1
	double mz1Inv = 1.0/mz;
	double mz2Inv = 1.0/mz/mz;
	double log_sz = log(sz);
	double log_mz = log(mz);
	double del1LnOld = 0xFFFFFFFF;
	double del2LnOld = 0xFFFFFFFF;
	double del1Ln = 0;
	double del2Ln = 0;
	double fact;
	for(int k=1; k<10; k++)
	{
		fact = fact2ln(k);
		del1Ln = 2*k*log_sz+fact-(2*k+1)*log_mz;
		del2Ln = log(2*k+1)+2*k*log_sz+fact-(2*k+2)*log_mz;
		if(del1Ln < del1LnOld)
		{
			mz1Inv = mz1Inv+exp(del1Ln);
			del1LnOld = del1Ln;
		}
		if(del2Ln < del2LnOld)
		{
			mz2Inv = mz2Inv+exp(del2Ln);
			del2LnOld = del2Ln;
		}
	}

	// calc distribution moments
//	list<pair<Array2D<double>, Array2D<double> > > priorDistList;
	vector<pair<Array2D<double>, Array2D<double> > > priorDistList(mDeltaList.size());
//	for(int i=0; i<mDeltaList.size(); i++)
	tbb::parallel_for(0, (int)mDeltaList.size(), [&](int i)
	{
		Array2D<double> mDelta = mDeltaList[i];
		Array2D<double> SDelta = SDeltaList[i];
		Array2D<double> md(2,1), Sd(2,2,0.0);
		md = mDelta*mz1Inv;

		// Shift the mean to so the distribution is for the location of q2 instead of q2-q1-Lw*w
		double x = points[i].x;
		double y = points[i].y;

		Array2D<double> Lw(2,3);
		Lw[0][0] = 1.0/f*x*y; 		Lw[0][1] = -1.0/f*(1+x*x); 	Lw[0][2] = y;
		Lw[1][0] = 1.0/f*(1+y*y);	Lw[1][1] = -1.0/f*x*y;		Lw[1][2] = -x;
		md = md+dt*matmult(Lw, omega);

		md[0][0] += x;
		md[1][0] += y;

		Sd[0][0] = (pow(mDelta[0][0],2)+SDelta[0][0])*mz2Inv-pow(mDelta[0][0],2)*pow(mz1Inv,2);
		Sd[1][1] = (pow(mDelta[1][0],2)+SDelta[1][1])*mz2Inv-pow(mDelta[1][0],2)*pow(mz1Inv,2);

		Sd[0][1] = Sd[1][0] = x*y*pow(dt,2)*pow(svz,2)*mz2Inv + mDelta[0][0]*mDelta[1][0]*(mz2Inv-pow(mz1Inv,2));

		priorDistList[i] = pair<Array2D<double>, Array2D<double> >(md.copy(), Sd.copy());
	}
	);

	return priorDistList;
}

Array2D<double> calcCorrespondence(vector<pair<Array2D<double>, Array2D<double> > > const &priorDistList, vector<cv::Point2f> const &curPointList, Array2D<double> const &Sn, Array2D<double> const &SnInv)
{
	int N1 = priorDistList.size();
	int N2 = curPointList.size();

	if(N1 == 0 || N2 == 0)
		return Array2D<double>();

	// Precompute some things
	vector<Array2D<double> > SdInvmdList(N1), SaInvList(N1), SaList(N1);
	vector<double> fBList(N1), coeffList(N1), xRangeList(N1), yRangeList(N1);
	Array2D<double> eye2 = createIdentity(2);
	double det_Sn = Sn[0][0]*Sn[1][1] - Sn[0][1]*Sn[1][0];
//	for(int i=0; i<N1; i++)
	tbb::parallel_for(0, N1, [&](int i)
	{
		Array2D<double> md = priorDistList[i].first;
		Array2D<double> Sd = priorDistList[i].second;

		JAMA::Cholesky<double> chol_Sd(Sd);
		Array2D<double> SdInv = chol_Sd.solve(eye2);
		
		Array2D<double> SaInv = SdInv+SnInv;
		JAMA::Cholesky<double> chol_SaInv(SaInv);
		Array2D<double> Sa = chol_SaInv.solve(eye2);

		double fB = matmultS(transpose(md),matmult(SdInv,md));
		double det_Sd = Sd[0][0]*Sd[1][1] - Sd[0][1]*Sd[1][0];
		double det_Sa = Sa[0][0]*Sa[1][1] - Sa[0][1]*Sa[1][0];
		double coeff = sqrt(det_Sa)*2.0*ICSL::Constants::PI*sqrt(det_Sd*det_Sn);
		double xrange = 5*sqrt(Sd[0][0]+Sn[0][0]);
		double yrange = 5*sqrt(Sd[1][1]+Sn[1][1]);

		SdInvmdList[i] = matmult(SdInv, md);
		SaInvList[i] = SaInv.copy();
		SaList[i] = Sa.copy();
		fBList[i] = fB;
		coeffList[i] = coeff;
		xRangeList[i] = xrange;
		yRangeList[i] = yrange;
	}
	);

	Array2D<double> C(N1+1, N2+1);
	vector<double> colSum(N2,0.0);
	double probNoCorr = 1.0/640.0/480.0;
	for(int j=0; j<N2; j++)
	// parallelizing at this level is causing memory curruption, but not sure why
//	tbb::parallel_for(0, N2, [&](int j)
	{
		double x = curPointList[j].x;
		double y = curPointList[j].y;
		Array2D<double> mq(2,1);
		mq[0][0] = x;
		mq[1][0] = y;

		double fC = matmultS(transpose(mq),matmult(SnInv,mq));
//		for(int i=0; i<N1; i++)
		tbb::mutex mutex_sum;
		tbb::parallel_for(0, N1, [&](int i)
		{
			Array2D<double> md = priorDistList[i].first;

			if(abs(x-md[0][0]) < xRangeList[i] && abs(y-md[1][0]) < yRangeList[i] )
			{
//				mutex_list.lock();
				Array2D<double> ma = matmult(SaList[i],SdInvmdList[i]+matmult(SnInv,mq));
				double f = -1.0*matmultS(transpose(ma),matmult(SaInvList[i],ma))+fBList[i]+fC;
//				mutex_list.unlock();
				C[i][j] = coeffList[i]*exp(-0.5*f);
				mutex_sum.lock();
				colSum[j] += C[i][j];
				mutex_sum.unlock();
			}
			else
				C[i][j] = 0;
		}
		);

		C[N1][j] = probNoCorr;
//		mutex_sum.lock();
		colSum[j] += probNoCorr;
//		mutex_sum.unlock();
	}
//	);

	C[N1][N2] = 0;

	// Scale colums to unit sum
	for(int j=0; j<N2; j++)
	{
		for(int i=0; i<N1+1; i++)
			C[i][j] /= colSum[j];
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

void computeMAPEstimate(Array2D<double> &velMAP /*out*/, double &heightMAP /*out*/,
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
	int N1 = prevPoints.size();
	int N2 = curPoints.size();
	double f = focalLength;

	JAMA::Cholesky<double> chol_Sv(Sv), chol_Sn(Sn);
	Array2D<double> SvInv = chol_Sv.solve(createIdentity(3));
	Array2D<double> SnInv = chol_Sn.solve(createIdentity(2));

	double sz = sqrt(vz);
	
	///////////////////////////////////////////////////////////////
	// Build up constant matrices
	///////////////////////////////////////////////////////////////
	vector<Array2D<double> > LvList(N1), q1HatList(N1);
//	for(int i=0; i<N1; i++)
	tbb::parallel_for(0, N1, [&](int i)
	{
		double x = prevPoints[i].x;
		double y = prevPoints[i].y;

		Array2D<double> Lv(2,3);
		Lv[0][0] = -f; Lv[0][1] = 0;  Lv[0][2] = x;
		Lv[1][0] = 0;  Lv[1][1] = -f; Lv[1][2] = y;
		LvList[i] = Lv.copy();

		Array2D<double> Lw(2,3);
		Lw[0][0] = 1.0/f*x*y; 		Lw[0][1] = -1.0/f*(1+x*x); 	Lw[0][2] = y;
		Lw[1][0] = 1.0/f*(1+y*y);	Lw[1][1] = -1.0/f*x*y;		Lw[1][2] = -x;

		Array2D<double> q1(2,1);
		q1[0][0] = x;
		q1[1][0] = y;
		Array2D<double> q1Hat = q1 + matmult(Lw, omega);
		q1HatList[i] = q1Hat.copy();
	}
	);

for(int i=0; i<N1; i++)
{
	cout << "point " << i << " Lv" << endl;
	for(int k=0; k<2; k++)
		for(int j=0; j<3; j++)
				cout << LvList[i][k][j] << "\t" << endl;
	cout << "--------------------------------------------------" << endl;
}

	vector<Array2D<double> > AjList(N2);
//	for(int j=0; j<N2; j++)
	tbb::parallel_for(0, N2, [&](int j)
	{
		Array2D<double> Aj(2,3,0.0);
		for(int i=0; i<N1; i++)
		{
			if(C[i][j] > 0.01)
				Aj += C[i][j]*LvList[i];
		}
		Aj = dt*Aj;

		AjList[j] = Aj.copy();
	}
	);
//cout << "C" << endl;
//for(int i=0; i<C.dim1(); i++)
//{
//	for(int j=0; j<C.dim2(); j++)
//		cout << C[i][j] << "\t";
//	cout << endl;
//}
//cout << "--------------------------------------------------" << endl;

for(int i=0; i<N2; i++)
{
	cout << "point " << i << " Aj" << endl;
	for(int k=0; k<2; k++)
		for(int j=0; j<3; j++)
				cout << AjList[i][k][j] << "\t" << endl;
	cout << "--------------------------------------------------" << endl;
}

	double s0 = 0;
	Array2D<double> s1_T(1,3,0.0), S2(3,3,0.0);
	tbb::mutex mutex_S;
//	for(int j=0; j<N2; j++)
	tbb::parallel_for(0, N2, [&](int j)
	{
		Array2D<double> Aj = AjList[j];
		Array2D<double> Aj_T = transpose(Aj);
		if( (1-C[N1][j]) > 1e-1)
		{
			Array2D<double> q1(2,1), q2(2,1);
			q2[0][0] = curPoints[j].x;
			q2[1][0] = curPoints[j].y;
			Array2D<double> temp1(2,1,0.0);
			for(int i=0; i<N1; i++)
			{
				if(C[i][j] > 0.01)
				{
					q1[0][0] = prevPoints[i].x;
					q1[1][0] = prevPoints[i].y;
	
					temp1 += C[i][j]*(q2-q1HatList[i]);
				}
			}
	
			Array2D<double> temp2 = matmult(SnInv, Aj);
			mutex_S.lock();
			double 			ds0   = (1-C[N1][j])*matmultS(transpose(temp1), matmult(SnInv, temp1));
			Array2D<double>	ds1_T = (1-C[N1][j])*matmult(transpose(temp1), temp2);
			Array2D<double>	dS2   = (1-C[N1][j])*matmult(Aj_T, temp2);
			mutex_S.unlock();

			mutex_S.lock();
			s0   += ds0;
			s1_T += ds1_T;
			S2   += dS2;
			mutex_S.unlock();
		}
	}
	);


	Array2D<double> s1 = transpose(s1_T);

cout << "s0: " << s0 << endl;
cout << "--------------------------------------------------" << endl;
cout << "s1:\t";
for(int i=0; i<3; i++)
	cout << s1[i][0] << "\t";
cout << endl;
cout << "--------------------------------------------------" << endl;
cout << "S2:\n";
for(int i=0; i<3; i++)
{
	for(int j=0; j<3; j++)
		cout << S2[i][j] << "\t";
	cout << endl;
}
cout << "--------------------------------------------------" << endl;

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
	double zBest;
	double interval = 6.0*sz/10.0;
	for(double zTemp=mz-3*sz; zTemp < mz+3.1*sz; zTemp += interval )
	{
		velTemp.inject(solveVel(zTemp));
		double score = scoreFunc(velTemp, zTemp);
		if(score > scoreBest)
		{
			scoreBest = score;
			velBest.inject(velTemp);
			zBest = zTemp;
		}
	}

	///////////////////////////////////////////////////////////////
	// Line search to find the optimum
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

for(int i=0; i<3; i++)
	cout << velMAP[i][0] << "\t";
cout << endl;
}

}
}
