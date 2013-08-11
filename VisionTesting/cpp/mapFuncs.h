#ifndef ICSL_MAP_FUNCS
#define ICSL_MAP_FUNCS
#include <vector>

#include "../../Rover/cpp/Data.h"
#include "../../Rover/cpp/TNT/tnt.h"
#include "../../Rover/cpp/TNT/jama_cholesky.h"
#include "../../Rover/cpp/TNT/jama_eig.h"
#include "../../Rover/cpp/TNT_Utils.h"
#include "../../Rover/cpp/Time.h"
#include "../../Rover/cpp/constants.h"

#include "types.h"

namespace ICSL{
namespace Rover {
using namespace std;
using namespace ICSL::Quadrotor;
using namespace ICSL::Constants;
using namespace TNT;

inline DTYPE fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
vector<pair<Array2D<DTYPE>, Array2D<DTYPE> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
													Array2D<DTYPE> const &mv, Array2D<DTYPE> const &Sv, 
													DTYPE const &mz, DTYPE const &varz, 
													DTYPE const &focalLength, DTYPE const &dt,
													Array2D<DTYPE> const &omega);
Array2D<DTYPE> calcCorrespondence(vector<pair<Array2D<DTYPE>, Array2D<DTYPE> > > const &priorDistList, 
									vector<cv::Point2f> const &curPointList, 
									Array2D<DTYPE> const &Sn, 
									Array2D<DTYPE> const &SnInv);

void computeMAPEstimate(Array2D<DTYPE> &velMAP /*out*/, Array2D<DTYPE> &covVel /*out*/, DTYPE &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<DTYPE> const &C, // correspondence matrix
						Array2D<DTYPE> const &mv, // velocity mean
						Array2D<DTYPE> const &Sv, // velocity covariance
						DTYPE const &mz, // height mean
						DTYPE const &vz, // height variance
						Array2D<DTYPE> const &Sn, // feature measurement covariance
						DTYPE const &focalLength, DTYPE const &dt, Array2D<DTYPE> const &omega);

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
						int maxPointCnt);

//template<typename T>
//TNT::Array2D<T> logSO3(TNT::Array2D<T> const &R, DTYPE theta)
//{
//	Array2D<T> w(3,1);
//	if(abs(theta) > 0)
//	{
//		DTYPE sTheta2 = 2.0*sin(theta);
//		w[0][0] = (R[2][1]-R[1][2])/sTheta2;
//		w[1][0] = (R[0][2]-R[2][0])/sTheta2;
//		w[2][0] = (R[1][0]-R[0][1])/sTheta2;
//	}
//	else
//		w = Array2D<T>(3,1,0.0);
//
//	return w;
//}

extern DTYPE rs0, rs1, rs2, rs3, rs4, rs5;


}
}

#endif
