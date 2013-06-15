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

namespace ICSL{
namespace Rover {
using namespace std;
using namespace ICSL::Quadrotor;
using namespace ICSL::Constants;
using namespace TNT;

inline float fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
vector<pair<Array2D<float>, Array2D<float> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
													Array2D<float> const &mv, Array2D<float> const &Sv, 
													float const &mz, float const &varz, 
													float const &focalLength, float const &dt,
													Array2D<float> const &omega);
Array2D<float> calcCorrespondence(vector<pair<Array2D<float>, Array2D<float> > > const &priorDistList, 
									vector<cv::Point2f> const &curPointList, 
									Array2D<float> const &Sn, 
									Array2D<float> const &SnInv);

void computeMAPEstimate(Array2D<float> &velMAP /*out*/, Array2D<float> &covVel /*out*/, float &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<float> const &C, // correspondence matrix
						Array2D<float> const &mv, // velocity mean
						Array2D<float> const &Sv, // velocity covariance
						float const &mz, // height mean
						float const &vz, // height variance
						Array2D<float> const &Sn, // feature measurement covariance
						float const &focalLength, float const &dt, Array2D<float> const &omega);

void computeMAPEstimate(Array2D<float> &velMAP /*out*/, Array2D<float> &covVel /*out*/, float &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<float> const &C, // correspondence matrix
						Array2D<float> const &mv, // velocity mean
						Array2D<float> const &Sv, // velocity covariance
						float const &mz, // height mean
						float const &vz, // height variance
						Array2D<float> const &Sn, // feature measurement covariance
						float const &focalLength, float const &dt, Array2D<float> const &omega,
						int maxPointCnt);

template<typename T>
TNT::Array2D<T> logSO3(TNT::Array2D<T> const &R, float theta)
{
	Array2D<T> w(3,1);
	if(abs(theta) > 0)
	{
		float sTheta2 = 2.0*sin(theta);
		w[0][0] = (R[2][1]-R[1][2])/sTheta2;
		w[1][0] = (R[0][2]-R[2][0])/sTheta2;
		w[2][0] = (R[1][0]-R[0][1])/sTheta2;
	}
	else
		w = Array2D<float>(3,1,0.0);

	return w;
}

extern float rs0, rs1, rs2, rs3, rs4, rs5;


}
}

#endif
