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

inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
													Array2D<double> const &mv, Array2D<double> const &Sv, 
													double const &mz, double const &varz, 
													double const &focalLength, double const &dt,
													Array2D<double> const &omega);
Array2D<double> calcCorrespondence(vector<pair<Array2D<double>, Array2D<double> > > const &priorDistList, 
									vector<cv::Point2f> const &curPointList, 
									Array2D<double> const &Sn, 
									Array2D<double> const &SnInv);

void computeMAPEstimate(Array2D<double> &velMAP /*out*/, double &heightMAP /*out*/,
						vector<cv::Point2f> const &prevPoints,
						vector<cv::Point2f> const &curPoints, 
						Array2D<double> const &C, // correspondence matrix
						Array2D<double> const &mv, // velocity mean
						Array2D<double> const &Sv, // velocity covariance
						double const &mz, // height mean
						double const &vz, // height variance
						Array2D<double> const &Sn, // feature measurement covariance
						double const &focalLength, double const &dt, Array2D<double> const &omega);

template<typename T>
TNT::Array2D<T> logSO3(TNT::Array2D<T> const &R, double theta)
{
	Array2D<T> w(3,1);
	if(abs(theta) > 0)
	{
		double sTheta2 = 2.0*sin(theta);
		w[0][0] = (R[2][1]-R[1][2])/sTheta2;
		w[1][0] = (R[0][2]-R[2][0])/sTheta2;
		w[2][0] = (R[1][0]-R[0][1])/sTheta2;
	}
	else
		w = Array2D<double>(3,1,0.0);

	return w;
}


}
}

#endif
