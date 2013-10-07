#ifndef ICSL_ACTIVEOBJECT_H
#define ICSL_ACTIVEOBJECT_H
#include <vector>
#include <memory>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "toadlet/egg.h"

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "Time.h"
#include "constants.h"

namespace ICSL{
using namespace std;
using namespace ICSL::Quadrotor;
class ActiveRegion
{
	public:
	ActiveRegion();
	ActiveRegion(std::vector<cv::Point> points);

	cv::Point lastCenter;
	ICSL::Quadrotor::Time lastFoundTime;
	float life;
	std::vector<cv::Point> contour;
	cv::Moments mom;
	double huMom[7];
	double centralMoms[7];
	int id;

	TNT::Array2D<double> principalAxes;
	vector<double> principalAxesEigVal;
	

	TNT::Array2D<double> expectedPos, posCov;

	void copyData(const ActiveRegion &ao);

	// assumes moments have already been calculated
	void calcPrincipalAxes();

	cv::Point2f meanPos;
	void updatePosition(const TNT::Array2D<double> &mv, const TNT::Array2D<double> &Sv, 
						double mz, double varz, 
						double focalLength, const cv::Point2f &center,
						const TNT::Array2D<double> &omega,
						const Time &curTime);

	static TNT::Array2D<double> calcCorrespondence(const vector<shared_ptr<ActiveRegion>> &prevObjectList,
												   const vector<shared_ptr<ActiveRegion>> &curObjectList,
												   const TNT::Array2D<double> &Sn,
												   const TNT::Array2D<double> &SnInv,
												   double varxi, // variance of the shape metric
												   double probNoCorr);

	static double calcShapeDistance(const shared_ptr<ActiveRegion> &ao1, const shared_ptr<ActiveRegion> &ao2);

	static bool sortPredicate(const std::shared_ptr<ActiveRegion> &ao1, const std::shared_ptr<ActiveRegion> &ao2)
	{ return ao1->life > ao2->life; }

	private:
	static unsigned long lastID;

	static inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}

	constexpr static double maxLife = 20;
};

class Match
{
	public:
	std::shared_ptr<ActiveRegion> aoPrev, aoCur;
	float score;
};

}
#endif
