#ifndef ICSL_ACTIVEREGION_H
#define ICSL_ACTIVEREGION_H
#include <vector>
#include <memory>
#include <iostream>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "toadlet/egg.h"

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "Time.h"
#include "constants.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
class ActiveRegion : public enable_shared_from_this<ActiveRegion>
{
	public:
	ActiveRegion();
	ActiveRegion(std::vector<cv::Point> points);

	void copyData(const ActiveRegion &ao);
	void markFound(const Time &time);
	void addLife(float val);
	void setPosCov(const TNT::Array2D<double> cov){mPosCov.inject(cov);}
	void takeLife(float val);
	void kill();
	bool isAlive() const {return mLife > 0;}
	void addNeighbor(shared_ptr<ActiveRegion> n, bool doTwoWay);
	void removeNeigbor(int nid, bool doTwoWay);

	const cv::Point2f &getFoundPos() const {return mFoundPos;}
	const cv::Point2f &getPrevFoundPos() const {return mPrevFoundPos;}
	const TNT::Array2D<double> getExpectedPos() const {return mExpectedPos;}
	const Time &getLastFoundTime() const {return mLastFoundTime;}
	const vector<cv::Point> &getContour() const {return mContour;}
	const TNT::Array2D<double> &getPrincipalAxes() const {return mPrincipalAxes;}
	const vector<double> &getPrincipalAxesEigVal() const {return mPrincipalAxesEigVal;}
	float getLife() const {return mLife;}
	float getArea() const {return mMoments.m00;}
	int getId() const {return mId;}
	const vector<shared_ptr<ActiveRegion>> &getNeighbors() const {return mNeighbors;}
	const Time &getCreateTime() const {return mCreateTime;}

	// assumes moments have already been calculated
	void calcPrincipalAxes();

	void updatePositionDistribution(const TNT::Array2D<double> &mv, const TNT::Array2D<double> &Sv, 
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
	{ return ao1->mLife > ao2->mLife; }

	static float MAX_LIFE;

	protected:
	std::vector<cv::Point> mContour;
	cv::Point2f mFoundPos, mPrevFoundPos;
	ICSL::Quadrotor::Time mLastFoundTime, mCreateTime;;
	float mLife;
	cv::Moments mMoments;
	int mId;

	TNT::Array2D<double> mPrincipalAxes;
	vector<double> mPrincipalAxesEigVal;
	
	TNT::Array2D<double> mExpectedPos, mPosCov;

	vector<shared_ptr<ActiveRegion>> mNeighbors;

	static size_t lastID;
	static std::mutex mutex_lastID;
	static inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
};

class RegionMatch
{
	public:
	std::shared_ptr<ActiveRegion> aoPrev, aoCur;
	float score;
};

}
}
#endif
