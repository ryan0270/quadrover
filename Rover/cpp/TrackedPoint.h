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
class TrackedPoint
{
	public:
	TrackedPoint();
	TrackedPoint(const Time &time, const cv::Point2f &point);

	void copyData(const TrackedPoint &tp);
	void markFound(const Time &time, const cv::Point2f &pos);
	void addLife(float val);
	void setPosCov(const TNT::Array2D<double> cov){mPosCov.inject(cov);}
	void takeLife(float val);
	void kill();
	bool isAlive() const {return mLife > 0;}

	const cv::Point2f &getPos() const {return mPoint;}
	const TNT::Array2D<double> getExpectedPos() const {return mExpectedPos;}
	const Time &getLastFoundTime() const {return mLastFoundTime;}
	float getLife() const {return mLife;}
	int getId() const {return mId;}
	const Time &getCreateTime() const {return mCreateTime;}
	const vector<pair<Time, cv::Point2f>> &getHistory(){return mHistory;}

	// assumes have already been calculated
	void updatePositionDistribution(const TNT::Array2D<double> &mv, const TNT::Array2D<double> &Sv, 
						double mz, double varz, 
						double focalLength, const cv::Point2f &center,
						const TNT::Array2D<double> &omega,
						const Time &curTime);

	static TNT::Array2D<double> calcCorrespondence(const vector<shared_ptr<TrackedPoint>> &prevPointList,
												   const vector<shared_ptr<TrackedPoint>> &curPointList,
												   const TNT::Array2D<double> &Sn,
												   const TNT::Array2D<double> &SnInv,
												   double probNoCorr);

	static bool sortPredicate(const std::shared_ptr<TrackedPoint> &ao1, const std::shared_ptr<TrackedPoint> &ao2)
	{ return ao1->mLife > ao2->mLife; }

	static float MAX_LIFE;

	protected:
	cv::Point2f mPoint;
	ICSL::Quadrotor::Time mLastFoundTime, mCreateTime;;
	float mLife;
	int mId;

	vector<pair<Time, cv::Point2f>> mHistory;

	TNT::Array2D<double> mExpectedPos, mPosCov;

	static size_t lastID;
	static std::mutex mutex_lastID;
	static inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
};

class PointMatch
{
	public:
	std::shared_ptr<TrackedPoint> tpPrev, tpCur;
	float score;
};

}
}
#endif
