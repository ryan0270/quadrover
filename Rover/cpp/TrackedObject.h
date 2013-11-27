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

class TrackedObject
{
	public:
	TrackedObject();
	virtual ~TrackedObject(){};

	virtual void markFound(const Time &time, const cv::Point2f &pos);
	virtual void setPosCov(const TNT::Array2D<double> cov){mPosCov.inject(cov);}
	virtual void kill();
	virtual bool isAlive() const {return mIsAlive;}

	virtual const cv::Point2f &getLocation() const {return mLocation;}
	virtual const TNT::Array2D<double> &getExpectedPos() const {return mExpectedPos;}
	virtual const Time &getLastFoundTime() const {return mLastFoundTime;}
	virtual int getId() const {return mId;}
	virtual const Time &getCreateTime() const;
	virtual double getAge() const {return mHistory[0].first.getElapsedTimeNS()/1.0e9;}
	virtual const vector<pair<Time, cv::Point2f>> &getHistory() const {return mHistory;}

	// assumes location distributions have already been calculated
	virtual void updatePositionDistribution(const TNT::Array2D<double> &mv, const TNT::Array2D<double> &Sv, 
						double mz, double varz, 
						double focalLength, const cv::Point2f &center,
						const TNT::Array2D<double> &omega,
						const Time &curTime);

	static TNT::Array2D<double> calcCorrespondence(const vector<shared_ptr<TrackedObject>> &prevPointList,
												   const vector<shared_ptr<TrackedObject>> &curPointList,
												   const TNT::Array2D<double> &Sn,
												   const TNT::Array2D<double> &SnInv,
												   double probNoCorr);

	static TNT::Array2D<double> calcCorrespondence2(const vector<shared_ptr<TrackedObject>> &prevPointList,
												   const vector<shared_ptr<TrackedObject>> &curPointList,
												   double probNoCorr);

	// a hack leftover from the past, when I gave objects life and could sort on that
	// this will put alive objects first and dead objects at the back
	static bool sortAlivePredicate(const std::shared_ptr<TrackedObject> &to1, const std::shared_ptr<TrackedObject> &to2)
	{ return to1->mIsAlive && !to2->mIsAlive; }

	// sorts oldest first
	static bool sortAgePredicate(const std::shared_ptr<TrackedObject> &to1, const std::shared_ptr<TrackedObject> &to2)
	{ return to1->mHistory[0].first < to2->mHistory[0].first; }

	protected:
	cv::Point2f mLocation;
	ICSL::Quadrotor::Time mLastFoundTime;
	bool mIsAlive;
	int mId;

	vector<pair<Time, cv::Point2f>> mHistory;

	TNT::Array2D<double> mExpectedPos, mPosCov;

	static size_t lastID;
	static std::mutex mutex_lastID;
	static inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
};

class ObjectMatch
{
	public:
	std::shared_ptr<TrackedObject> object;
	cv::Point2f prevPos, curPos;
	
	float score;
};

class TrackedPoint : public TrackedObject
{
	public:
	TrackedPoint();
	TrackedPoint(const Time &time, const cv::Point2f &point);
	virtual ~TrackedPoint(){};
};

}
}
#endif
