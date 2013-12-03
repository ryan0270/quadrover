#ifndef ICSL_VELOCITY_ESTIMATOR_H
#define ICSL_VELOCITY_ESTIMATOR_H
#include <memory>
#include <sched.h>
#include <thread>
#include <mutex>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TNT/tnt.h"
#include "TNT/jama_lu.h"
#include "TNT/jama_cholesky.h"

#include "Data.h"
#include "Time.h"
#include "QuadLogger.h"
#include "Observer_Translational.h"
#include "FeatureFinder.h"
#include "Listeners.h"

#include "toadlet/egg.h"

namespace ICSL {
namespace Quadrotor {
using namespace TNT;

class VelocityEstimator : public FeatureFinderListener,
						  public RegionFinderListener,
						  public CommManagerListener
{
	public:
	VelocityEstimator();
	virtual ~VelocityEstimator();

	void start(){ thread th(&VelocityEstimator::run, this); th.detach(); }
	void run();
	void shutdown();
	void initialize();

	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};

	void setStartTime(Time t){mStartTime.setTime(t);}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setObserverTranslational(Observer_Translational *obsv){mObsvTranslational = obsv;}
	void setRotPhoneToCam(const TNT::Array2D<double> &rot);

	void addListener(VelocityEstimatorListener *l){mListeners.push_back(l);}

	uint32_t getLastVisionDelayTimeUS(){mMutex_data.lock(); uint32_t temp=mLastDelayTimeUS; mMutex_data.unlock(); return temp;};

	// for CommManagerListener
	void onNewCommVelEstMeasCov(float measCov);
	void onNewCommVelEstProbNoCorr(float probNoCorr);

	// for FeatureFinderListener
	void onFeaturesFound(const shared_ptr<ImageFeatureData> &data);

	// for RegionFinderListener 
	void onRegionsFound(const shared_ptr<ImageRegionData> &data);

	protected:
	bool mRunning, mDone;
	int mThreadPriority, mScheduler;
	TNT::Array2D<double> mRotPhoneToCam, mRotCamToPhone;
	TNT::Array2D<double> mRotPhoneToCam2, mRotCamToPhone2;
	QuadLogger *mQuadLogger;
	Time mStartTime;

	Collection<VelocityEstimatorListener*> mListeners;

	bool mNewImageDataAvailable, mNewRegionDataAvailable;
	shared_ptr<ImageFeatureData> mLastImageFeatureData;
	shared_ptr<ImageRegionData> mLastRegionData;

	std::mutex mMutex_imageData, mMutex_data, mMutex_params;

	Observer_Translational *mObsvTranslational;

	uint32_t mLastDelayTimeUS;

	float mMeasCov, mProbNoCorr;

	bool doVelocityEstimate(const shared_ptr<ImageFeatureData> oldFeatureData,
							const shared_ptr<ImageFeatureData> curFeatureData,
							TNT::Array2D<double> &velEstOUT,
							double &heightEstOUT,
							double visionMeasCov,
							double probNoCorr) const;
	bool doVelocityEstimate(const shared_ptr<ImageRegionData> oldRegionData,
						    const shared_ptr<ImageRegionData> curRegionData,
							Array2D<double> &velEst, 
							double &heightEst,
							double visionMeasCov,
							double probNoCorr) const;

	static inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
	static vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(const vector<cv::Point2f> &points, 
														const Array2D<double> &mv, const Array2D<double> &Sv, 
														double mz, double varz, 
														double focalLength, double dt,
														const Array2D<double> &omega);
	static Array2D<double> calcCorrespondence(const vector<pair<Array2D<double>, Array2D<double> > > &priorDistList, 
										const vector<cv::Point2f> &curPointList, 
										const Array2D<double> &Sn, 
										const Array2D<double> &SnInv,
										float probNoCorr);
	
	static void computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
							const vector<cv::Point2f> &prevPoints,
							const vector<cv::Point2f> &curPoints, 
							const Array2D<double> &C, // correspondence matrix
							const Array2D<double> &mv, // velocity mean
							const Array2D<double> &Sv, // velocity covariance
							double mz, // height mean
							double vz, // height variance
							const Array2D<double> &Sn, // feature measurement covariance
							double focalLength, double dt, const Array2D<double> &omega);
	
	static void computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
							const vector<cv::Point2f> &prevPoints,
							const vector<cv::Point2f> &curPoints, 
							const Array2D<double> &C, // correspondence matrix
							const Array2D<double> &mv, // velocity mean
							const Array2D<double> &Sv, // velocity covariance
							double mz, // height mean
							double vz, // height variance
							const Array2D<double> &Sn, // feature measurement covariance
							double focalLength, double dt, const Array2D<double> &omega,
							int maxPointCnt);
	
};

} // namespace Rover
} // namespace ICSL

#endif
