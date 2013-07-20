#ifndef ICSL_VELOCITY_ESTIMATOR_LISTENER_H
#define ICSL_VELOCITY_ESTIMATOR_LISTENER_H

#include <memory>
#include "Data.h"

namespace ICSL {
namespace Quadrotor { 
using namespace std; 
class VelocityEstimatorListener
{
	public:
	VelocityEstimatorListener(){};
	virtual ~VelocityEstimatorListener(){};

	virtual void onVelocityEstimator_newEstimate(shared_ptr<DataVector<double> > const &velData)=0;
};
}}
#endif

#ifndef ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
#ifndef ICSL_VELOCITY_ESTIMATOR_H
#define ICSL_VELOCITY_ESTIMATOR_H
#include <memory>
#include <sched.h>
#include <thread>

#include "toadlet/egg.h"

#include "TNT/tnt.h"
#include "TNT/jama_lu.h"
#include "TNT/jama_cholesky.h"

#include "Time.h"
#include "QuadLogger.h"
#include "Data.h"
#include "Observer_Angular.h"
#include "Observer_Translational.h"
#include "VisionProcessor.h"
#include "FeatureFinder.h"

namespace ICSL {
namespace Quadrotor {

using namespace std;

class VelocityEstimator : public FeatureFinderListener
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
	void setRotPhoneToCam(TNT::Array2D<double> const &rot){mRotPhoneToCam.inject(rot); mRotCamToPhone.inject(transpose(rot));}

	void addListener(VelocityEstimatorListener *l){mListeners.push_back(l);}

	// for VisionProcessorListener
//	void onImageProcessed(shared_ptr<ImageMatchData> const data);
//	void onImageTargetFound(shared_ptr<ImageTargetFindData> const data){};
//	void onImageLost(){};

	// for FeatureFinderListener
	void onFeaturesFound(shared_ptr<ImageFeatureData> const &data);

	protected:
	bool mRunning, mDone;
	int mThreadPriority, mScheduler;
	TNT::Array2D<double> mRotPhoneToCam, mRotCamToPhone;
	QuadLogger *mQuadLogger;
	Time mStartTime;

	Collection<VelocityEstimatorListener*> mListeners;

	bool mNewImageDataAvailable;
	shared_ptr<ImageFeatureData> mLastImageFeatureData;

	toadlet::egg::Mutex mMutex_imageData;

	Observer_Translational *mObsvTranslational;

	void doVelocityEstimate(shared_ptr<ImageFeatureData> const &oldFeatureData,
							shared_ptr<ImageFeatureData> const &curFeatureData,
							TNT::Array2D<double> &velEstOUT,
							double &heightEstOUT) const;

	static inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
	static vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(vector<cv::Point2f> const &points, 
														Array2D<double> const &mv, Array2D<double> const &Sv, 
														double const &mz, double const &varz, 
														double const &focalLength, double const &dt,
														Array2D<double> const &omega);
	static Array2D<double> calcCorrespondence(vector<pair<Array2D<double>, Array2D<double> > > const &priorDistList, 
										vector<cv::Point2f> const &curPointList, 
										Array2D<double> const &Sn, 
										Array2D<double> const &SnInv);
	
	static void computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
							vector<cv::Point2f> const &prevPoints,
							vector<cv::Point2f> const &curPoints, 
							Array2D<double> const &C, // correspondence matrix
							Array2D<double> const &mv, // velocity mean
							Array2D<double> const &Sv, // velocity covariance
							double const &mz, // height mean
							double const &vz, // height variance
							Array2D<double> const &Sn, // feature measurement covariance
							double const &focalLength, double const &dt, Array2D<double> const &omega);
	
	static void computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
							vector<cv::Point2f> const &prevPoints,
							vector<cv::Point2f> const &curPoints, 
							Array2D<double> const &C, // correspondence matrix
							Array2D<double> const &mv, // velocity mean
							Array2D<double> const &Sv, // velocity covariance
							double const &mz, // height mean
							double const &vz, // height variance
							Array2D<double> const &Sn, // feature measurement covariance
							double const &focalLength, double const &dt, Array2D<double> const &omega,
							int maxPointCnt);
	
};

} // namespace Rover
} // namespace ICSL

#endif
#endif //ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
