#ifndef ICSL_VELOCITY_ESTIMATOR_LISTENER_H
#define ICSL_VELOCITY_ESTIMATOR_LISTENER_H

#include <memory>
#include "Data.h"

namespace ICSL {
namespace Quadrotor { 
//using namespace std; 
class VelocityEstimatorListener
{
	public:
	VelocityEstimatorListener(){};
	virtual ~VelocityEstimatorListener(){};

	virtual void onVelocityEstimator_newEstimate(shared_ptr<DataVector<double> > const &velData, shared_ptr<Data<double> > const &heightData)=0;
};
}}
#endif

#ifndef ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
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
#include "Observer_Angular.h"
#include "Observer_Translational.h"
#include "FeatureFinder.h"

#include "toadlet/egg.h"

namespace ICSL {
namespace Quadrotor {

//using namespace std;
using namespace TNT;

class VelocityEstimator : public FeatureFinderListener,
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
	void setRotPhoneToCam(const TNT::Array2D<double> &rot){mRotPhoneToCam.inject(rot); mRotCamToPhone.inject(transpose(rot));}

	void addListener(VelocityEstimatorListener *l){mListeners.push_back(l);}

	uint32 getLastVisionDelayTimeUS(){mMutex_data.lock(); uint32 temp=mLastDelayTimeUS; mMutex_data.unlock(); return temp;};

	// for CommManagerListener
	void onNewCommVelEstMeasCov(const float &measCov);
	void onNewCommVelEstProbNoCorr(const float &probNoCorr);

	// for FeatureFinderListener
	void onFeaturesFound(const shared_ptr<ImageFeatureData> &data);

	protected:
	bool mRunning, mDone;
	int mThreadPriority, mScheduler;
	TNT::Array2D<double> mRotPhoneToCam, mRotCamToPhone;
	QuadLogger *mQuadLogger;
	Time mStartTime;

	Collection<VelocityEstimatorListener*> mListeners;

	bool mNewImageDataAvailable;
	shared_ptr<ImageFeatureData> mLastImageFeatureData;

	std::mutex mMutex_imageData, mMutex_data, mMutex_params;

	Observer_Translational *mObsvTranslational;

	uint32 mLastDelayTimeUS;

	float mMeasCov, mProbNoCorr;

	void doVelocityEstimate(shared_ptr<ImageFeatureData> oldFeatureData,
							shared_ptr<ImageFeatureData> curFeatureData,
							TNT::Array2D<double> &velEstOUT,
							double &heightEstOUT,
							double visionMeasCov,
							double probNoCorr) const;

	static inline double fact2ln(int n){return lgamma(2*n+1)-n*log(2)-lgamma(n+1);}
	static vector<pair<Array2D<double>, Array2D<double> > > calcPriorDistributions(const vector<cv::Point2f> &points, 
														const Array2D<double> &mv, const Array2D<double> &Sv, 
														const double &mz, const double &varz, 
														const double &focalLength, const double &dt,
														const Array2D<double> &omega);
	static Array2D<double> calcCorrespondence(const vector<pair<Array2D<double>, Array2D<double> > > &priorDistList, 
										const vector<cv::Point2f> &curPointList, 
										const Array2D<double> &Sn, 
										const Array2D<double> &SnInv,
										const float &probNoCorr);
	
	static void computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
							const vector<cv::Point2f> &prevPoints,
							const vector<cv::Point2f> &curPoints, 
							const Array2D<double> &C, // correspondence matrix
							const Array2D<double> &mv, // velocity mean
							const Array2D<double> &Sv, // velocity covariance
							const double &mz, // height mean
							const double &vz, // height variance
							const Array2D<double> &Sn, // feature measurement covariance
							const double &focalLength, const double &dt, const Array2D<double> &omega);
	
	static void computeMAPEstimate(Array2D<double> &velMAP /*out*/, Array2D<double> &covVel /*out*/, double &heightMAP /*out*/,
							const vector<cv::Point2f> &prevPoints,
							const vector<cv::Point2f> &curPoints, 
							const Array2D<double> &C, // correspondence matrix
							const Array2D<double> &mv, // velocity mean
							const Array2D<double> &Sv, // velocity covariance
							const double &mz, // height mean
							const double &vz, // height variance
							const Array2D<double> &Sn, // feature measurement covariance
							const double &focalLength, const double &dt, const Array2D<double> &omega,
							int maxPointCnt);
	
};

} // namespace Rover
} // namespace ICSL

#endif
#endif //ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
