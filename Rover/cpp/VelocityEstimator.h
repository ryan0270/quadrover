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

#include "toadlet/egg.h"

#include "TNT/tnt.h"
#include "TNT/jama_lu.h"

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

class VelocityEstimator : //public VisionProcessorListener,
						  public FeatureFinderListener,
						  public toadlet::egg::Thread
{
	public:
	VelocityEstimator();
	virtual ~VelocityEstimator();

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

	void doVelocityEstimate(shared_ptr<ImageFeatureData> const &featureData) const;

};

} // namespace Rover
} // namespace ICSL

#endif
#endif //ICSL_VELOCITY_ESTIMATOR_LISTENER_ONLY
