#ifndef ICSL_OBJECT_TRACKER_H
#define ICSL_OBJECT_TRACKER_H
#include <memory>
#include <sched.h>
#include <thread>
#include <mutex>
#include <vector>
#include <list>

#include <opencv2/core/core.hpp>

#include "TNT/tnt.h"

#include "toadlet/egg.h"

#include "QuadLogger.h"
#include "Common.h"
#include "Time.h"
#include "Data.h"
#include "Listeners.h"
#include "TrackedObject.h"
#include <Observer_Translational.h>
#include <Observer_Angular.h>

namespace ICSL {
namespace Quadrotor {
using namespace std;

class ObjectTracker : public FeatureFinderListener,
					  public RegionFinderListener
{
	public:
	ObjectTracker();
	virtual ~ObjectTracker();

	void shutdown();
	void start(){ thread th(&ObjectTracker::run, this); th.detach(); }
	void initialize();
	void setThreadPriority(int sched, int priority){mScheduler = sched; mThreadPriority = priority;};
	void setStartTime(Time t){mStartTime = t;}
	void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
	void setObserverTranslation(Observer_Translational *obsv){mObsvTranslation = obsv;}
	void setObserverAngular(Observer_Angular *obsv){mObsvAngular = obsv;}

	const vector<shared_ptr<TrackedObject>> &getTrackedObjects() const {return mTrackedObjects;}

	static void drawResults(cv::Mat &img,
							const vector<ObjectMatch> &matches,
							const vector<shared_ptr<TrackedObject>> &repeatObjects,
							const vector<shared_ptr<TrackedObject>> &newObjects);

	void addListener(ObjectTrackerListener *listener){mListeners.push_back(listener);}

	// for FeatureFinderListener
	void onFeaturesFound(const shared_ptr<ImageFeatureData> &data);

	// for RegionFinderListener
	void onRegionsFound(const shared_ptr<ImageRegionData> &data);

	protected:
	bool mRunning, mFinished;
	QuadLogger *mQuadLogger;
	Time mStartTime;
	int mThreadPriority, mScheduler;

	bool mNewFeatureDataAvailable, mNewRegionDataAvailable;
	shared_ptr<ImageFeatureData> mFeatureData;
	shared_ptr<ImageRegionData> mRegionData;
	std::mutex mMutex_featureData;

	vector<shared_ptr<TrackedObject>> mTrackedObjects;

	vector<ObjectTrackerListener*> mListeners;

	Observer_Translational *mObsvTranslation;
	Observer_Angular *mObsvAngular;

	void run();

	void matchify(const std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedObject>> &curObjects,
				  std::vector<ICSL::Quadrotor::ObjectMatch> &goodMatches,
				  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedObject>> &repeatPoints,
				  std::vector<std::shared_ptr<ICSL::Quadrotor::TrackedObject>> &newPoints,
				  const TNT::Array2D<double> Sn,
				  const TNT::Array2D<double> SnInv,
				  double probNoCorr,
				  const ICSL::Quadrotor::Time &imageTime);

	static void doSimilarityCheck(vector<shared_ptr<TrackedObject>> &objects);
};

} // namespace Quadrotor
} // namespace ICSL 

#endif
