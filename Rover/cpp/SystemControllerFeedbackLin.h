#ifndef CLASS_SYSTEMCONTROLLERFEEDBACKLIN
#define CLASS_SYSTEMCONTROLLERFEEDBACKLIN
#include <toadlet/toadlet.h>

#include "TNT/tnt.h"

#include "ICSL/constants.h"

#include "Time.h"
#include "CommManager.h"
#include "QuadLogger.h"
#include "Observer_Angular.h"
#include "Observer_Translational.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;

class SystemControllerFeedbackLin : public CommManagerListener, 
									public Observer_AngularListener,
									public Observer_TranslationalListener
{
	public:

		SystemControllerFeedbackLin();
		virtual ~SystemControllerFeedbackLin(){};

		void setGain(TNT::Array2D<double> const &gain){mGain.inject(gain);}
		void setIntegratorGain(TNT::Array2D<double> const &gain){mGainInt.inject(gain);}
		void setIntegratorLimit(TNT::Array2D<double> const &limit){mErrIntLimit.inject(limit);}
		void setDesiredState(TNT::Array2D<double> const &x);
		void setDesPosAccel(TNT::Array2D<double> const &a);
		void setMass(double m){mMass = m;}
		void setForceScaling(double k){mForceScaling = k;}

		void useFilteredStates(bool b){mUseFilteredStates = b;}

		TNT::Array2D<double> const getDesiredState(){mMutex_data.lock(); Array2D<double> tempState = mDesiredState.copy(); mMutex_data.unlock(); return tempState;}
		TNT::Array2D<double> const getCurState(){mMutex_data.lock(); Array2D<double> tempState = mCurState.copy(); mMutex_data.unlock(); return tempState;}
		TNT::Array2D<double> const getErrorMemory(){mMutex_data.lock(); Array2D<double> tempInt = mErrInt.copy(); mMutex_data.unlock(); return tempInt;}

		TNT::Array2D<double> calcControl(TNT::Array2D<double> const &curAct);

		void resetErrorMemory(){mMutex_data.lock(); mErrInt = Array2D<double>(12,1,0.0); mMutex_data.unlock();}

		void setStartTime(Time t){mStartTime = t;}
		void setQuadLogger(QuadLogger *log){mQuadLogger = log;}
		void setRotViconToPhone(TNT::Array2D<double> const &rot){mRotViconToPhone.inject(rot);}

		// from CommManagerListener
		void onNewCommPosControllerGains(float const gainP[12], float const gainI[12], float const gainILimit[12], float mass, float forceScaling);

		// for Observer_AngularListener
		void onObserver_AngularUpdated(TNT::Array2D<double> const &att, TNT::Array2D<double> const &angularVel);

		// for Observer_TranslationalListener
		void onObserver_TranslationalUpdated(TNT::Array2D<double> const &pos, TNT::Array2D<double> const &vel);

	protected:
		Time mStartTime;
		QuadLogger *mQuadLogger;
		bool mUseFilteredStates;
		double mMass, mForceScaling;
		TNT::Array2D<double> mCurErrorRaw, mCurErrorFilt;
		TNT::Array2D<double> mCurState, mDesiredState, mDesPosAccel;
//		ICSL::ISystemModelAffine *mDynamicModel;
		TNT::Array2D<double> mGain, mGainInt;
		TNT::Array2D<double> mErrInt, mErrIntLimit;
		TNT::Array2D<double> mRotViconToPhone;

		toadlet::egg::Mutex mMutex_data;

		Time mLastControlTime;

		static double constrain(double val, double minVal, double maxVal)
		{ return min(maxVal, max(minVal, val)); }
};
}
}
#endif
