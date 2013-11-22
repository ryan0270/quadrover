#ifndef ICSL_LISTERNERS_H
#define ICSL_LISTERNERS_H
#include <memory>
#include "Data.h"
#include "TNT/tnt.h"
#include "toadlet/egg.h"

namespace ICSL {
namespace Quadrotor {
class AttitudeThrustControllerListener
{
	public:
	AttitudeThrustControllerListener(){};
	virtual ~AttitudeThrustControllerListener(){};

	virtual void onAttitudeThrustControllerCmdsSent(double const cmds[4])=0;
};

class Observer_AngularListener
{
	public:
	virtual ~Observer_AngularListener(){};

	virtual void onObserver_AngularUpdated(const std::shared_ptr<SO3Data<double>> &attData, const std::shared_ptr<DataVector<double>> &angularVelData)=0;
};

class Observer_TranslationalListener
{
	public:
	Observer_TranslationalListener(){};
	virtual ~Observer_TranslationalListener(){};

	virtual void onObserver_TranslationalUpdated(const TNT::Array2D<double> &pos, const TNT::Array2D<double> &vel)=0;
	virtual void onObserver_TranslationalImageProcessed(const shared_ptr<ImageTranslationData> &data){};
};

class TranslationControllerListener
{
	public:
	TranslationControllerListener(){};
	virtual ~TranslationControllerListener(){};

	virtual void onTranslationControllerAccelCmdUpdated(const TNT::Array2D<double> &accelCmd)=0;
};

class MotorInterfaceListener
{
	public:
	virtual void onMotorWarmupDone()=0;
};

class SensorManagerListener
{
	public:
	virtual void onNewSensorUpdate(const std::shared_ptr<IData> &data)=0;
};

class FeatureFinderListener
{
	public:
		virtual ~FeatureFinderListener(){};

		virtual void onFeaturesFound(const std::shared_ptr<ImageFeatureData> &data)=0;
};

class VelocityEstimatorListener
{
	public:
	VelocityEstimatorListener(){};
	virtual ~VelocityEstimatorListener(){};

	virtual void onVelocityEstimator_newEstimate(const std::shared_ptr<DataVector<double> > &velData, const std::shared_ptr<Data<double> > &heightData)=0;
};

class TargetFinderListener
{
	public:
	virtual ~TargetFinderListener(){};

	virtual void onTargetFound(const std::shared_ptr<ImageTargetFindData> &data)=0;
};

class RegionFinderListener
{
	public:
	virtual void onRegionsFound(const std::shared_ptr<ImageRegionLocData> &data)=0;
};

class SonarListener
{
	public:
	virtual void onNewSonar(const shared_ptr<HeightData<double>> &data)=0;
};

class CommManagerListener
{
	public:
	explicit CommManagerListener(){};
	virtual ~CommManagerListener(){};

	virtual void onNewCommMotorOn(){};
	virtual void onNewCommMotorOff(){};
//	virtual void onNewCommGainPID(float const rollPID[3], float const pitchPID[3], float yawPID[3]){};
	virtual void onNewCommMotorTrim(const int trim[4]){};
	virtual void onNewCommObserverReset(){};
	virtual void onNewCommAttObserverGain(double gainP, double gainI, double accelWeight, double magWeight){};
	virtual void onNewCommTimeSync(int time){};
	virtual void onNewCommLogTransfer(){};
//	virtual void onNewCommControlSystemGains(const Collection<float> &gains){};
	virtual void onNewCommSendControlSystem(const toadlet::egg::Collection<toadlet::tbyte> &buff){};
	virtual void onNewCommLogMask(uint32_t mask){};
	virtual void onNewCommLogClear(){};
	virtual void onNewCommStateVicon(const toadlet::egg::Collection<float> &data){};
	virtual void onNewCommDesState(const toadlet::egg::Collection<float> &data){};
	virtual void onCommConnectionLost(){};
	virtual void onNewCommForceGain(float k){};
	virtual void onNewCommTorqueGain(float k){};
	virtual void onNewCommMass(float m){};
	virtual void onNewCommIbvsGains(const toadlet::egg::Collection<float> &posGains, const toadlet::egg::Collection<float> &velGains){};
	virtual void onNewCommUseIbvs(bool useIbvs){};
	virtual void onNewCommAccelBias(float xBias, float yBias, float zBias){};
	virtual void onNewCommForceGainAdaptRate(float rate){};
	virtual void onNewCommAttitudeGains(const toadlet::egg::Collection<float> &gains){};
	virtual void onNewCommTransGains(const toadlet::egg::Collection<float> &gains){};
	virtual void onNewCommKalmanMeasVar(const toadlet::egg::Collection<float> &std){};
	virtual void onNewCommKalmanDynVar(const toadlet::egg::Collection<float> &std){};
	virtual void onNewCommNominalMag(const toadlet::egg::Collection<float> &nomMag){};
	virtual void onNewCommMotorArmLength(float l){};
	virtual void onNewCommImgBufferSize(int size){};
	virtual void onNewCommBarometerZeroHeight(float h){};
	virtual void onNewCommVisionFeatureFindQualityLevel(float qLevel){};
	virtual void onNewCommVisionFeatureFindSeparationDistance(int sepDist){};
	virtual void onNewCommVisionFeatureFindFASTThreshold(int thresh){};
	virtual void onNewCommVisionFeatureFindPointCntTarget(int target){};
	virtual void onNewCommVisionFeatureFindFASTAdaptRate(float r){};
	virtual void onNewCommVelEstMeasCov(float measCov){};
	virtual void onNewCommVelEstProbNoCorr(float probNoCorr){};
	virtual void onNewCommSetDesiredPos(){};
	virtual void onNewCommViconCameraOffset(float x, float y, float z){};
	virtual void onNewCommTargetNominalLength(float length){};
	virtual void onNewCommMAPHeightMeasCov(float cov){};
}; // class CommManagerListener
}
}
#endif
