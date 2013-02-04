#ifndef CLASS_SYSTEMCONTROLLERHIERARCHICAL
#define CLASS_SYSTEMCONTROLLERHIERARCHICAL
#pragma warning(disable : 4996)
#include <exception>
#include <string>
#include <mxml.h>

#include <QWidget>
#include <QtGui>

#include "toadlet/egg.h"

#include "TNT/tnt.h"

#include "ICSL/constants.h"
#include "ICSL/SystemModel/ISystemModelAffine.h"
#include "Quadrotor/SystemModelQuadrotor/src/SystemModelQuadrotor.h"

namespace ICSL{
namespace Quadrotor{
	using namespace std;

	class SystemControllerHierarchicalException : public exception
	{
		public:
			SystemControllerHierarchicalException(){mMessage = "SystemControllerHierarchical exception occurred";};
			SystemControllerHierarchicalException(string msg){mMessage = msg;};
			~SystemControllerHierarchicalException() throw(){};
			const char* what() const throw() {return mMessage.c_str();};

		protected:
			string mMessage;
	};

	class SystemControllerHierarchical
	{
		public:

			SystemControllerHierarchical();
			virtual ~SystemControllerHierarchical(){};

			void setSystemModel(ICSL::ISystemModelAffine *sysModel){mDynamicModel = sysModel;};
			void setIntegratorGain(double newGain){mIntSys.setGain(newGain);};
			void setIntegratorSatLimit(double newLimit){mIntSys.setSatLimit(newLimit);};
			void setIntegratorPosLimit(double newLimit){mIntSys.setPosLimit(newLimit);};
			void setDesiredState(TNT::Array2D<double> const &x);
			void setDeltaT(double dt){mDeltaT = dt;};
			void useFilteredStates(bool b){mUseFilteredStates = b;}

			TNT::Array2D<double> const getDesiredState(){mMutex_dataAccess.lock(); Array2D<double> tempState = mDesiredState.copy(); mMutex_dataAccess.unlock(); return tempState;}
			TNT::Array2D<double> const getErrorMemory(){mMutex_dataAccess.lock(); Array2D<double> temp = stackVertical(mIntSys.getVal(), stackVertical(mIntSys.getValDot(),mIntSys.getValDDot())); mMutex_dataAccess.unlock(); return temp;}

			TNT::Array2D<double> calcControl();

			void resetErrorMemory(){mIntSys.reset();}

			void populateConfigTree(QTreeWidgetItem *root);
			void applyConfigTree(QTreeWidgetItem *root);
			void saveConfig(mxml_node_t *root);
			void loadConfig(mxml_node_t *root);
			

		protected:
			bool mUseFilteredStates;
			double mDeltaT;
			TNT::Array2D<double> mDesiredState;
			ICSL::ISystemModelAffine *mDynamicModel;
			TNT::Array2D<double> mAccelRef;
			double mAccelPosLimit;
			double mGainThrust, mGainP, mGainV, mGainYaw;
			
			SystemModelLinear mErrorFilterSys;
			string mErrorFilterSysFilename;

			toadlet::egg::Mutex mMutex_dataAccess;

			static double constrain(double val, double minVal, double maxVal)
			{ return min(maxVal, max(minVal, val)); }

			static double integrationLimiter(double s, double limit)
			{
				return limit/sqrt(1+s);
			}

			class IntegratorSystem
			{
			public:
				explicit IntegratorSystem()
				{
					mGain=1;
					mVal=TNT::Array2D<double>(3, 1, 0.0);
					mValDot=TNT::Array2D<double>(3, 1, 0.0);
					mValDDot=TNT::Array2D<double>(3, 1, 0.0);
					mPosLimit = 1;
					mSatLimit = 0;
				}
				virtual ~IntegratorSystem(){};
				void update(TNT::Array2D<double> const &posErr, double dt)
				{
					TNT::Array2D<double> valSat(3, 1);
					for (int i=0 ; i <3 ; i++)
						valSat[i][0] = SystemControllerHierarchical::constrain(mVal[i][0], -mSatLimit, mSatLimit);
					mValDDot=-2.0*mGain*mValDot - mGain*mGain*(mVal-valSat) + mGain*SystemControllerHierarchical::integrationLimiter(TNT::norm2(posErr)*TNT::norm2(posErr), mPosLimit)*posErr;
					mValDot +=	dt*mValDDot;
					mVal += dt*mValDot;
				
				}

				TNT::Array2D<double> getVal(){ return mVal; };
				TNT::Array2D<double> getValDot(){ return mValDot; };
				TNT::Array2D<double> getValDDot(){ return mValDDot;};
				double getGain(){ return mGain; };
				double getSatLimit(){ return mSatLimit; };
				double getPosLimit(){ return mPosLimit; };
				
				void reset()
				{
					mVal=TNT::Array2D<double>(3, 1, 0.0);
					mValDot=TNT::Array2D<double>(3, 1, 0.0);
					mValDDot=TNT::Array2D<double>(3, 1, 0.0);
				}
				
				void setGain(double newGain){ mGain = newGain;};
				void setSatLimit(double newLimit){ mSatLimit = newLimit;};
				void setPosLimit(double newLimit){ mPosLimit = newLimit;};
				
			protected:
				double mGain;
				TNT::Array2D<double> mVal, mValDot, mValDDot;
				double mSatLimit, mPosLimit;
			};
			IntegratorSystem mIntSys;
	};
}
}
#endif
