#ifndef CLASS_SYSTEMCONTROLLERFEEDBACKLIN
#define CLASS_SYSTEMCONTROLLERFEEDBACKLIN
#pragma warning(disable : 4996)
#include <exception>
#include <string>
#include <mxml.h>

#include "TNT/tnt.h"

#include "ICSL/constants.h"
#include "ICSL/SystemModel/ISystemModelAffine.h"
#include "ICSL/SystemModel/SystemModelLinear/src/SystemModelLinear.h"

#include "Quadrotor/SystemModelQuadrotor/src/SystemModelQuadrotor.h"

namespace ICSL{
namespace Quadrotor{
	using namespace std;

	class SystemControllerFeedbackLinException : public exception
	{
		public:
			SystemControllerFeedbackLinException(){mMessage = "SystemControllerFeedbackLin exception occurred";};
			SystemControllerFeedbackLinException(string msg){mMessage = msg;};
			~SystemControllerFeedbackLinException() throw(){};
			const char* what() const throw() {return mMessage.c_str();};

		protected:
			string mMessage;
	};

	class SystemControllerFeedbackLin
	{
		public:

			SystemControllerFeedbackLin();
			virtual ~SystemControllerFeedbackLin(){};

			void setSystemModel(ICSL::ISystemModelAffine *sysModel){mDynamicModel = sysModel;}
			void setGain(TNT::Array2D<double> const &gain){mGain.inject(gain);}
			void setIntegratorGain(TNT::Array2D<double> const &gain){mGainInt.inject(gain);}
			void setIntegratorLimit(TNT::Array2D<double> const &limit){mErrIntLimit.inject(limit);}
			void setDesiredState(TNT::Array2D<double> const &x);
			void setDesiredAccel(TNT::Array2D<double> const &a);
			void setDeltaT(double dt){mDeltaT = dt;}
			void useFilteredStates(bool b){mUseFilteredStates = b;}

			TNT::Array2D<double> const getDesiredState(){mMutex_DataAccess.lock(); Array2D<double> tempState = mDesiredState.copy(); mMutex_DataAccess.unlock(); return tempState;}
			TNT::Array2D<double> const getDesiredAccel(){mMutex_DataAccess.lock(); Array2D<double> tempState = mDesiredAccel.copy(); mMutex_DataAccess.unlock(); return tempState;}
			TNT::Array2D<double> const getErrorMemory(){mMutex_DataAccess.lock(); Array2D<double> tempInt = mErrInt.copy(); mMutex_DataAccess.unlock(); return tempInt;}
			TNT::Array2D<double> const getGainP(){mMutex_DataAccess.lock(); Array2D<double> temp = mGain.copy(); mMutex_DataAccess.unlock(); return temp;}
			TNT::Array2D<double> const getGainI(){mMutex_DataAccess.lock(); Array2D<double> temp = mGainInt.copy(); mMutex_DataAccess.unlock(); return temp;}
			TNT::Array2D<double> const getGainILimit(){mMutex_DataAccess.lock(); Array2D<double> temp = mErrIntLimit.copy(); mMutex_DataAccess.unlock(); return temp;}

			TNT::Array2D<double> calcControl();

			void resetErrorMemory(){mMutex_DataAccess.lock(); mErrInt = Array2D<double>(12,1,0.0); mMutex_DataAccess.unlock();}

			void populateConfigTree(QTreeWidgetItem *root);
			void applyConfigTree(QTreeWidgetItem *root);
			void saveConfig(mxml_node_t *root);
			void loadConfig(mxml_node_t *root);
			

		protected:
			bool mUseFilteredStates;
			double mDeltaT;
			TNT::Array2D<double> mDesiredState, mDesiredAccel;
			ICSL::ISystemModelAffine *mDynamicModel;
			TNT::Array2D<double> mGain, mGainInt;
			TNT::Array2D<double> mErrInt, mErrIntLimit;

			SystemModelLinear mErrorFilterSys;
			string mErrorFilterSysFilename;

			toadlet::egg::Mutex mMutex_DataAccess;

			static double constrain(double val, double minVal, double maxVal)
			{ return min(maxVal, max(minVal, val)); }
	};
}
}
#endif
