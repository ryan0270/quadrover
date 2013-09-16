#ifndef CLASS_ISYSTEMMODELAFFINE
#define CLASS_ISYSTEMMODELAFFINE
#include <string>
#include "TNT/tnt.h"

#include "TNT_Utils.h"

#include "ISystemModel.h"

namespace ICSL
{
using namespace std;
using namespace TNT;
	/*!
	 * Interface class for systems of the form
	 * \dot{x} = f(x) + g(x) u
	 */
	class ISystemModelAffine : public ISystemModel
	{
		public:
		explicit ISystemModelAffine(){};
		ISystemModelAffine(int numStates, int numAct) : ISystemModel(numStates, numAct) {};
		virtual ~ISystemModelAffine(){};

		virtual Array2D<double> calcDriftVector(Array2D<double> const &curState)=0;
		virtual Array2D<double> calcActuatorGainMatrix(Array2D<double> const &curState)=0;
		virtual Array2D<double> calcStateDerivative(Array2D<double> const &curState, Array2D<double> const &curActuation)
		{
			mMutex_DataAccess.lock();
			Array2D<double> deriv;
//			Array2D<double> f(mNumStates,1);
//			f.inject(calcDriftVector(curState));
//			Array2D<double> g(mNumStates,mNumInputs);
//			g.inject(calcActuatorGainMatrix(curState));

			Array2D<double> f = calcDriftVector(curState);
			Array2D<double> g = calcActuatorGainMatrix(curState);

			deriv = f+matmult(g,curActuation);

			if(f.dim1() == 0 || f.dim2() == 0)
				Log::alert("calcStateDerivative: Drift vector failed to calculate");
			if(g.dim1() == 0 || g.dim2() == 0)
				Log::alert("calcStateDerivative: Actuator gain matrix failed to calculate");
			if(deriv.dim1() == 0 || deriv.dim2() == 0)
				Log::alert("calcStateDerivative: Derivative failed to calculate");

			mMutex_DataAccess.unlock();
			return deriv;
		};

	};
}

#endif
