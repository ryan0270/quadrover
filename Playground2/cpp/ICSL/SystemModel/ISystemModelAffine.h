#ifndef CLASS_ISYSTEMMODELAFFINE
#define CLASS_ISYSTEMMODELAFFINE
#include <TNT/tnt.h>

#include "TNT_Utils.h"

#include "ISystemModel.h"

using namespace TNT;

namespace ICSL
{
	/*!
	 * Interface class for systems of the form
	 * \dot{x} = f(x) + g(x) u
	 */
	class ISystemModelAffine : public ISystemModel
	{
	public:
			explicit ISystemModelAffine(){};
			virtual ~ISystemModelAffine(){};

			virtual Array2D<double> calcDriftVector(Array2D<double> const &curState)=0;
			virtual Array2D<double> calcActuatorGainMatrix(Array2D<double> const &curState)=0;
			virtual Array2D<double> calcStateDerivative(Array2D<double> const &curState, Array2D<double> const &curActuation)
			{
				mMutex_DataAccess.lock();
				Array2D<double> deriv;
				Array2D<double> f = calcDriftVector(curState).copy();
				if(f.dim1() == 0 || f.dim2() == 0)
				{
					Log::alert(String("calcStateDerivative: Drift vector failed to calculate"));
					mMutex_DataAccess.unlock();
					return Array2D<double>(0,0);
				}

				Array2D<double> g = calcActuatorGainMatrix(curState).copy();
				if(g.dim1() == 0 || g.dim2() == 0)
				{
					Log::alert(String("calcStateDerivative: Actuator gain matrix failed to calculate"));
					mMutex_DataAccess.unlock();
					return Array2D<double>(0,0);
				}

				deriv = f+matmult(g,curActuation);
				if(deriv.dim1() == 0 || deriv.dim2() == 0)
				{
					Log::alert(String("calcStateDerivative: Derivative failed to calculat"));
					mMutex_DataAccess.unlock();
					return Array2D<double>(0,0);
				}

				mMutex_DataAccess.unlock();
				return deriv;
			};

	};
}

#endif
