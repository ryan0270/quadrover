#ifndef ICSL_OBSERVER_TRANSLATIONAL_SIM
#define ICSL_OBSERVER_TRANSLATIONAL_SIM
#include <memory>
#include <list>

#include "TNT/tnt.h"

#include "Data.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;

class Observer_Translational
{
	public:
	list<shared_ptr<DataVector<double>>> tranStateBuffer;

	TNT::Array2D<double> estimateStateAtTime(const Time &t)
	{ return IData::interpolate(t, tranStateBuffer); }

	TNT::Array2D<double> estimateErrCovAtTime(const Time &t)
	{
		TNT::Array2D<double> errCov(9,9,0.0);
		errCov[0][0] = errCov[1][1] = 0.01*0.01;
		errCov[2][2] = 0.01*0.01;
		errCov[3][3] = errCov[4][4] = errCov[5][5] = 10*0.1*0.1;
		errCov[6][6] = errCov[7][7] = errCov[8][8] = 0.01*0.01;

		return errCov;
	}
};

}
}

#endif
