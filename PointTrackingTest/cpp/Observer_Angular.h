#ifndef ICSL_OBSERVER_ANGULAR_SIM
#define ICSL_OBSERVER_ANGULAR_SIM
#include <memory>
#include <list>

#include "TNT/tnt.h"

#include "Data.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;

class Observer_Angular
{
	public:
	list<shared_ptr<SO3Data<double>>> angleStateBuffer;

	SO3 estimateAttAtTime(const Time &t)
	{ return IData::interpolate(t, angleStateBuffer); }
};

}
}

#endif
