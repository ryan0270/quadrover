#ifndef ICSL_TIME_H
#define ICSL_TIME_H

#include <time.h>
#include <toadlet/egg.h>

using namespace std;
using toadlet::uint64;

namespace ICSL{
namespace Quadrotor{

class Time
{
	public:
		explicit Time(){setTime();}
		virtual ~Time(){};

		void clear(){mTime.tv_sec = 0; mTime.tv_nsec = 0;}
		void setTime(){clock_gettime(CLOCK_MONOTONIC,&mTime);};
		void setTime(Time const &t){mTime.tv_sec = t.mTime.tv_sec; mTime.tv_nsec = t.mTime.tv_nsec;}
		void setTimeMS(uint64 t){mTime.tv_sec = t/1e3; mTime.tv_nsec = (t%(uint64)1e3)*1e6;}
		void setTimeUS(uint64 t){mTime.tv_sec = t/1e6; mTime.tv_nsec = (t%(uint64)1e6)*1e3;}
		void setTimeNS(uint64 t){mTime.tv_sec = t/1e9; mTime.tv_nsec = (t%(uint64)1e9);}
		
		uint64 getNS() const {return mTime.tv_sec*1e9+mTime.tv_nsec;}
		uint64 getUS() const {return mTime.tv_sec*1e6+mTime.tv_nsec/1.0e3;}
		uint64 getMS() const {return mTime.tv_sec*1e3+mTime.tv_nsec/1.0e6;}

		uint64 getElapsedTimeNS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e9+(now.mTime.tv_nsec-mTime.tv_nsec); }
		uint64 getElapsedTimeUS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e6+(now.mTime.tv_nsec-mTime.tv_nsec)/1.0e3; }
		uint64 getElapsedTimeMS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e3+(now.mTime.tv_nsec-mTime.tv_nsec)/1.0e6; }

		static uint64 nowMS(){Time now; return now.getMS();}
		static uint64 nowUS(){Time now; return now.getUS();}
		static uint64 nowNS(){Time now; return now.getNS();}

		static uint64 calcDiffNS(Time const &tm1,Time const &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e9+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec); }
		static uint64 calcDiffUS(Time const &tm1,Time const &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e6+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec)/1.0e3; }
		static uint64 calcDiffMS(Time const &tm1,Time const &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e3+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec)/1.0e6; }

	protected:
		timespec mTime;
};

}
}

#endif


