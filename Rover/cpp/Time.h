#ifndef ICSL_TIME_H
#define ICSL_TIME_H

#include <time.h>
#include <toadlet/egg.h>

namespace ICSL{
namespace Quadrotor{
//using namespace std;

class Time
{
	public:
		explicit Time(){setTime();}
		virtual ~Time(){};

		void clear(){mTime.tv_sec = 0; mTime.tv_nsec = 0;}
		void setTime(){clock_gettime(CLOCK_MONOTONIC,&mTime);};
		void setTime(const Time &t){mTime.tv_sec = t.mTime.tv_sec; mTime.tv_nsec = t.mTime.tv_nsec;}
		void setTimeMS(toadlet::uint64 t){mTime.tv_sec = t/1e3; mTime.tv_nsec = (t%(toadlet::uint64)1e3)*1e6;}
		void setTimeUS(toadlet::uint64 t){mTime.tv_sec = t/1e6; mTime.tv_nsec = (t%(toadlet::uint64)1e6)*1e3;}
		void setTimeNS(toadlet::uint64 t){mTime.tv_sec = t/1e9; mTime.tv_nsec = (t%(toadlet::uint64)1e9);}
		
		toadlet::uint64 getNS() const {return mTime.tv_sec*1e9+mTime.tv_nsec;}
		toadlet::uint64 getUS() const {return mTime.tv_sec*1e6+mTime.tv_nsec/1.0e3;}
		toadlet::uint64 getMS() const {return mTime.tv_sec*1e3+mTime.tv_nsec/1.0e6;}

		toadlet::uint64 getElapsedTimeNS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e9+(now.mTime.tv_nsec-mTime.tv_nsec); }
		toadlet::uint64 getElapsedTimeUS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e6+(now.mTime.tv_nsec-mTime.tv_nsec)/1.0e3; }
		toadlet::uint64 getElapsedTimeMS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e3+(now.mTime.tv_nsec-mTime.tv_nsec)/1.0e6; }

		static toadlet::uint64 nowMS(){Time now; return now.getMS();}
		static toadlet::uint64 nowUS(){Time now; return now.getUS();}
		static toadlet::uint64 nowNS(){Time now; return now.getNS();}

		static toadlet::uint64 calcDiffNS(const Time &tm1, const Time &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e9+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec); }
		static toadlet::uint64 calcDiffUS(const Time &tm1, const Time &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e6+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec)/1.0e3; }
		static toadlet::uint64 calcDiffMS(const Time &tm1, const Time &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e3+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec)/1.0e6; }

		bool operator > (const Time &t) const
		{
			if(mTime.tv_sec > t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec > t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		bool operator >= (const Time &t) const
		{
			if(mTime.tv_sec > t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec >= t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		bool operator < (const Time &t) const
		{
			if(mTime.tv_sec < t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec < t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		bool operator <= (const Time &t) const
		{
			if(mTime.tv_sec < t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec <= t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		bool operator == (const Time &t) const {return mTime.tv_nsec == t.mTime.tv_nsec && mTime.tv_sec == t.mTime.tv_sec;}

	protected:
		timespec mTime;
};

}
}

#endif


