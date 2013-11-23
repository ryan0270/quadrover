#ifndef ICSL_TIME_H
#define ICSL_TIME_H

#include <time.h>
//#include <toadlet/egg.h>

namespace ICSL{
class Time
{
	public:
		Time(){setTime();}
		virtual ~Time(){};

		inline void clear(){mTime.tv_sec = 0; mTime.tv_nsec = 0;}

		inline void setTime(){clock_gettime(CLOCK_MONOTONIC,&mTime);};
		inline void setTime(const Time &t){mTime.tv_sec = t.mTime.tv_sec; mTime.tv_nsec = t.mTime.tv_nsec;}
		inline void setTimeMS(int64_t t){mTime.tv_sec = t/1e3; mTime.tv_nsec = (t%(int64_t)1e3)*1e6;}
		inline void setTimeUS(int64_t t){mTime.tv_sec = t/1e6; mTime.tv_nsec = (t%(int64_t)1e6)*1e3;}
		inline void setTimeNS(int64_t t){mTime.tv_sec = t/1e9; mTime.tv_nsec = (t%(int64_t)1e9);}

		inline void addTimeMS(int64_t ms)
		{ addTimeNS(ms*1.e6); }

		inline void addTimeUS(int64_t us)
		{ addTimeNS(us*1.e3); }

		inline void addTimeNS(int64_t ns)
		{ mTime.tv_nsec += ns; } // It seems that this already properly handles overflow
		
		inline int64_t getNS() const {return (int64_t)mTime.tv_sec*1e9+mTime.tv_nsec;}
		inline int64_t getUS() const {return (int64_t)mTime.tv_sec*1e6+mTime.tv_nsec/1.0e3;}
		inline int64_t getMS() const {return (int64_t)mTime.tv_sec*1e3+mTime.tv_nsec/1.0e6;}

		inline int64_t getElapsedTimeNS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e9+(now.mTime.tv_nsec-mTime.tv_nsec); }
		inline int64_t getElapsedTimeUS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e6+(now.mTime.tv_nsec-mTime.tv_nsec)/1.0e3; }
		inline int64_t getElapsedTimeMS() const {Time now; return (now.mTime.tv_sec-mTime.tv_sec)*1e3+(now.mTime.tv_nsec-mTime.tv_nsec)/1.0e6; }

		inline static int64_t nowMS(){Time now; return now.getMS();}
		inline static int64_t nowUS(){Time now; return now.getUS();}
		inline static int64_t nowNS(){Time now; return now.getNS();}

		inline static int64_t calcDiffNS(const Time &tm1, const Time &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e9+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec); }
		inline static int64_t calcDiffUS(const Time &tm1, const Time &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e6+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec)/1.0e3; }
		inline static int64_t calcDiffMS(const Time &tm1, const Time &tm2) { return (tm2.mTime.tv_sec-tm1.mTime.tv_sec)*1e3+(tm2.mTime.tv_nsec-tm1.mTime.tv_nsec)/1.0e6; }

		inline bool operator > (const Time &t) const
		{
			if(mTime.tv_sec > t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec > t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		inline bool operator >= (const Time &t) const
		{
			if(mTime.tv_sec > t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec >= t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		inline bool operator < (const Time &t) const
		{
			if(mTime.tv_sec < t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec < t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		inline bool operator <= (const Time &t) const
		{
			if(mTime.tv_sec < t.mTime.tv_sec)
				return true;
			else if(mTime.tv_sec == t.mTime.tv_sec && mTime.tv_nsec <= t.mTime.tv_nsec)
				return true;
			else
				return false;
		}

		inline bool operator == (const Time &t) const {return mTime.tv_nsec == t.mTime.tv_nsec && mTime.tv_sec == t.mTime.tv_sec;}

	protected:
		timespec mTime;
};

}

#endif


