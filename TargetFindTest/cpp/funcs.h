#ifndef ICSL_TARGETTEST_FUNCS
#define ICSL_TARGETTEST_FUNCS
#include <vector>
#include <memory>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/features2d/features2d.hpp>

#include "TNT/tnt.h"
#include "TNT_Utils.h"
#include "constants.h"

#include "Time.h"
#include "ActiveObject.h"
//#include "mser.h"

namespace ICSL {

class TimeKeeper
{
	public:
	static vector<double> times;

	static double sum(int idx1, int idx2)
	{
		double sum = 0;
		for(int i=idx1; i<=idx2; i++)
			sum += times[i];
		return sum;
	}
};

vector<vector<cv::Point>> findContours(const cv::Mat &img);

vector<shared_ptr<ActiveObject>> objectify(const vector<vector<cv::Point>> &contours,
										   const TNT::Array2D<double> Sn,
										   const TNT::Array2D<double> SnInv,
										   double varxi, double probNoCorr,
										   const Time &curTime);

void matchify(vector<shared_ptr<ActiveObject>> &activeObjects,
			  const vector<shared_ptr<ActiveObject>> &curObjects,
			  vector<Match> &goodMatches,
			  vector<shared_ptr<ActiveObject>> &repeatObjects,
			  const TNT::Array2D<double> Sn,
			  const TNT::Array2D<double> SnInv,
			  double varxi, double probNoCorr,
			  const Time &curTime);
}

#endif
