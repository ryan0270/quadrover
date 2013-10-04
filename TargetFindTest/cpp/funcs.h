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

namespace ICSL {

vector<vector<cv::Point>> findContours(const cv::Mat &img);

}

#endif
