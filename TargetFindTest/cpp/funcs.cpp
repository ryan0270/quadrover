#include "funcs.h"

namespace ICSL {
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

vector<vector<cv::Point>> findContours(const cv::Mat &img)
{
	cv::Mat imgGray;
	cvtColor(img, imgGray, CV_BGR2GRAY);

	int delta = 5*2;
	int minArea = 1000;
	int maxArea = 0.5*240*320;
	double maxVariation = 0.25; // smaller reduces number of regions
	double minDiversity = 0.4; // smaller increase the number of regions
	cv::MSER mserDetector(delta, minArea, maxArea, maxVariation, minDiversity);
	vector<vector<cv::Point>> regions;
	mserDetector(imgGray, regions);

	// preallocate
	cv::Mat mask(img.rows,img.cols,CV_8UC1, cv::Scalar(0));

	/////////////////// Find contours ///////////////////////
	vector<vector<cv::Point>> allContours;
	int border = 2;
	for(int i=0; i<regions.size(); i++)
	{
		cv::Rect boundRect = boundingRect( cv::Mat(regions[i]) );
		boundRect.x = max(0, boundRect.x-border);
		boundRect.y = max(0, boundRect.y-border);
		boundRect.width = min(img.cols, boundRect.x+boundRect.width+2*border)-boundRect.x;
		boundRect.height= min(img.rows, boundRect.y+boundRect.height+2*border)-boundRect.y;;
		cv::Point corner(boundRect.x, boundRect.y);
		mask(boundRect) = cv::Scalar(0);
		uchar *row;
		int step = mask.step;
		for(int j=0; j<regions[i].size(); j++)
		{
			int x = regions[i][j].x;
			int y = regions[i][j].y;
			mask.at<uchar>(y,x) = 255;
		}

		cv::Canny(mask(boundRect), mask(boundRect), 50, 100, 5, true);

		vector<vector<cv::Point>> contours;
		cv::findContours(mask(boundRect), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, corner);

		for(int i=0; i<contours.size(); i++)
		{
			if(cv::contourArea(contours[i]) < minArea)
				continue;
			allContours.push_back(contours[i]);
		}
	}

	return allContours;
}

}
