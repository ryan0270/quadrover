#include <memory>
#include <fstream>
#include <sstream>
#include <iostream>
#include <list>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "Time.h"
#include "FeatureFinder.h"
#include "TrackedObject.h"
#include "ObjectTracker.h"
#include "QuadLogger.h"
#include "Data.h"
#include "Rotation.h"
#include "Observer_Translational.h"
#include "Observer_Angular.h"
#include "Listeners.h"


class MyListener : public ICSL::Quadrotor::ObjectTrackerListener
{
	public:
	std::shared_ptr<ICSL::Quadrotor::ObjectTrackerData> data;

	void onObjectsTracked(const std::shared_ptr<ICSL::Quadrotor::ObjectTrackerData> &data)
	{ this->data = data; }
};

void loadData(const std::string &dataDir, const std::string &imgDir,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataImage>> &imageDataBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataVector<double>>> &tranStateBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::SO3Data<double>>> &attBuffer);

int main(int argv, char* argc[])
{
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
	using namespace TNT;
	using namespace std;
	using toadlet::egg::Log;
	using toadlet::egg::String;
	cout << "start chadding" << endl;

	string dataDir;
	int dataSet = 0;
	int startImg=0, endImg=0;
	switch(dataSet)
	{
		case 0:
			dataDir = "../dataSets/Nov13_3";
			break;
	}

	string imgDir;
	imgDir = dataDir + "/video";

	// Load the log file
	list<shared_ptr<DataVector<double>>> tranStateBuffer;
	list<shared_ptr<SO3Data<double>>> attBuffer;
	list<shared_ptr<DataImage>> imgDataBuffer;
	loadData(dataDir, dataDir+"/video", imgDataBuffer, tranStateBuffer, attBuffer);

	///////////////////////////////////////////////////////////////////////////

	cv::namedWindow("chad",1);
	cv::moveWindow("chad",0,0);
	cv::namedWindow("bob",1);
	cv::moveWindow("bob",321,0);
	cv::namedWindow("tom",1);
	cv::moveWindow("tom",0,261);

	Observer_Translational obsvTranslation;
	obsvTranslation.tranStateBuffer = tranStateBuffer;

	Observer_Angular obsvAngular;
	obsvAngular.angleStateBuffer = attBuffer;

	ObjectTracker objectTracker;
	objectTracker.initialize();
	objectTracker.start();
	objectTracker.setObserverTranslation(&obsvTranslation);
	objectTracker.setObserverAngular(&obsvAngular);

	MyListener myListener;
	myListener.data = NULL;
	objectTracker.addListener(&myListener);

	list<shared_ptr<DataImage>>::const_iterator imgIter = imgDataBuffer.begin();
	// Skip the first several images
	for(int i=0; i<200; i++)
		imgIter++;

	int keypress = 0;
	cv::Mat img(240,320, CV_8UC3, cv::Scalar(0)), oldImg(240,320,CV_8UC3,cv::Scalar(0)), imgGray;
	int activeCnt = 0;
	int imgCnt = 0;
	Time curTime;
	vector<cv::Point2f> points;
	float qualityLevel = 0.01;
	float fastThresh = 20;
	int sepDist = 20;
	float pointCntTarget = 30;
	float fastAdaptRate = 0.05;
	shared_ptr<DataImage> curImgData, prevImgData;
	curImgData = *imgIter;
	imgIter++;
	double longestLife = 0;
	double avgOldestLife = 0;
	float avgOldestLifeCnt = 0;
	while(keypress != (int)'q' && imgIter != imgDataBuffer.end())
	{
		prevImgData = curImgData;
		curImgData = *imgIter;
		curTime.setTime(curImgData->timestamp);
		curImgData->image->copyTo(img);
		curImgData->imageGray->copyTo(imgGray);
		double f = curImgData->focalLength;
		cv::Point2f center = curImgData->center;
		double dt = Time::calcDiffNS(prevImgData->timestamp, curImgData->timestamp)/1.0e9;


		myListener.data = NULL;
Time start;
		points = FeatureFinder::findFeaturePoints(imgGray, qualityLevel, sepDist, fastThresh);
		if(points.size() > 0)
		{
			cv::undistortPoints(points, points, *curImgData->cameraMatrix, *curImgData->distCoeffs);
			for(int i=0; i<points.size(); i++)
				points[i] = points[i]*f + center;
		}
		fastThresh += min(1.0f, max(-1.0f, fastAdaptRate*((float)points.size()-pointCntTarget)));
		fastThresh = max(5.0f, fastThresh);

		// make data and pass to the object tracker
		shared_ptr<ImageFeatureData> data(new ImageFeatureData());
		data->featurePoints= points;
		data->imageData = curImgData;
		data->timestamp.setTime(curImgData->timestamp);
		objectTracker.onFeaturesFound(data);

		while(myListener.data == NULL)
			System::msleep(1);

		const vector<shared_ptr<TrackedObject>> trackedObjects = objectTracker.getTrackedObjects();
		vector<shared_ptr<TrackedObject>> repeatObjects, newObjects;
		repeatObjects = myListener.data->trackedObjects;
		newObjects = myListener.data->newObjects;
		
		activeCnt += trackedObjects.size();

		imshow("chad",oldImg);

		cv::Mat dblImg(img.rows, 2*img.cols, img.type());
		oldImg.copyTo(dblImg(cv::Rect(0,0,oldImg.cols,oldImg.rows)));

		for(int i=0; i<points.size(); i++)
			circle(img, points[i], 4, cv::Scalar(255,0,0), -1);

		img.copyTo(oldImg);
		imshow("bob",img);

		vector<vector<cv::Point2f>> repeatPts(repeatObjects.size());
		for(int i=0; i<repeatObjects.size(); i++)
			circle(img, repeatObjects[i]->getLocation(), 4, cv::Scalar(0,0,255), -1);

//		stringstream name;
//		name << imgDir << "/annotated/img_" << imgCnt << ".bmp";
//		imwrite(name.str().c_str(),img);

		img.copyTo(dblImg(cv::Rect(oldImg.cols,0,img.cols,img.rows)));
		cv::Point2f offset(321,0);
		for(int i=0; i<repeatObjects.size(); i++)
		{
			const vector<pair<Time, cv::Point2f>> history = repeatObjects[i]->getHistory();
			vector<pair<Time, cv::Point2f>>::const_iterator iter = history.end();
			iter--;
			cv::Point2f p2 = iter->second;
			iter--;
			cv::Point2f p1 = iter->second;

			line(dblImg,p1, p2+offset, cv::Scalar(0,255,0), 2);
		}
		imshow("tom",dblImg);

		keypress = cv::waitKey(1) % 256;

		imgIter++;
		imgCnt++;

		Time oldTime = objectTracker.getOldest()->getCreateTime();
		double curLongLife = Time::calcDiffNS(oldTime, curTime)/1.0e9;
		avgOldestLife += curLongLife;
		avgOldestLifeCnt++;
		if(curLongLife > longestLife)
		{
			longestLife = curLongLife;
//			cout << "New record: " << longestLife << endl;
		}
		if(imgCnt % 100 == 0)
		{
			cout << "Tracked points: " << trackedObjects.size() << "\t\t";
			cout << "Oldest life time: " << curLongLife << endl;
		}
	}

	cout << "Avg oldest life: " << avgOldestLife/avgOldestLifeCnt << endl;

	objectTracker.shutdown();

//	for(int i=0; i<TimeKeeper::times.size(); i++)
//		if(TimeKeeper::times[i] != 0)
//			Log::alert(String()+"t" + i +":\t" + (TimeKeeper::times[i]/imgCnt));
//	Log::alert(String()+"avg algo time: " + TimeKeeper::sum(0,4)/imgCnt);
//	Log::alert(String()+"num regions:\t" + activeCnt/imgCnt);

    return 0;
}

void loadData(const std::string &dataDir, const std::string &imgDir,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataImage>> &imgDataBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::DataVector<double>>> &tranStateBuffer,
			std::list<std::shared_ptr<ICSL::Quadrotor::SO3Data<double>>> &attBuffer)
{
	using namespace std;
	using namespace ICSL;
	using namespace ICSL::Quadrotor;
	using namespace TNT;

	imgDataBuffer.clear();
	tranStateBuffer.clear();
	attBuffer.clear();

	// Camera calibration
	cv::Point2f center;
	shared_ptr<cv::Mat> mCameraMatrix_640x480, mCameraMatrix_320x240, mCameraDistortionCoeffs;
	cv::FileStorage fs;
	string calibFilename = dataDir + "/calib_640x480.yml";
	fs.open(calibFilename .c_str(), cv::FileStorage::READ);
	if( fs.isOpened() )
	{
		mCameraMatrix_640x480 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraDistortionCoeffs = shared_ptr<cv::Mat>(new cv::Mat());

		fs["camera_matrix"] >> *mCameraMatrix_640x480;
		fs["distortion_coefficients"] >> *mCameraDistortionCoeffs;
		cout << "Camera calib loaded from " << calibFilename.c_str() << endl;

		mCameraMatrix_320x240 = shared_ptr<cv::Mat>(new cv::Mat());
		mCameraMatrix_640x480->copyTo( *mCameraMatrix_320x240 );
		(*mCameraMatrix_320x240) = (*mCameraMatrix_320x240)*0.5;

		cout << "Loaded camera matrix" << endl;
	}
	else
		cout << "Failed to open " <<  calibFilename.c_str();
	fs.release();

	string line;
	string dataFilename = dataDir+"/phoneLog.txt";
	ifstream file(dataFilename.c_str());
	if(file.is_open())
	{
		getline(file, line); // first line is a throw-away
		getline(file, line); // second line is also a throw-away

		
		Array2D<double> tranState(9,1);
		double w, x, y, z; // for quaternions
		while(file.good())
		{
			getline(file, line);
			stringstream ss(line);
			double time;
			int type;
			ss >> time >> type;

			switch(type)
			{
				case LOG_ID_IMAGE:
					{
						int id;
						ss >> time >> id;

						shared_ptr<cv::Mat> img(new cv::Mat());
						shared_ptr<cv::Mat> imgGray(new cv::Mat());

						stringstream ss2;
						ss2 << "image_" << id << ".bmp";
						string imgFilename = imgDir+"/"+ss2.str();
						*img = cv::imread(imgFilename);
						if(img->data != NULL)
						{
							shared_ptr<DataImage> data(new DataImage());
							data->timestamp.setTimeMS(time);
							data->imageId = id;
							data->image = img;
							cv::cvtColor(*img, *imgGray, CV_BGR2GRAY); 
							data->imageGray = imgGray;
							data->imageFormat = IMG_FORMAT_BGR;
							data->cap = NULL;
							if(img->rows = 240)
								data->cameraMatrix = mCameraMatrix_320x240;
							else
								data->cameraMatrix = mCameraMatrix_640x480;
							data->focalLength = data->cameraMatrix->at<double>(0,0);
							data->center.x = data->cameraMatrix->at<double>(0,2);
							data->center.y = data->cameraMatrix->at<double>(1,2);
							data->distCoeffs = mCameraDistortionCoeffs;

							imgDataBuffer.push_back(data);
						}
					}
					break;
				case LOG_ID_CUR_TRANS_STATE:
					{
						for(int i=0; i<9; i++)
							ss >> tranState[i][0];
						shared_ptr<DataVector<double>> data(new DataVector<double>);
						data->timestamp.setTimeMS(time);
						data->data = tranState.copy();
						tranStateBuffer.push_back(data);
					}
					break;
				case LOG_ID_CUR_ATT:
					{
						shared_ptr<SO3Data<double>> data(new SO3Data<double>);
//						ss >> time >> w >> x >> y >> z;
//						SO3 rot(Quaternion(w,x,y,z));
						ss >> time >> x >> y >> z;
						SO3 rot(createRotMat_ZYX(z,y,x));
						data->timestamp.setTimeMS(time);
						data->rotation = rot;
						attBuffer.push_back(data);
					}
					break;
			}
		}

		file.close();
	}
	else
		cout << "Failed to open file: " << dataFilename << endl;

	cout << "Loaded " << tranStateBuffer.size()+attBuffer.size() << " lines" << endl;
}
