#ifndef ICSL_DATA_H
#define ICSL_DATA_H
#include <memory>
#include <list>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "TNT/tnt.h"
#include "Common.h"
#include "Time.h"
#include "TNT_Utils.h"
#include "BlobDetector.h"

namespace ICSL {
namespace Quadrotor {
enum ImageFormat
{
	IMG_FORMAT_BGR=1,
	IMG_FORMAT_RGB,
	IMG_FORMAT_HSV,
	IMG_FORMAT_GRAY,
};

enum DataType
{
	DATA_TYPE_UNDEFINED=0,
	DATA_TYPE_GYRO,
	DATA_TYPE_ACCEL,
	DATA_TYPE_MAG,
	DATA_TYPE_PRESSURE,
	DATA_TYPE_IMAGE,
	DATA_TYPE_PHONE_TEMP,
	DATA_TYPE_STATE_TRAN,
	DATA_TYPE_NET_ACCEL_TRAN,
	DATA_TYPE_KF_ERR_COV,
	DATA_TYPE_VICON_POS,
	DATA_TYPE_VICON_VEL,
	DATA_TYPE_VICON_HEIGHT,
	DATA_TYPE_CAMERA_POS,
	DATA_TYPE_CAMERA_VEL,
	DATA_TYPE_OPTIC_FLOW_VEL,
	DATA_TYPE_MOTOR_CMDS,
	DATA_TYPE_THRUST,
	DATA_TYPE_THRUST_DIR,
	DATA_TYPE_ATTITUDE,
	DATA_TYPE_ANGULAR_VEL,
};

class DataVector; 
class Data
{
	public:
	Data(){type = DATA_TYPE_UNDEFINED;}
	Data(double d, DataType t){data = d; type = t;}

	virtual void lock(){mMutex.lock();}
	virtual void unlock(){mMutex.unlock();}

	virtual void copyTo(Data &d) {
		if(&d == this)
			return;
		lock();
		d.lock();
		d.timestamp.setTime(timestamp); 
		d.type = type;
		d.data = data; 
		d.dataCalibrated = dataCalibrated;
		d.unlock();
		unlock();
	}
	double data, dataCalibrated;
	Time timestamp;
	DataType type;

	static double interpolate(Time const &t, Data const &d1, Data const &d2);
	static TNT::Array2D<double> interpolate(Time const &t, DataVector const &d1, DataVector const &d2);
	static double interpolate(Time const &t, std::list<shared_ptr<Data> > const &d);
	static TNT::Array2D<double> interpolate(Time const &t, std::list<shared_ptr<DataVector> > const &d);

	// ideally, the list would be passed in const here but then the returned iterator has to be const and I
	// couldn't do some things with it (like modify the list based on the returned iterator)
	static std::list<std::shared_ptr<Data> >::iterator findIndex(Time const &t, std::list<std::shared_ptr<Data> > &d);
	static std::list<std::shared_ptr<Data> >::iterator findIndexReverse(Time const &t, std::list<std::shared_ptr<Data> > &d);
	static std::list<std::shared_ptr<DataVector> >::iterator findIndex(Time const &t, std::list<std::shared_ptr<DataVector> > &d);
	static std::list<std::shared_ptr<DataVector> >::iterator findIndexReverse(Time const &t, std::list<std::shared_ptr<DataVector> > &d);

	static std::list<std::shared_ptr<Data> >::iterator truncate(Time const &t, std::list<std::shared_ptr<Data> > &d);
	static void truncate(Time const &t, std::list<std::shared_ptr<DataVector> > &d);

	protected:
	toadlet::egg::Mutex mMutex;
};

class DataVector : public Data
{
	public:
	DataVector(){type = DATA_TYPE_UNDEFINED;};
//	DataVector(TNT::Array2D<double> const &d, DataType t){data = d.copy(); type = t;}

	void copyTo(DataVector &d) {
		if(&d == this)
			return;
		lock();
		d.lock();
		d.timestamp.setTime(timestamp); 
		d.data = data.copy(); 
		d.dataCalibrated = dataCalibrated.copy();
		d.type = type;
		d.unlock();
		unlock();
	}
	TNT::Array2D<double> data, dataCalibrated;
};

class DataImage : public Data
{
	public:
	DataImage() : att(3,1,0.0), startAngularVel(3,1,0.0), endAngularVel(3,1,0.)  {
		type = DATA_TYPE_IMAGE; 
		imgFormat = IMG_FORMAT_BGR; 
		img = shared_ptr<cv::Mat>(new cv::Mat());
		cap = NULL;
	}
	DataImage(cv::Mat img1, TNT::Array2D<double> att1, TNT::Array2D<double> angularVel1, ImageFormat fmt){
		img1.copyTo(*img);
		att = att1.copy();
		startAngularVel = startAngularVel.copy();
		endAngularVel = endAngularVel.copy();
		imgFormat = fmt;
	}

	void copyTo(DataImage &d) {
		if(&d == this)
			return;
		lock();
		d.lock();
		d.timestamp.setTime(timestamp); 
		img->copyTo(*(d.img)); 
		d.att.inject(att);
		d.startAngularVel.inject(startAngularVel);
		d.endAngularVel.inject(endAngularVel);
		d.focalLength = focalLength;
		d.unlock();
		unlock();
	}
	shared_ptr<cv::Mat> img;
	TNT::Array2D<double> att;
	TNT::Array2D<double> startAngularVel, endAngularVel;
	ImageFormat imgFormat;
	double focalLength;
	shared_ptr<cv::VideoCapture> cap;
};

class DataPhoneTemp : public Data
{
	public:
	DataPhoneTemp(){type = DATA_TYPE_PHONE_TEMP; battTemp = secTemp = fgTemp = tmuTemp = -1;}

	void copyTo(DataPhoneTemp &d) const {
		d.timestamp.setTime(timestamp); 
		d.battTemp = battTemp;
		d.secTemp = secTemp;
		d.fgTemp = fgTemp;
		d.tmuTemp = tmuTemp;
	}
	float battTemp, secTemp, fgTemp, tmuTemp;
};

class ImageMatchData
{
	public:
	vector<vector<cv::Point2f> > featurePoints;
	shared_ptr<DataImage> imgData0, imgData1;
	double dt;

	void lock(){mMutex.lock(); if(imgData0 != NULL) imgData0->lock(); if(imgData1 != NULL) imgData1->lock();}
	void unlock(){mMutex.unlock(); if(imgData0 != NULL) imgData0->unlock(); if(imgData1 != NULL) imgData1->unlock();}

	shared_ptr<cv::Mat> imgAnnotated;

	protected:
	toadlet::egg::Mutex mMutex;
};

class ImageTargetFindData
{
	public:
	vector<BlobDetector::Blob> circleBlobs;
	shared_ptr<DataImage> imgData;

	void lock(){mMutex.lock(); if(imgData != NULL) imgData->lock();}
	void unlock(){mMutex.unlock(); if(imgData != NULL) imgData->unlock();}

	protected:
	toadlet::egg::Mutex mMutex;
};


} // namespace Quadrotor
} // namespace ICSL

#endif
