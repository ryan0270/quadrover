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

template<class T> class Data;
template<class T> class DataVector; 
class IData
{
	public:
	virtual void lock(){mMutex.lock();}
	virtual void unlock(){mMutex.unlock();}
	Time timestamp;
	DataType type;

	template <class T1, class T2>
	static T1 interpolate(Time const &t, Data<T1> const &d1, Data<T2> const &d2);
	template <class T>
	static T interpolate(Time const &t, std::list<shared_ptr<Data<T> > > const &d);
	template <class T1, class T2>
	static TNT::Array2D<T1> interpolate(Time const &t, DataVector<T1> const &d1, DataVector<T2> const &d2);
	template <class T>
	static TNT::Array2D<T> interpolate(Time const &t, std::list<shared_ptr<DataVector<T> > > const &d);

	// ideally, the list would be passed in const here but then the returned iterator has to be const and I
	// couldn't do some things with it (like modify the list based on the returned iterator)
	template <class T>
	static typename std::list<std::shared_ptr<Data<T> > >::iterator findIndex(Time const &t, std::list<std::shared_ptr<Data<T> > > &d);
	template <class T>
	static typename std::list<std::shared_ptr<Data<T> > >::iterator findIndexReverse(Time const &t, std::list<std::shared_ptr<Data<T> > > &d);
	template <class T>
	static typename std::list<std::shared_ptr<DataVector<T> > >::iterator findIndex(Time const &t, std::list<std::shared_ptr<DataVector<T> > > &d);
	template <class T>
	static typename std::list<std::shared_ptr<DataVector<T> > >::iterator findIndexReverse(Time const &t, std::list<std::shared_ptr<DataVector<T> > > &d);

	template<class T>
	static typename std::list<std::shared_ptr<Data<T> > >::iterator truncate(Time const &t, std::list<std::shared_ptr<Data<T> > > &d);
	template <class T>
	static void truncate(Time const &t, std::list<std::shared_ptr<DataVector<T> > > &d);

	static bool timeSortPredicate(shared_ptr<IData> const &d1, shared_ptr<IData> const &d2){return d1->timestamp < d2->timestamp;}

	protected:
	toadlet::egg::Mutex mMutex;
};

template<class Tclass>
class Data : public IData
{
	public:
	Data(){type = DATA_TYPE_UNDEFINED;}
	Data(Tclass d, DataType t){data = d; type = t;}

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
	Tclass data, dataCalibrated;
};

template <class Tclass>
class DataVector : public IData
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
	TNT::Array2D<Tclass> data, dataCalibrated;
};

class DataImage : public IData
{
	public:
	DataImage() : att(3,1,0.0), angularVel(3,1,0.)  {
		type = DATA_TYPE_IMAGE; 
		imgFormat = IMG_FORMAT_BGR; 
		img = shared_ptr<cv::Mat>(new cv::Mat());
		cap = NULL;
		id = sNextID()++;
	}
//	DataImage(cv::Mat img1, TNT::Array2D<double> att1, TNT::Array2D<double> angularVel1, ImageFormat fmt){
//		img1.copyTo(*img);
//		att = att1.copy();
//		angularVel = angularVel.copy();
//		imgFormat = fmt;
//	}

	void copyTo(DataImage &d) {
		if(&d == this)
			return;
		lock();
		d.lock();
		d.timestamp.setTime(timestamp); 
		img->copyTo(*(d.img)); 
		d.att.inject(att);
		d.angularVel.inject(angularVel);
		d.focalLength = focalLength;
		d.unlock();
		unlock();
	}
	unsigned int id;
	shared_ptr<cv::Mat> img;
	TNT::Array2D<double> att;
	TNT::Array2D<double> angularVel;
	ImageFormat imgFormat;
	double focalLength;
	shared_ptr<cv::VideoCapture> cap;

	private:
	static inline unsigned int &sNextID(){ static unsigned int data = 0; return data;}
}; 

class DataAnnotatedImage : public IData
{
	public:
	DataAnnotatedImage(){ };

	shared_ptr<cv::Mat> imgAnnotated;
	shared_ptr<DataImage> imgDataSource;
};

template <class T>
class DataPhoneTemp : public Data<T>
{
	public:
	DataPhoneTemp() {Data<T>::type = DATA_TYPE_PHONE_TEMP; battTemp = secTemp = fgTemp = tmuTemp = -1;}

//	void copyTo(DataPhoneTemp<T> &d) const {
//		d.timestamp.setTime(timestamp); 
//		d.battTemp = battTemp;
//		d.secTemp = secTemp;
//		d.fgTemp = fgTemp;
//		d.tmuTemp = tmuTemp;
//	}
	T battTemp, secTemp, fgTemp, tmuTemp;
};

class ImageMatchData : public IData
{
	public:
	vector<vector<cv::Point2f> > featurePoints;
	shared_ptr<DataImage> imgData0, imgData1;
	shared_ptr<DataAnnotatedImage> imgAnnotated;
//	shared_ptr<cv::Mat> imgAnnotated;
//	double dt;

	void lock(){mMutex.lock(); if(imgData0 != NULL) imgData0->lock(); if(imgData1 != NULL) imgData1->lock();}
	void unlock(){mMutex.unlock(); if(imgData0 != NULL) imgData0->unlock(); if(imgData1 != NULL) imgData1->unlock();}

	protected:
	toadlet::egg::Mutex mMutex;
};

class ImageTargetFindData : public IData
{
	public:
	vector<BlobDetector::Blob> circleBlobs;
	shared_ptr<DataImage> imgData;

	void lock(){mMutex.lock(); if(imgData != NULL) imgData->lock();}
	void unlock(){mMutex.unlock(); if(imgData != NULL) imgData->unlock();}

	protected:
	toadlet::egg::Mutex mMutex;
};

//////////////////////////////////////////// Template implementations //////////////////////////////////////
// the list is assumed to be sorted in increasing time order
template <class T>
TNT::Array2D<T> IData::interpolate(Time const &t, list<shared_ptr<DataVector<T> > > const &d)
{
	if(d.size() == 0)
		return TNT::Array2D<T>();

	TNT::Array2D<T> interp;

	if(t <= d.front()->timestamp)
	{
		interp = d.front()->data.copy();
	}
	else if(t >= d.back()->timestamp)
	{
		interp = d.back()->data.copy();
	}
	else
	{
		shared_ptr<DataVector<T> > d1, d2;
		typename list<shared_ptr<DataVector<T> > >::const_iterator iter = d.begin();
		while(iter != d.end() && (*iter)->timestamp < t)
		{
			d1 = *iter;
			iter++;
		}
		if(iter != d.end())
		{
			d2 = *iter;
			double a = Time::calcDiffUS(d1->timestamp,t);
			double b = Time::calcDiffUS(t, d2->timestamp);
			interp= b/(a+b)*d1->data + a/(a+b)*d2->data;
		}
		else
		{
			interp = d1->data.copy();
		}
	}

	return interp;
}

template <class T1, class T2>
TNT::Array2D<T1> IData::interpolate(Time const &t, DataVector<T1> const &d1, DataVector<T2> const &d2)
{
	TNT::Array2D<double> interp;

	DataVector<T1> const *d1p, *d2p;
	if(d1.timestamp < d2.timestamp)
	{
		d1p = &d1;
		d2p = &d2;
	}
	else
	{
		d1p = &d2;
		d2p = &d1;
	}

	if(t < d1p->timestamp)
		interp= d1p->data.copy();
	else if(t > d2p->timestamp)
		interp= d2p->data.copy();
	else
	{
		double a = Time::calcDiffUS(d1p->timestamp,t);
		double b = Time::calcDiffUS(t, d2p->timestamp);
		interp= b/(a+b)*d1p->data+a/(a+b)*d2p->data;
	}

	return interp;
}

template <class T1, class T2>
T1 IData::interpolate(Time const &t, Data<T1> const &d1, Data<T2> const &d2)
{
	T1 interp;

	Data<T1> const *d1p;
	Data<T2> const *d2p;
	if(d1.timestamp < d2.timestamp)
	{
		d1p = &d1;
		d2p = &d2;
	}
	else
	{
		d1p = &d2;
		d2p = &d1;
	}

	if(t < d1p->timestamp)
		interp = d1p->data;
	else if(t > d2p->timestamp)
		interp = d2p->data;
	else
	{
		T1 a = Time::calcDiffUS(d1p->timestamp,t);
		T1 b = Time::calcDiffUS(t, d2p->timestamp);
		interp= b/(a+b)*d1p->data+a/(a+b)*d2p->data;
	}

	return interp;
}

template <class T>
T IData::interpolate(Time const &t, std::list<shared_ptr<Data<T> > > const &d)
{
	if(d.size() == 0)
		return 0;
	T interp;

	if(t < d.front()->timestamp)
		interp = d.front()->data;
	else if(t > d.back()->timestamp)
		interp = d.back()->data;
	else
	{
		shared_ptr<Data<T> > d1, d2;
		typename list<shared_ptr<Data<T> > >::const_iterator iter = d.begin();
		while(iter != d.end() && (*iter)->timestamp < t)
		{
			d1 = *iter;
			iter++;
		}
		if(iter != d.end())
		{
			d2 = *iter;
			T a = Time::calcDiffUS(d1->timestamp,t);
			T b = Time::calcDiffUS(t, d2->timestamp);
			interp= b/(a+b)*d1->data+a/(a+b)*d2->data;
		}
		else
			interp = d1->data;
	}

	return interp;
}

template <class T>
typename list<shared_ptr<DataVector<T> > >::iterator IData::findIndex(Time const &t, list<shared_ptr<DataVector<T> > > &d)
{
	typename list<shared_ptr<DataVector<T> > >::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

template <class T>
typename list<shared_ptr<DataVector<T> > >::iterator IData::findIndexReverse(Time const &t, list<shared_ptr<DataVector<T> > > &d)
{
	typename list<shared_ptr<DataVector<T> > >::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp > t)
		i--;
	return i;
}

template <class T>
void IData::truncate(Time const &t, list<shared_ptr<DataVector<T> > > &d)
{
	if(d.size() == 0)
	{
		return;
	}
	if(t > d.back()->timestamp)
	{
		return;
	}
	if(t < d.front()->timestamp)
	{
		d.clear();
		return;
	}

	typename list<shared_ptr<DataVector<T> > >::iterator iter = d.end();
	iter--;
	while(t < (*iter)->timestamp && iter != d.begin())
	{
		iter--;
		d.pop_back();
	}
}

// returns first index greater than or equal to t, starting from the front
template <class T>
typename list<shared_ptr<Data<T> > >::iterator IData::findIndex(Time const &t, list<shared_ptr<Data<T> > > &d)
{
	typename list<shared_ptr<Data<T> > >::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

// returns first index less than or equal to t, starting from the back
// If t < d.begin(), d.begin() is still returned
template <class T>
typename list<shared_ptr<Data<T> > >::iterator IData::findIndexReverse(Time const &t, list<shared_ptr<Data<T> > > &d)
{
	typename list<shared_ptr<Data<T> > >::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp > t)
		i--;
	return i;
}

template <class T>
typename list<shared_ptr<Data<T> > >::iterator IData::truncate(Time const &t, list<shared_ptr<Data<T> > > &d)
{
	if(d.size() == 0)
		return d.end();
	if(t > d.back()->timestamp)
	{
		typename list<shared_ptr<Data<T> > >::iterator it = d.end();
		it--;
		return it;
	}
	if(t < d.front()->timestamp)
	{
		d.clear();
		return d.end();
	}

	typename list<shared_ptr<Data<T> > >::iterator iter = d.end();
	iter--;
	while(t < (*iter)->timestamp && iter != d.begin())
	{
		iter--;
		d.pop_back();
	}

	return iter;
}



} // namespace Quadrotor
} // namespace ICSL

#endif
