#ifndef ICSL_DATA_H
#define ICSL_DATA_H
#include <memory>
#include <mutex>
#include <list>

#include "TNT/tnt.h"
#include "Common.h"
#include "Time.h"
#include "TNT_Utils.h"
#include "constants.h"
#include "Rotation.h"
#include "ActiveRegion.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace ICSL {
namespace Quadrotor {
using namespace ICSL::Constants;
using namespace std;
using namespace toadlet::egg;

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
	DATA_TYPE_MAP_VEL,
	DATA_TYPE_MAP_HEIGHT,
	DATA_TYPE_GRAVITY_DIR,
	DATA_TYPE_RAW_ACCEL,
	DATA_TYPE_SO3,
	DATA_TYPE_HEIGHT,
	DATA_TYPE_TARGET_FIND,
};

template<class T> class Data;
template<class T> class DataVector; 
template<class T> class SO3Data;
template<class T> class HeightData;
class IData
{
	public:
	IData(){ dataID = sNextDataID()++; }
	
	virtual void lock(){mMutex.lock();}
	virtual void unlock(){mMutex.unlock();}
	Time timestamp;
	DataType type;
	unsigned long dataID;

	template <class T1, class T2>
	static T1 interpolate(const Time &t, const Data<T1> &d1, const Data<T2> &d2);
	template <class T>
	static T interpolate(const Time &t, const std::list<shared_ptr<Data<T>>> &d);
	template <class T1, class T2>
	static TNT::Array2D<T1> interpolate(const Time &t, const DataVector<T1> &d1, const DataVector<T2> &d2);
	template <class T>
	static TNT::Array2D<T> interpolate(const Time &t, const std::list<shared_ptr<DataVector<T>>> &d);
	template <class T>
	static SO3 interpolate(const Time &t, const std::list<shared_ptr<SO3Data<T>>> &d);

	// Right now this returns the interpolated compensated value
	// I should work out something smarter in the future
	template <class T>
	static T interpolate(const Time &t, const std::list<shared_ptr<HeightData<T>>> &d);

	// ideally, the list would be passed in const here but then the returned iterator has to be const and I
	// couldn't do some things with it (like modify the list based on the returned iterator)
	template <class T>
	static typename std::list<std::shared_ptr<Data<T>>>::iterator findIndex(const Time &t, std::list<std::shared_ptr<Data<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<Data<T>>>::iterator findIndexReverse(const Time &t, std::list<std::shared_ptr<Data<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<DataVector<T>>>::iterator findIndex(const Time &t, std::list<std::shared_ptr<DataVector<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<DataVector<T>>>::iterator findIndexReverse(const Time &t, std::list<std::shared_ptr<DataVector<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<SO3Data<T>>>::iterator findIndex(const Time &t, std::list<std::shared_ptr<SO3Data<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<SO3Data<T>>>::iterator findIndexReverse(const Time &t, std::list<std::shared_ptr<SO3Data<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<SO3Data<T>>>::const_iterator findIndexReverse(const Time &t, const std::list<std::shared_ptr<SO3Data<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<HeightData<T>>>::iterator findIndex(const Time &t, std::list<std::shared_ptr<HeightData<T>>> &d);
	template <class T>
	static typename std::list<std::shared_ptr<HeightData<T>>>::iterator findIndexReverse(const Time &t, std::list<std::shared_ptr<HeightData<T>>> &d);

	template<class T>
	static typename std::list<std::shared_ptr<Data<T>>>::iterator truncate(const Time &t, std::list<std::shared_ptr<Data<T>>> &d);
	template <class T>
	static void truncate(const Time &t, std::list<std::shared_ptr<DataVector<T>>> &d);

	static bool timeSortPredicate(const shared_ptr<IData> &d1, const shared_ptr<IData> &d2){return d1->timestamp < d2->timestamp;}

	protected:
	std::mutex mMutex;
	static inline unsigned long &sNextDataID(){ static unsigned long data = 0; return data;}
};

template<class Tclass>
class Data : public IData
{
	public:
	Data() : IData() {type = DATA_TYPE_UNDEFINED;}
	Data(Tclass d, DataType t) : IData() {data = d; type = t;}

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
	DataVector() : IData() {type = DATA_TYPE_UNDEFINED;};
//	DataVector(TNT::Array2D<doubleconst > &d, DataType t){data = d.copy(); type = t;}

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
	DataImage() : IData(), angularVel(3,1,0.) 
	{
		type = DATA_TYPE_IMAGE; 
		imageFormat = IMG_FORMAT_BGR; 
		image = shared_ptr<cv::Mat>(new cv::Mat());
		cap = NULL;
		focalLength = 0;
		imageId= sNextImageID()++;
	}
	
	unsigned int imageId;
	shared_ptr<cv::Mat> image;
	shared_ptr<cv::Mat> imageGray;
	SO3 att;
	TNT::Array2D<double> angularVel;
	ImageFormat imageFormat;
	float focalLength;
	cv::Point2f center;
	shared_ptr<cv::VideoCapture> cap;
	shared_ptr<cv::Mat> cameraMatrix;
	shared_ptr<cv::Mat> distCoeffs;

	private:
	static inline unsigned int &sNextImageID(){ static unsigned int data = 0; return data;}
}; 

class DataAnnotatedImage : public IData
{
	public:
	DataAnnotatedImage() : IData() { };

	shared_ptr<cv::Mat> imageAnnotated;
	shared_ptr<DataImage> imageDataSource;

	void lock(){mMutex.lock(); if(imageDataSource != NULL) imageDataSource->lock();};
	void unlock(){mMutex.unlock(); if(imageDataSource != NULL) imageDataSource->unlock();}
};

template <class T>
class DataPhoneTemp : public Data<T>
{
	public:
	DataPhoneTemp() : Data<T>() {Data<T>::type = DATA_TYPE_PHONE_TEMP; battTemp = secTemp = fgTemp = /*tmuTemp =*/ -1;}

	T battTemp, secTemp, fgTemp;//, tmuTemp;
};

class ImageMatchData : public IData
{
	public:
	ImageMatchData() : IData(){};
	vector<vector<cv::Point2f>> featurePoints;
	shared_ptr<DataImage> imageData0, imageData1;
	shared_ptr<DataAnnotatedImage> imageAnnotated;

	void lock(){mMutex.lock(); if(imageData0 != NULL) imageData0->lock(); if(imageData1 != NULL) imageData1->lock();}
	void unlock(){mMutex.unlock(); if(imageData0 != NULL) imageData0->unlock(); if(imageData1 != NULL) imageData1->unlock();}
};

class ImageFeatureData : public IData
{
	public:
	ImageFeatureData() : IData() {};
	vector<cv::Point2f> featurePoints;
	shared_ptr<DataImage> imageData;
	shared_ptr<DataAnnotatedImage> imageAnnotated;

	void lock(){mMutex.lock(); if(imageData != NULL) imageData->lock(); if(imageAnnotated != NULL) imageAnnotated->lock();}
	void unlock(){mMutex.unlock(); if(imageData != NULL) imageData->unlock(); if(imageAnnotated != NULL) imageAnnotated->unlock();}
};

class Rect
{
	public:
	Rect(){center.x = 0; center.y = 0; area = 0;}
	Rect(vector<cv::Point2f> startContour)
	{
		contourInt.resize(startContour.size());
		for(int i=0; i<startContour.size(); i++)
		{
			contourInt[i].x = (int)(startContour[i].x+0.5);
			contourInt[i].y = (int)(startContour[i].y+0.5);
			contour.push_back(startContour[i]);
		}

		mom = moments(startContour);
		center.x = mom.m10/mom.m00;
		center.y = mom.m01/mom.m00;
		area = mom.m00;

		int maxIndex;
		double maxLength = -1;
		for(int i=0; i<contour.size(); i++)
		{
			lineLengths.push_back( norm(contour[(i+1)%4]-contour[i]) );
			if(lineLengths[i] > maxLength)
			{
				maxIndex = i;
				maxLength = lineLengths[i];
			}
		}

		cv::Point2f p1, p2;
		p1 = contour[maxIndex];
		p2 = contour[(maxIndex+1)%4];
		angle = atan2(p2.y-p1.y, p2.x-p1.x);
		while(angle < -PI/2.0)
			angle += PI; // yes, I really do mean PI and not 2*PI
		while(angle > PI/2.0)
			angle -= PI;
	}

	vector<cv::Point> contourInt;
	vector<cv::Point2f> contour;
	cv::Point2f center;
	double area;
	cv::Moments mom;
	vector<double> lineLengths;
	double angle; // of the longest line, from -PI/2 to PI/2

	static double accumAreaFunc(double val, const shared_ptr<Rect> &data){return val+data->area;}
	static cv::Point2f accumCenterFunc(const cv::Point2f &val, const shared_ptr<Rect> &data){return val+data->center;}
};

class RectGroup
{
	public:
	RectGroup() : zeroPoint(0,0) {};
	RectGroup(shared_ptr<Rect> &data){add(data);}

	cv::Point2f meanCenter;
	double meanArea;
	vector<shared_ptr<Rect>> squareData;

	void add(shared_ptr<Rect> &data)
	{
		squareData.push_back(data);

		meanArea = accumulate(squareData.begin(), squareData.end(), 0.0, Rect::accumAreaFunc) / (float)squareData.size();
		meanCenter = accumulate(squareData.begin(), squareData.end(), zeroPoint, Rect::accumCenterFunc);
		meanCenter.x /= (float)squareData.size();
		meanCenter.y /= (float)squareData.size();
	}

	private:
	const cv::Point2f zeroPoint; // this should be static but then I'd have to make a .cpp file just for it
};

class ImageTargetFindData : public IData
{
	public:
	ImageTargetFindData() : IData() {};
	shared_ptr<RectGroup> target;
	shared_ptr<DataImage> imageData;
	shared_ptr<DataAnnotatedImage> imageAnnotatedData;

	void lock(){mMutex.lock(); if(imageData != NULL) imageData->lock(); if(imageAnnotatedData != NULL) imageAnnotatedData->lock();}
	void unlock(){mMutex.unlock(); if(imageData != NULL) imageData->unlock(); if(imageAnnotatedData != NULL) imageAnnotatedData->unlock();}
};

class ImageTargetFind2Data : public IData
{
	public:
	ImageTargetFind2Data() : IData() {};
	vector<Match> matchedRegions;
	shared_ptr<DataImage> imageData;
	shared_ptr<DataAnnotatedImage> imageAnnotatedData;

	void lock(){mMutex.lock(); if(imageData != NULL) imageData->lock(); if(imageAnnotatedData != NULL) imageAnnotatedData->lock();}
	void unlock(){mMutex.unlock(); if(imageData != NULL) imageData->unlock(); if(imageAnnotatedData != NULL) imageAnnotatedData->unlock();}
};

template <class T>
class SO3Data : public IData
{
	public:
	SO3Data() : IData(){};

	SO3 rotation;
};

template <class T>
class HeightData : public IData
{
	public:
	HeightData() : IData(){};

	T height, heightRaw;
};

//////////////////////////////////////////// Template implementations //////////////////////////////////////
// the list is assumed to be sorted in increasing time order
template <class T>
TNT::Array2D<T> IData::interpolate(const Time &t, const list<shared_ptr<DataVector<T>>> &d)
{
	if(d.size() == 0)
		return TNT::Array2D<T>();

	TNT::Array2D<T> interp;

	if(t <= d.front()->timestamp)
		interp = d.front()->data.copy();
	else if(t >= d.back()->timestamp)
		interp = d.back()->data.copy();
	else
	{
		shared_ptr<DataVector<T>> d1, d2;
		typename list<shared_ptr<DataVector<T>>>::const_iterator iter = d.begin();
		while(iter != d.end() && (*iter)->timestamp < t)
		{
			d1 = *iter;
			iter++;
		}
		if(iter != d.end())
		{
			d2 = *iter;
			double a = Time::calcDiffNS(d1->timestamp,t);
			double b = Time::calcDiffNS(t, d2->timestamp);
			if(a+b == 0) // shouldn't happen in reality, but it possibly could happen here
				interp = d2->data;
			else
				interp= b/(a+b)*d1->data + a/(a+b)*d2->data;
		}
		else
			interp = d1->data.copy();
	}

	return interp;
}

template <class T1, class T2>
TNT::Array2D<T1> IData::interpolate(const Time &t, const DataVector<T1> &d1, const DataVector<T2> &d2)
{
	TNT::Array2D<double> interp;

	const DataVector<T1> *d1p, *d2p;
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
		double a = Time::calcDiffNS(d1p->timestamp,t);
		double b = Time::calcDiffNS(t, d2p->timestamp);
		if(a+b == 0) // shouldn't happen in reality, but it possibly could happen here
			interp = d2->data;
		else
			interp= b/(a+b)*d1p->data+a/(a+b)*d2p->data;
	}

	return interp;
}

template <class T1, class T2>
T1 IData::interpolate(const Time &t, const Data<T1> &d1, const Data<T2> &d2)
{
	T1 interp;

	const Data<T1> *d1p;
	const Data<T2> *d2p;
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
		double a = Time::calcDiffNS(d1p->timestamp,t);
		double b = Time::calcDiffNS(t, d2p->timestamp);
		if(a+b == 0) // shouldn't happen in reality, but it possibly could happen here
			interp = d2->data;
		else
			interp= b/(a+b)*d1p->data+a/(a+b)*d2p->data;
	}

	return interp;
}

template <class T>
T IData::interpolate(const Time &t, const std::list<shared_ptr<Data<T>>> &d)
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
		shared_ptr<Data<T>> d1, d2;
		typename list<shared_ptr<Data<T>>>::const_iterator iter = d.begin();
		while(iter != d.end() && (*iter)->timestamp < t)
		{
			d1 = *iter;
			iter++;
		}
		if(iter != d.end())
		{
			d2 = *iter;
			double a = Time::calcDiffNS(d1->timestamp,t);
			double b = Time::calcDiffNS(t, d2->timestamp);
			if(a+b == 0) // shouldn't happen in reality, but it possibly could happen here
				interp = d2->data;
			else
				interp= b/(a+b)*d1->data+a/(a+b)*d2->data;
		}
		else
			interp = d1->data;
	}

	return interp;
}

template <class T>
SO3 IData::interpolate(const Time &t, const std::list<shared_ptr<SO3Data<T>>> &d)
{
	if(d.size() == 0)
		return SO3();

	typename list<shared_ptr<SO3Data<T>>>::const_iterator i1, i2;
	i1 = findIndexReverse(t, d);
	i2 = i1;
	i2++;

	// This assumes no overflow on calcDiff
	if( i2 == d.end() ||  Time::calcDiffUS((*i1)->timestamp, t) < Time::calcDiffUS(t, (*i2)->timestamp) )
		return SO3( (*i1)->rotation);
	else
		return SO3( (*i2)->rotation);
}

template <class T>
T IData::interpolate(const Time &t, const std::list<shared_ptr<HeightData<T>>> &d)
{
	if(d.size() == 0)
		return 0;
	T interp;

	if(t < d.front()->timestamp)
		interp = d.front()->height;
	else if(t > d.back()->timestamp)
		interp = d.back()->height;
	else
	{
		shared_ptr<HeightData<T>> d1, d2;
		typename list<shared_ptr<Data<T>>>::const_iterator iter = d.begin();
		while(iter != d.end() && (*iter)->timestamp < t)
		{
			d1 = *iter;
			iter++;
		}
		if(iter != d.end())
		{
			d2 = *iter;
			double a = Time::calcDiffNS(d1->timestamp,t);
			double b = Time::calcDiffNS(t, d2->timestamp);
			if(a+b == 0) // shouldn't happen in reality, but it possibly could happen here
				interp = d2->data;
			else
				interp= b/(a+b)*d1->height+a/(a+b)*d2->height;
		}
		else
			interp = d1->height;
	}

	return interp;
}

template <class T>
typename list<shared_ptr<DataVector<T>>>::iterator IData::findIndex(const Time &t, list<shared_ptr<DataVector<T>>> &d)
{
	typename list<shared_ptr<DataVector<T>>>::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

template <class T>
typename list<shared_ptr<DataVector<T>>>::iterator IData::findIndexReverse(const Time &t, list<shared_ptr<DataVector<T>>> &d)
{
	typename list<shared_ptr<DataVector<T>>>::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp >= t)
		i--;
	return i;
}

// returns first index greater than or equal to t, starting from the front
template <class T>
typename list<shared_ptr<Data<T>>>::iterator IData::findIndex(const Time &t, list<shared_ptr<Data<T>>> &d)
{
	typename list<shared_ptr<Data<T>>>::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

// returns first index less than or equal to t, starting from the back
// If t < d.begin(), d.begin() is still returned
template <class T>
typename list<shared_ptr<Data<T>>>::iterator IData::findIndexReverse(const Time &t, list<shared_ptr<Data<T>>> &d)
{
	typename list<shared_ptr<Data<T>>>::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp > t)
		i--;
	return i;
}

template <class T>
typename list<shared_ptr<SO3Data<T>>>::iterator IData::findIndex(const Time &t, list<shared_ptr<SO3Data<T>>> &d)
{
	typename list<shared_ptr<SO3Data<T>>>::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

template <class T>
typename list<shared_ptr<SO3Data<T>>>::iterator IData::findIndexReverse(const Time &t, list<shared_ptr<SO3Data<T>>> &d)
{
	typename list<shared_ptr<SO3Data<T>>>::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp >= t)
		i--;
	return i;
}

template <class T>
typename list<shared_ptr<SO3Data<T>>>::const_iterator IData::findIndexReverse(const Time &t, const list<shared_ptr<SO3Data<T>>> &d)
{
	typename list<shared_ptr<SO3Data<T>>>::const_iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp >= t)
		i--;
	return i;
}

template <class T>
typename list<shared_ptr<HeightData<T>>>::iterator IData::findIndex(const Time &t, list<shared_ptr<HeightData<T>>> &d)
{
	typename list<shared_ptr<HeightData<T>>>::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

template <class T>
typename list<shared_ptr<HeightData<T>>>::iterator IData::findIndexReverse(const Time &t, list<shared_ptr<HeightData<T>>> &d)
{
	typename list<shared_ptr<HeightData<T>>>::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp >= t)
		i--;
	return i;
}

template <class T>
void IData::truncate(const Time &t, list<shared_ptr<DataVector<T>>> &d)
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

	typename list<shared_ptr<DataVector<T>>>::iterator iter = d.end();
	iter--;
	while(t < (*iter)->timestamp && iter != d.begin())
	{
		iter--;
		d.pop_back();
	}
}

template <class T>
typename list<shared_ptr<Data<T>>>::iterator IData::truncate(const Time &t, list<shared_ptr<Data<T>>> &d)
{
	if(d.size() == 0)
		return d.end();
	if(t > d.back()->timestamp)
	{
		typename list<shared_ptr<Data<T>>>::iterator it = d.end();
		it--;
		return it;
	}
	if(t < d.front()->timestamp)
	{
		d.clear();
		return d.end();
	}

	typename list<shared_ptr<Data<T>>>::iterator iter = d.end();
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
