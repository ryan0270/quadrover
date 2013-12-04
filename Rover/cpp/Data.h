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
#include "TrackedObject.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

namespace ICSL {
namespace Quadrotor {
using namespace ICSL::Constants;
using namespace std;
using namespace toadlet::egg;

class ActiveRegion;

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
	DATA_TYPE_IMAGE_MATCH,
	DATA_TYPE_IMAGE_FEATURES,
	DATA_TYPE_IMAGE_TARGET_FIND,
	DATA_TYPE_ANNOTATED_IMAGE,
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
	DATA_TYPE_IMAGE_TRANSLATION,
	DATA_TYPE_IMAGE_REGIONS,
	DATA_TYPE_OBJECT_TRACKER,
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

	template<class T1, class T2>	static T1 interpolate(const Time &t, const Data<T1> &d1, const Data<T2> &d2);
	template<class T> 				static T interpolate(const Time &t, const std::list<shared_ptr<Data<T>>> &d);
	template<class T1, class T2>	static TNT::Array2D<T1> interpolate(const Time &t, const DataVector<T1> &d1, const DataVector<T2> &d2);
	template<class T>				static TNT::Array2D<T> interpolate(const Time &t, const std::list<shared_ptr<DataVector<T>>> &d);
	template<class T>				static SO3 interpolate(const Time &t, const std::list<shared_ptr<SO3Data<T>>> &d);

	// Right now this returns the interpolated compensated value
	// I should work out something smarter in the future
	template <class T> static T interpolate(const Time &t, const std::list<shared_ptr<HeightData<T>>> &d);

	// ideally, the list would be passed in const here but then the returned iterator has to be const and I
	// couldn't do some things with it (like modify the list based on the returned iterator)
	template<class T> static typename std::list<std::shared_ptr<Data<T>>>::iterator			findIndex(const Time &t, std::list<std::shared_ptr<Data<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<DataVector<T>>>::iterator	findIndex(const Time &t, std::list<std::shared_ptr<DataVector<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<SO3Data<T>>>::iterator		findIndex(const Time &t, std::list<std::shared_ptr<SO3Data<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<HeightData<T>>>::iterator	findIndex(const Time &t, std::list<std::shared_ptr<HeightData<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<Data<T>>>::iterator			findIndexReverse(const Time &t, std::list<std::shared_ptr<Data<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<DataVector<T>>>::iterator	findIndexReverse(const Time &t, std::list<std::shared_ptr<DataVector<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<SO3Data<T>>>::iterator		findIndexReverse(const Time &t, std::list<std::shared_ptr<SO3Data<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<SO3Data<T>>>::const_iterator findIndexReverse(const Time &t, const std::list<std::shared_ptr<SO3Data<T>>> &d);
	template<class T> static typename std::list<std::shared_ptr<HeightData<T>>>::iterator	findIndexReverse(const Time &t, std::list<std::shared_ptr<HeightData<T>>> &d);

	template<class T> static typename std::list<std::shared_ptr<Data<T>>>::iterator truncate(const Time &t, std::list<std::shared_ptr<Data<T>>> &d);
	template<class T> static void truncate(const Time &t, std::list<std::shared_ptr<DataVector<T>>> &d);

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
	Data(Tclass d, DataType t) : IData() {dataRaw = d; type = t;}

	Tclass dataRaw, dataCalibrated;
};

template <class Tclass>
class DataVector : public IData
{
	public:
	DataVector() : IData() {type = DATA_TYPE_UNDEFINED;};

	TNT::Array2D<Tclass> dataRaw, dataCalibrated;
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
	DataAnnotatedImage() : IData() {type = DATA_TYPE_ANNOTATED_IMAGE;}

	shared_ptr<cv::Mat> imageAnnotated;
	shared_ptr<DataImage> imageDataSource;

	void lock(){mMutex.lock(); if(imageDataSource != NULL) imageDataSource->lock();};
	void unlock(){mMutex.unlock(); if(imageDataSource != NULL) imageDataSource->unlock();}
};

template <class T>
class DataPhoneTemp : public IData
{
	public:
	DataPhoneTemp() : IData() {type = DATA_TYPE_PHONE_TEMP; battTemp = secTemp = fgTemp = /*tmuTemp =*/ -1;}

	T battTemp, secTemp, fgTemp;//, tmuTemp;
};

class ImageMatchData : public IData
{
	public:
	ImageMatchData() : IData(){type = DATA_TYPE_IMAGE_MATCH;}
	vector<vector<cv::Point2f>> featurePoints;
	shared_ptr<DataImage> imageData0, imageData1;
	shared_ptr<DataAnnotatedImage> imageAnnotated;

	void lock(){mMutex.lock(); if(imageData0 != NULL) imageData0->lock(); if(imageData1 != NULL) imageData1->lock();}
	void unlock(){mMutex.unlock(); if(imageData0 != NULL) imageData0->unlock(); if(imageData1 != NULL) imageData1->unlock();}
};

class ImageFeatureData : public IData
{
	public:
	ImageFeatureData() : IData() {type = DATA_TYPE_IMAGE_FEATURES;}
	vector<cv::Point2f> featurePoints;
	shared_ptr<DataImage> imageData;
	shared_ptr<DataAnnotatedImage> imageAnnotated;

	void lock(){mMutex.lock(); if(imageData != NULL) imageData->lock(); if(imageAnnotated != NULL) imageAnnotated->lock();}
	void unlock(){mMutex.unlock(); if(imageData != NULL) imageData->unlock(); if(imageAnnotated != NULL) imageAnnotated->unlock();}
};

class ImageRegionData : public IData
{
	public:
	ImageRegionData() : IData() {type = DATA_TYPE_IMAGE_REGIONS;}
	vector<vector<cv::Point2f>> regionContours;
	vector<cv::Point2f> regionCentroids;
	vector<cv::Moments> regionMoments;
	shared_ptr<DataImage> imageData;
	shared_ptr<DataAnnotatedImage> imageAnnotated;

	void lock(){mMutex.lock(); if(imageData != NULL) imageData->lock(); if(imageAnnotated != NULL) imageAnnotated->lock();}
	void unlock(){mMutex.unlock(); if(imageData != NULL) imageData->unlock(); if(imageAnnotated != NULL) imageAnnotated->unlock();}
};

class ObjectTrackerData : public IData
{
	public:
	ObjectTrackerData() : IData() {type = DATA_TYPE_OBJECT_TRACKER;}
	vector<cv::Point2f> repeatObjectLocs, newObjectLocs; // data that will not change
	vector<shared_ptr<TrackedObject>> repeatObjects, newObjects; // these are just pointers, so the object location might change before it's used
	vector<ObjectMatch> matches;

	shared_ptr<DataImage> imageData;
	shared_ptr<DataAnnotatedImage> imageAnnotatedData;

	void lock(){mMutex.lock(); if(imageData != NULL) imageData->lock(); if(imageAnnotatedData != NULL) imageAnnotatedData->lock();}
	void unlock(){mMutex.unlock(); if(imageData != NULL) imageData->unlock(); if(imageAnnotatedData != NULL) imageAnnotatedData->unlock();}
};

class ImageTranslationData : public IData
{
	public:
	ImageTranslationData() : IData() {type = DATA_TYPE_IMAGE_TRANSLATION;}
	shared_ptr<ObjectTrackerData> objectTrackingData;
	vector<shared_ptr<TrackedObject>> goodObjects;
	vector<cv::Point2f> goodPoints; // location of found points, already adjusted for current attitude and image offset
	vector<double> goodPointScores; // match score when finding this point
	vector<cv::Point2f> nominalPoints; // nominal location of the respective found points
	cv::Point2f imageOffset;
};

template <class T>
class SO3Data : public IData
{
	public:
	SO3Data() : IData(){type = DATA_TYPE_SO3;}

	SO3 rotation;
};

template <class T>
class HeightData : public IData
{
	public:
	HeightData() : IData(){type = DATA_TYPE_HEIGHT;}

	T height, heightRaw;
};

//////////////////////////////////////////// Template implementations //////////////////////////////////////
// the list is assumed to be sorted in increasing time order
template <class T>
TNT::Array2D<T> IData::interpolate(const Time &t, const list<shared_ptr<DataVector<T>>> &d)
{
	if(d.size() == 0)
		return TNT::Array2D<T>();

	TNT::Array2D<T> interpRaw, interpCalibrated;

	if(t <= d.front()->timestamp)
	{
		interpRaw = d.front()->dataRaw.copy();
		interpCalibrated = d.front()->dataCalibrated.copy();
	}
	else if(t >= d.back()->timestamp)
	{
		interpRaw = d.back()->dataRaw.copy();
		interpCalibrated = d.back()->dataCalibrated.copy();
	}
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
			{
				interpRaw = d2->dataRaw;
				interpCalibrated = d2->dataCalibrated;
			}
			else
			{
				interpRaw = b/(a+b)*d1->dataRaw + a/(a+b)*d2->dataRaw;
				interpCalibrated = b/(a+b)*d1->dataCalibrated + a/(a+b)*d2->dataCalibrated;
			}
		}
		else
		{
			interpRaw = d1->dataRaw.copy();
			interpCalibrated = d1->dataCalibrated.copy();
		}
	}

	return interpCalibrated;
}

template <class T1, class T2>
TNT::Array2D<T1> IData::interpolate(const Time &t, const DataVector<T1> &d1, const DataVector<T2> &d2)
{
	TNT::Array2D<double> interpRaw, interpCalibrated;

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
	{
		interpRaw = d1p->dataRaw.copy();
		interpCalibrated = d1p->dataCalibrated.copy();
	}
	else if(t > d2p->timestamp)
	{
		interpRaw = d2p->dataRaw.copy();
		interpCalibrated = d2p->dataCalibrated.copy();
	}
	else
	{
		double a = Time::calcDiffNS(d1p->timestamp,t);
		double b = Time::calcDiffNS(t, d2p->timestamp);
		if(a+b == 0) // shouldn't happen in reality, but it possibly could happen here
		{
			interpRaw = d2->dataRaw;
			interpCalibrated = d2->dataCalibrated;
		}
		else
		{
			interpRaw= b/(a+b)*d1p->dataRaw+a/(a+b)*d2p->dataRaw;
			interpCalibrated= b/(a+b)*d1p->dataCalibrated+a/(a+b)*d2p->dataCalibrated;
		}
	}

	return interpCalibrated;
}

template <class T1, class T2>
T1 IData::interpolate(const Time &t, const Data<T1> &d1, const Data<T2> &d2)
{
	T1 interpRaw, interpCalibrated;

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
	{
		interpRaw = d1p->dataRaw;
		interpCalibrated = d1p->dataCalibrated;
	}
	else if(t > d2p->timestamp)
	{
		interpRaw = d2p->dataRaw;
		interpCalibrated = d2p->dataCalibrated;
	}
	else
	{
		double a = Time::calcDiffNS(d1p->timestamp,t);
		double b = Time::calcDiffNS(t, d2p->timestamp);
		if(a+b == 0) // shouldn't happen in reality, but it possibly could happen here
		{
			interpRaw = d2->dataRaw;
			interpCalibrated = d2->dataCalibrated;
		}
		else
		{
			interpRaw= b/(a+b)*d1p->dataRaw+a/(a+b)*d2p->dataRaw;
			interpCalibrated= b/(a+b)*d1p->dataCalibrated+a/(a+b)*d2p->dataCalibrated;
		}
	}

	return interpCalibrated;
}

template <class T>
T IData::interpolate(const Time &t, const std::list<shared_ptr<Data<T>>> &d)
{
	if(d.size() == 0)
		return 0;

	T interpRaw, interpCalibrated;

	if(t < d.front()->timestamp)
	{
		interpRaw = d.front()->dataRaw;
		interpCalibrated = d.front()->dataCalibrated;
	}
	else if(t > d.back()->timestamp)
	{
		interpRaw = d.back()->dataRaw;
		interpCalibrated = d.back()->dataCalibrated;
	}
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
			{
				interpRaw = d2->dataRaw;
				interpCalibrated = d2->dataCalibrated;
			}
			else
			{
				interpRaw= b/(a+b)*d1->dataRaw+a/(a+b)*d2->dataRaw;
				interpCalibrated= b/(a+b)*d1->dataCalibrated+a/(a+b)*d2->dataCalibrated;
			}
		}
		else
		{
			interpRaw = d1->dataRaw;
			interpCalibrated = d1->dataCalibrated;
		}
	}

	return interpCalibrated;
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
