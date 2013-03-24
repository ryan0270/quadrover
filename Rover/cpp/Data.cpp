#include "Data.h"

namespace ICSL {
namespace Quadrotor {
using namespace std;
double Data::interpolate(Time const &t, Data const &d1, Data const &d2)
{
	double interp;

	Data const *d1p,  *d2p;
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
		double a = Time::calcDiffUS(d1p->timestamp,t);
		double b = Time::calcDiffUS(t, d2p->timestamp);
		interp= b/(a+b)*d1p->data+a/(a+b)*d2p->data;
	}

	return interp;
}

TNT::Array2D<double> Data::interpolate(Time const &t, DataVector const &d1, DataVector const &d2)
{
	TNT::Array2D<double> interp;

	DataVector const *d1p, *d2p;
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

// the list is assumed to sorted in increasing time order
double Data::interpolate(Time const &t, list<shared_ptr<Data> > const &d)
{
	if(d.size() == 0)
		return 0;
	double interp;

	if(t < d.front()->timestamp)
		interp = d.front()->data;
	else if(t > d.back()->timestamp)
		interp = d.back()->data;
	else
	{
		shared_ptr<Data> d1, d2;
		list<shared_ptr<Data> >::const_iterator iter = d.begin();
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
			interp= b/(a+b)*d1->data+a/(a+b)*d2->data;
		}
		else
			interp = d1->data;
	}

	return interp;
}

// the list is assumed to sorted in increasing time order
TNT::Array2D<double> Data::interpolate(Time const &t, list<shared_ptr<DataVector> > const &d)
{
	if(d.size() == 0)
		return TNT::Array2D<double>();

	TNT::Array2D<double> interp;

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
		shared_ptr<DataVector> d1, d2;
		list<shared_ptr<DataVector> >::const_iterator iter = d.begin();
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

// returns first index greater than or equal to t, starting from the front
list<shared_ptr<Data> >::iterator Data::findIndex(Time const &t, list<shared_ptr<Data> > &d)
{
	list<shared_ptr<Data> >::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

list<shared_ptr<DataVector> >::iterator Data::findIndex(Time const &t, list<shared_ptr<DataVector> > &d)
{
	list<shared_ptr<DataVector> >::iterator i = d.begin();
	while(i != d.end() && (*i)->timestamp < t)
		i++;
	return i;
}

// returns first index less than or equal to t, starting from the back
// If t < d.begin(), d.begin() is still returned
list<shared_ptr<Data> >::iterator Data::findIndexReverse(Time const &t, list<shared_ptr<Data> > &d)
{
	list<shared_ptr<Data> >::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp > t)
		i--;
	return i;
}

list<shared_ptr<DataVector> >::iterator Data::findIndexReverse(Time const &t, list<shared_ptr<DataVector> > &d)
{
	list<shared_ptr<DataVector> >::iterator i = d.end();
	if(d.size() == 0)
		return i;

	i--;
	while(i != d.begin() && (*i)->timestamp > t)
		i--;
	return i;
}

list<shared_ptr<Data> >::iterator Data::truncate(Time const &t, list<shared_ptr<Data> > &d)
{
	if(d.size() == 0)
		return d.end();
	if(t > d.back()->timestamp)
	{
		list<shared_ptr<Data> >::iterator it = d.end();
		it--;
		return it;
	}
	if(t < d.front()->timestamp)
	{
		d.clear();
		return d.end();
	}

	list<shared_ptr<Data> >::iterator iter = d.end();
	iter--;
	while(t < (*iter)->timestamp && iter != d.begin())
	{
		iter--;
		d.pop_back();
	}

	return iter;
}

void Data::truncate(Time const &t, list<shared_ptr<DataVector> > &d)
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

	list<shared_ptr<DataVector> >::iterator iter = d.end();
	iter--;
	while(t < (*iter)->timestamp && iter != d.begin())
	{
		iter--;
		d.pop_back();
	}
}

} // namespace Quadrotor
} // namesapce ICSL
