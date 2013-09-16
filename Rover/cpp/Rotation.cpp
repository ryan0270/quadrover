#include "Rotation.h"

namespace ICSL{
using namespace std;
using namespace TNT;
using namespace ICSL::Constants;

Quaternion::Quaternion()
{
	mVal[0] = 1;
	mVal[1] = mVal[2] = mVal[3] = 0;
}

Quaternion::Quaternion(const Quaternion &q)
{
	for(int i=0; i<4; i++)
		mVal[i] = q.mVal[i];
}

Quaternion::Quaternion(double w, double x, double y, double z)
{
	mVal[0] = w;
	mVal[1] = x;
	mVal[2] = y;
	mVal[3] = z;
}

Quaternion::Quaternion(const Array2D<double> &m)
{
	setFromRotMat(m);
}

Array2D<double> Quaternion::getVectorPart() const
{
	Array2D<double> v(3,1);
	v[0][0] = mVal[1];
	v[1][0] = mVal[2];
	v[2][0] = mVal[3];

	return v;
}

// Adapted from the paper "Integrating generic sensor fusion algorithms with sound state representations through encapsulation of manifolds"
void Quaternion::getAngleAxis1(double &angleOUT, Array2D<double> &axisOUT) const
{
	if(axisOUT.dim1() != 3 || axisOUT.dim2() != 1)
		axisOUT = Array2D<double>(3,1);

	double n = sqrt(mVal[1]*mVal[1]+mVal[2]*mVal[2]+mVal[3]*mVal[3]);

	if(n == 0)
	{
		angleOUT = 0;
		axisOUT[0][0] = axisOUT[1][0] = axisOUT[2][0] = 0;
	}
	else if(mVal[0] == 0)
	{
		angleOUT = PI;
		axisOUT[0][0] = mVal[1];
		axisOUT[1][0] = mVal[2];
		axisOUT[2][0] = mVal[3];
		axisOUT = 1.0/norm2(axisOUT)*axisOUT;
	}
	else
	{
		angleOUT = 2.0*atan(n/mVal[0]);
		axisOUT[0][0] = mVal[1];
		axisOUT[1][0] = mVal[2];
		axisOUT[2][0] = mVal[3];
		axisOUT = 1.0/norm2(axisOUT)*axisOUT;
	}
}

// taken from http://en.wikipedia.org/wiki/Rotation_matrix#Quaternion
void Quaternion::setFromRotMat(const Array2D<double> &m)
{
	double t = trace(m);

	if(t >= 0)
	{
		double r = sqrt(1.0+t);
		double s = 0.5/r;
		mVal[0] = 0.5*r;
		mVal[1] = (m[2][1]-m[1][2])*s;
		mVal[2] = (m[0][2]-m[2][0])*s;
		mVal[3] = (m[1][0]-m[0][1])*s;
	}
	else if(m[0][0] > m[1][1] && m[0][0] > m[2][2])
	{
		double r = sqrt(1.0+m[0][0]-m[1][1]-m[2][2]);
		double s = 0.5/r;
		mVal[0] = (m[2][1]-m[1][2])*s;
		mVal[1] = 0.5*r;
		mVal[2] = (m[0][1]+m[1][0])*s;
		mVal[3] = (m[2][0]+m[0][2])*s;
	}
	else if(m[1][1] > m[2][2])
	{
		double r = sqrt(1.0+m[1][1]-m[0][0]-m[2][2]);
		double s = 0.5/r;
		mVal[0] = (m[0][2]-m[2][0])*s;
		mVal[1] = (m[0][1]+m[1][0])*s;
		mVal[2] = 0.5*r;
		mVal[3] = (m[1][2]+m[2][1])*s;
	}
	else
	{
		double r = sqrt(1.0+m[2][2]-m[0][0]-m[1][1]);
		double s = 0.5/r;
		mVal[0] = (m[1][0]-m[0][1])*s;
		mVal[1] = (m[0][2]+m[2][0])*s;
		mVal[2] = (m[1][2]+m[2][1])*s;
		mVal[3] = 0.5*r;
	}
}

void Quaternion::set(double w, double x, double y, double z)
{
	mVal[0] = w;
	mVal[1] = x;
	mVal[2] = y;
	mVal[3] = z;
}

void Quaternion::set(const Quaternion &q)
{
	memcpy(&(mVal[0]), &(q.mVal[0]), 4*sizeof(double));
}

Quaternion Quaternion::conj() const
{
	Quaternion q(*this);
	for(int i=1; i<4; i++)
		q.mVal[i] *= -1;

	return q;
}

Array2D<double> Quaternion::toRotMat() const
{
	Array2D<double> R(3,3);
	double xSq = mVal[1]*mVal[1];
	double ySq = mVal[2]*mVal[2];
	double zSq = mVal[3]*mVal[3];
	double wSq = mVal[0]*mVal[0];
	double xy2 = 2.0*mVal[1]*mVal[2];
	double xz2 = 2.0*mVal[1]*mVal[3];
	double yz2 = 2.0*mVal[2]*mVal[3];
	double wx2 = 2.0*mVal[0]*mVal[1];
	double wy2 = 2.0*mVal[0]*mVal[2];
	double wz2 = 2.0*mVal[0]*mVal[3];

	R[0][0] = wSq + xSq - ySq - zSq;
	R[0][1] = xy2 - wz2;
	R[0][2] = xz2 + wy2;

	R[1][0] = xy2 + wz2;
	R[1][1] = wSq - xSq + ySq - zSq;
	R[1][2] = yz2 - wx2;
	
	R[2][0] = xz2 - wy2;
	R[2][1] = yz2 + wx2;
	R[2][2] = wSq - xSq - ySq + zSq; 
	return R;
}

double Quaternion::dot(const Quaternion &q) const
{
	double d = 0;
	for(int i=0; i<4; i++)
		d += mVal[i]*q.mVal[i];

	return d;
}

double Quaternion::norm() const
{
	return sqrt(dot(*this));
}

Quaternion Quaternion::inv() const
{
	return conj()/norm();
}

Array2D<double> Quaternion::toAnglesZYX() const
{
	// I'll do this better later, but I'm lazy
	Array2D<double> rotMat = toRotMat();

	Array2D<double> euler(3,1);
	euler[0][0] = atan2(rotMat[2][1], rotMat[2][2]); // roll ... @TODO: this should be range checked
	euler[1][0] = atan2(-rotMat[2][0], sqrt(1.0-rotMat[2][0]*rotMat[2][0]));
	euler[2][0] = atan2(rotMat[1][0], rotMat[0][0]);

	return euler;
}

Quaternion& Quaternion::operator=(const Quaternion &other) // note that 'other' is passed by value
{
	memcpy(&(mVal[0]), &(other.mVal[0]), 4*sizeof(double));
	return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& q)
{
	double r[4];
	memcpy(&(r[0]), &(mVal[0]), 4*sizeof(double));
	
	mVal[0] = r[0]*q.mVal[0]-r[1]*q.mVal[1]-r[2]*q.mVal[2]-r[3]*q.mVal[3];
	mVal[1] = r[0]*q.mVal[1]+r[1]*q.mVal[0]-r[2]*q.mVal[3]+r[3]*q.mVal[2];
	mVal[2] = r[0]*q.mVal[2]+r[1]*q.mVal[3]+r[2]*q.mVal[0]-r[3]*q.mVal[1];
	mVal[3] = r[0]*q.mVal[3]-r[1]*q.mVal[2]+r[2]*q.mVal[1]+r[3]*q.mVal[0];

	return *this;
}

Quaternion& Quaternion::operator*=(double s)
{
	for(int i=0; i<4; i++)
		mVal[i] *= s;
}

Quaternion& Quaternion::operator/=(double s)
{
	for(int i=0; i<4; i++)
		mVal[i] /= s;
}

Quaternion& Quaternion::operator+=(const Quaternion& q)
{
	for(int i=0; i<4; i++)
		mVal[i] += q.mVal[i];

	return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& q)
{
	for(int i=0; i<4; i++)
		mVal[i] -= q.mVal[i];

	return *this;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
SO3_LieAlgebra::SO3_LieAlgebra()
{
}

SO3_LieAlgebra::SO3_LieAlgebra(const TNT::Array2D<double> &v)
{
	if(v.dim2() == 1)
		mVector = v.copy();
	else // assume it's 3x3 matrix form
	{
		mVector = TNT::Array2D<double>(3,1);
		mVector[0][0] = v[2][1];
		mVector[1][0] = v[0][2];
		mVector[2][0] = v[1][0];
	}
}

TNT::Array2D<double> SO3_LieAlgebra::toMatrix() const
{
	if(mVector.dim1() != 3)
		return TNT::Array2D<double>(3,1,0.0);

	TNT::Array2D<double> m(3,3);
	m[0][0] = m[1][1] = m[2][2] = 0;
	m[1][2] = -(m[2][1] = mVector[0][0]);
	m[2][0] = -(m[0][2] = mVector[1][0]);
	m[0][1] = -(m[1][0] = mVector[2][0]);

	return m;
}

SO3 SO3_LieAlgebra::integrate(double dt)
{
	double n = norm2(mVector);
	if(n == 0)
		return SO3();

	double s = sin(n*dt);
	double c = cos(n*dt);
	Array2D<double> v_x = this->toMatrix();
	Array2D<double> A = createIdentity(3.0) + s/n*v_x + (1-c)/n/n*matmult(v_x, v_x);
	return SO3(A);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SO3::SO3() : mQuaternion(1,0,0,0)
{ }

SO3::SO3(const SO3 &so3)
{
	mQuaternion.set(so3.mQuaternion);
}

SO3::SO3(const TNT::Array2D<double> &rotMat)
{
	mQuaternion.setFromRotMat(rotMat);
}

SO3::SO3(const Quaternion &q)
{
	mQuaternion.set(q);
}

// I think this corresponds to Tait-Bryan angles actually
TNT::Array2D<double> SO3::getAnglesZYX() const
{
	return mQuaternion.toAnglesZYX();
}

void SO3::reset()
{
	mQuaternion.set(1,0,0,0);
}

bool SO3::isIdentity() const
{return mQuaternion.isIdentity(); }


SO3& SO3::operator*=(const SO3& lhs)
{
	mQuaternion *= lhs.mQuaternion;
	return *this;
}

SO3& SO3::operator=(const SO3 &other)
{
	mQuaternion = other.mQuaternion;
	return *this;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace ICSL
