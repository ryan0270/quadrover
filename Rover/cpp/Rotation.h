#ifndef ICSL_ROTATION
#define ICSL_ROTATION

#include "TNT/tnt.h"
#include "TNT_Utils.h"

#include "constants.h"

namespace ICSL {

class Quaternion
{
	public:
	Quaternion();
	Quaternion(const Quaternion &q);
	Quaternion(Quaternion &&q) = default;
	Quaternion(const TNT::Array2D<double> &m);
	Quaternion(double w, double x, double y, double z);

	double getScalarPart() const {return mVal[0];}
	TNT::Array2D<double> getVectorPart() const;
	TNT::Array2D<double> toRotMat() const;

	void set(double w, double x, double y, double z);
	void set(const Quaternion &q);
//	void setFromRotMat2(const TNT::Array2D<double> &m);
	void setFromRotMat(const TNT::Array2D<double> &m);

	Quaternion conj() const;
	double dot(const Quaternion &q) const;
	double norm() const;
	Quaternion inv() const;

	// These are for unit quaternions only
	Quaternion inv1() const {return conj();}
	void getAngleAxis1(double &angleOUT, TNT::Array2D<double> &axisOUT) const;

	TNT::Array2D<double> toAnglesZYX() const;

	// Actually, for unit quat we only need mVal[0] == 1, but I'm not sure if that holds in general so
	// for future proofing I'll do this 
	bool isIdentity() const {return mVal[0] == 1 && mVal[1] == mVal[2] == mVal[3] == 0;}

	Quaternion& operator=(const Quaternion &other);

	// NOTE: lhs will LEFT-multiply the current rotation 
	Quaternion& operator*=(const Quaternion &q);
	Quaternion& operator*=(double s);
	Quaternion& operator/=(double s);
	Quaternion& operator+=(const Quaternion &q);
	Quaternion& operator-=(const Quaternion &q);

	protected:
	double mVal[4];
};

inline Quaternion operator*(const Quaternion &lhs, Quaternion rhs)
{
	rhs *= lhs;
	return rhs;
}

inline Quaternion operator*(Quaternion q, double s)
{
	q *= s;
	return q;
}

inline Quaternion operator*(double s, Quaternion q)
{
	q *= s;
	return q;
}

inline Quaternion operator/(Quaternion q, double s)
{
	q /= s;
	return q;
}

inline Quaternion operator+(Quaternion lhs, const Quaternion &rhs)
{
	lhs += rhs;
	return lhs;
}

inline Quaternion operator-(Quaternion lhs, const Quaternion &rhs)
{
	lhs -= rhs;
	return lhs;
}

class SO3;
class SO3_LieAlgebra
{
	public:
	SO3_LieAlgebra();
	SO3_LieAlgebra(const TNT::Array2D<double> &v);
	
	TNT::Array2D<double> toVector() const {return mVector.copy();}
	TNT::Array2D<double> toMatrix() const;

	SO3 integrate(double dt);

	protected:
	TNT::Array2D<double> mVector;
};

class SO3 
{
	public:
	SO3();
	SO3(const SO3 &so3);
	SO3(SO3&& so3) = default;
	SO3(const TNT::Array2D<double> &rotMat);
	SO3(const Quaternion &q);

	// to the identity rotation
	void reset();

	void setRotMat(const TNT::Array2D<double> &R){mQuaternion.setFromRotMat(R);}

	TNT::Array2D<double> getRotMat() const {return mQuaternion.toRotMat();}
	TNT::Array2D<double> getAnglesZYX() const;
	Quaternion getQuaternion() const {return mQuaternion;}

	void getAngleAxis(double &angle, TNT::Array2D<double> &axis){mQuaternion.getAngleAxis1(angle, axis);}

	bool isIdentity() const;

	SO3 inv() const { return SO3(mQuaternion.conj());}

	SO3& operator=(const SO3 &other);

	// NOTE: lhs will LEFT-multiply the current rotation 
	SO3& operator*=(const SO3& lhs);

	protected:
	Quaternion mQuaternion;
};

inline SO3 operator*(const SO3 &lhs, SO3 rhs)
{
	rhs *= lhs;
	return rhs;
}

// TODO: Probably faster to do direct quaternion multiplication
// but I'm not sure the exact math for that yet
template <class T>
inline TNT::Array2D<T> operator*(const SO3 &lhs, const TNT::Array2D<T> &v)
{
	assert(v.dim1() == 3 && v.dim2() == 1);

	Quaternion q0 = lhs.getQuaternion();
	Quaternion q1(0,v[0][0],v[1][0],v[2][0]);

	return (q0*q1*q0.conj()).getVectorPart();
}

}

#endif
