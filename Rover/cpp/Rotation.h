#ifndef ICSL_ROTATION
#define ICSL_ROTATION

#include "TNT/tnt.h"
#include "TNT_Utils.h"

namespace ICSL {

class SO3_LieAlgebra
{
	public:
	SO3_LieAlgebra(){};
	SO3_LieAlgebra(const TNT::Array2D<double> &v)
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
	
	TNT::Array2D<double> toVector() const {return mVector.copy();}
	TNT::Array2D<double> toMatrix() const
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

	protected:
	TNT::Array2D<double> mVector;
};

class SO3 
{
	public:
	SO3(){mRotMat = TNT::createIdentity(3.0);}
	SO3(SO3 const &so3){mRotMat = so3.mRotMat.copy();}
	SO3(TNT::Array2D<double> const &rotMat){mRotMat = rotMat.copy();}

	void setRotMat(const TNT::Array2D<double> &R){mRotMat.inject(R);}

	TNT::Array2D<double> getRotMat() const {return mRotMat.copy();}
	TNT::Array2D<double> getEulerAnglesZYX() const
	{
		TNT::Array2D<double> euler(3,1);
		euler[0][0] = atan2(mRotMat[2][1], mRotMat[2][2]); // roll ... @TODO: this should be range checked
		euler[1][0] = atan2(-mRotMat[2][0], sqrt(mRotMat[0][0]*mRotMat[0][0] + mRotMat[1][0]*mRotMat[1][0])); // pitch 
		euler[2][0] = atan2(mRotMat[1][0], mRotMat[0][0]);

		return euler;
	}

	SO3_LieAlgebra log(double theta) const
	{
		TNT::Array2D<double> w(3,1);
		if(abs(theta) > 0)
		{
			double sTheta2 = 2.0*sin(theta);
			w[0][0] = (mRotMat[2][1]-mRotMat[1][2])/sTheta2;
			w[1][0] = (mRotMat[0][2]-mRotMat[2][0])/sTheta2;
			w[2][0] = (mRotMat[1][0]-mRotMat[0][1])/sTheta2;
		}
		else
			w = TNT::Array2D<double>(3,1,0.0);

		return SO3_LieAlgebra(w);
	}

	bool isIdentity() const {return mRotMat[0][0] == 1 && mRotMat[1][1] == 1;}

	SO3 inv() const { return SO3(transpose(mRotMat));}

	SO3& operator*=(const SO3& rhs)
	{
		mRotMat.inject( matmult(mRotMat, rhs.mRotMat) );
		return *this;
	}

	protected:
	TNT::Array2D<double> mRotMat;
};

inline SO3 operator*(SO3 lhs, const SO3 &rhs)
{
	lhs *= rhs;
	return lhs;
}

}

#endif
