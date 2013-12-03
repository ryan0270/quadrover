/// This file is for useful matrix operations not included in the
// base TNT Array classes
#ifndef TNT_UTILS_h
#define TNT_UTILS_h

#include "TNT/tnt.h"
#include <cmath>
#include <toadlet/egg.h>

namespace TNT{
	template <class T>
	inline Array2D<T> transpose(const Array2D<T> &array)
	{
		int numRow = array.dim1();
		int numCol = array.dim2();

		Array2D<T> result(numCol,numRow);

		int i,j;
		for( i=0; i<numRow; i++)
			for(j=0; j<numCol; j++)
				result[j][i] = array[i][j];

		return result;
	}

	template <class T>
	inline T trace(const Array2D<T> &array)
	{
		assert(array.dim1() == array.dim2());

		T tr = 0;
		for(int i=0; i<array.dim1(); i++)
			tr += array[i][i];

		return tr;
	}

	template <class T1, class T2>
	inline Array2D<T1> scalarMult(T2 scalar, const Array2D<T1> &mat)
	{
		Array2D<T1> result(mat.dim1(), mat.dim2());

		for(int i=0; i<mat.dim1(); i++)
			for(int j=0; j<mat.dim2(); j++)
				result[i][j] = scalar*mat[i][j];

		return result;
	}

	template <class T1, class T2>
	inline Array2D<T1> operator*(T2 scalar, const Array2D<T1> &mat)
	{
		return scalarMult(scalar, mat);
	}

	template <class T1, class T2>
	inline Array2D<T1> operator*(const Array2D<T1> &mat, T2 scalar)
	{
		return scalarMult(scalar, mat);
	}

	/*!
	  requires a 1D vector
	  */
	template <class T>
	inline T norm2(const Array2D<T> &mat)
	{
		assert(mat.dim1() == 1 || mat.dim2() == 1);
		T result = 0;
		if(mat.dim1()==1)
			for(int i=0; i<mat.dim2(); i++)
				result += pow(mat[0][i],2);
		else
			for(int i=0; i<mat.dim1(); i++)
				result += pow(mat[i][0],2);

		return sqrt(result);
	}

	/*!
	  Does an inline normalization (original vector is modified)
	  requires a 1D vector
	  */
	template <class T>
	inline void normalize(Array2D<T> &mat)
	{
		assert(mat.dim1() == 1 || mat.dim2() == 1);

		double den = 0;
		if(mat.dim1() == 1)
		{
			for(int j=0; j<mat.dim2(); j++)
				den += mat[0][j]*mat[0][j];
			den = sqrt(den);
			for(int j=0; j<mat.dim2(); j++)
				mat[0][j] /= den;
		}
		else
		{
			for(int i=0; i<mat.dim1(); i++)
				den += mat[i][0]*mat[0][i];
			den = sqrt(den);
			for(int i=0; i<mat.dim1(); i++)
				mat[i][0] /= den;
		}
	}

	template <class T>
	inline T matmultS(const Array2D<T> &mat1, const Array2D<T> &mat2)
	{
		assert(mat1.dim1() == 1 && mat2.dim2() == 1);
		return matmult(mat1, mat2)[0][0];
	}


	template <class T>
	inline Array2D<T> submat(const Array2D<T> &mat, int rowStart, int rowStop, int colStart, int colStop)
	{
		assert(rowStart >= 0 && rowStop <= mat.dim1() &&
				colStart >= 0 && colStop <= mat.dim2());

		Array2D<T> result(rowStop-rowStart+1, colStop-colStart+1);
		for(int i=0; i<=rowStop-rowStart; i++)
			for(int j=0; j<=colStop-colStart; j++)
				result[i][j] = mat[i+rowStart][j+colStart];

		return result;
	}

	/*!
	  \param mat is modified to have its rows replaces
	  */
	template <class T>
	inline void assignRows(Array2D<T> &mat, int rowStart, int rowStop, const Array2D<T> &rows)
	{
		assert(rowStart >= 0 && rowStop < mat.dim1() &&
				rows.dim1() == rowStop-rowStart+1 && rows.dim2() == mat.dim2());

		for(int r=rowStart; r<=rowStop; r++)
			for(int c=0; c<mat.dim2(); c++)
				mat[r][c] = rows[r-rowStart][c];
	}

	/*!
	  \param mat is modified to have its rows replaces
	  */
	template <class T>
	inline void assignRows(Array2D<T> &mat, int rowStart, int rowStop, T val)
	{
		assert(rowStart >= 0 && rowStop < mat.dim1());

		for(int r=rowStart; r<=rowStop; r++)
			for(int c=0; c<mat.dim2(); c++)
				mat[r][c] = val;
	}

	/*!
	  \param mat is modified to have its cols replaces
	  */
	template <class T>
	inline void assignColumns(Array2D<T> &mat, int colStart, int colStop, const Array2D<T> &cols)
	{
		assert(colStart >= 0 && colStop < mat.dim2() &&
				cols.dim2() == colStop-colStart+1 && cols.dim1() == mat.dim1());
		//	if(colStart < 0 || colStop >= mat.dim2() || cols.dim2() != colStop-colStart+1 || cols.dim1() != mat.dim1() )
		//		throw("assignColumns error");

		for(int r=0; r<mat.dim1(); r++)
			for(int c=colStart; c<=colStop; c++)
				mat[r][c] = cols[r][c-colStart];
	}

	/*!
	  \param mat is modified to have its cols replaces
	  */
	template <class T>
	inline void assignColumns(Array2D<T> &mat, int colStart, int colStop, T val)
	{
		assert(colStart >= 0 && colStop < mat.dim2());

		for(int r=0; r<mat.dim1(); r++)
			for(int c=colStart; c<=colStop; c++)
				mat[r][c] = val;
	}

	template <class T1, class T2>
	inline void assignSubmat(Array2D<T1> &mat, int rowStart, int colStart, const Array2D<T2> &submat)
	{
		assert(mat.dim1() >= rowStart+submat.dim1() && mat.dim2() >= colStart+submat.dim2());

		for(int i=0; i<submat.dim1(); i++)
			for(int j=0; j<submat.dim2(); j++)
				mat[rowStart+i][colStart+j] = submat[i][j];
	}

	template <class T>
	inline Array2D<T> stackVertical(const Array2D<T> &matTop, const Array2D<T> &matBottom)
	{
		assert(matTop.dim2() == matBottom.dim2());
		//	if(matTop.dim2() != matBottom.dim2())
		//		throw("TNT::stackVertical error - matrix dimensions do not match");

		Array2D<T> matNew(matTop.dim1()+matBottom.dim1(), matTop.dim2());
		assignRows(matNew,0,matTop.dim1()-1,matTop);
		assignRows(matNew,matTop.dim1(),matNew.dim1()-1,matBottom);

		return matNew;
	}

	template <class T>
	inline Array2D<T> stackVertical(const Array2D<T> &matTop, T val)
	{
		assert(matTop.dim2() == 1);
		//	if(matTop.dim2() != 1)
		//		throw("TNT::stackVertical error - matTop.dim2() != 1");

		return stackVertical(matTop, Array2D<T>(1,1,val));
	}

	template <class T>
	inline Array2D<T> stackHorizontal(const Array2D<T> &matLeft, const Array2D<T> &matRight)
	{
		assert(matLeft.dim1() == matRight.dim1());

		Array2D<T> matNew(matLeft.dim1(), matLeft.dim2()+matRight.dim2());
		assignColumns(matNew, 0, matLeft.dim2()-1, matLeft);
		assignColumns(matNew, matLeft.dim2(), matNew.dim2()-1, matRight);

		return matNew;
	}

	template <class T>
	inline Array2D<T> cross(const Array2D<T> &matLeft, const Array2D<T> &matRight)
	{
		assert(matLeft.dim1() == 3 && matRight.dim1() == 3 && 
				matLeft.dim2() == 1 && matRight.dim2() == 1);

		Array2D<T> matNew(3,1);
		matNew[0][0] = matLeft[1][0]*matRight[2][0]-matLeft[2][0]*matRight[1][0];
		matNew[1][0] = matLeft[2][0]*matRight[0][0]-matLeft[0][0]*matRight[2][0];
		matNew[2][0] = matLeft[0][0]*matRight[1][0]-matLeft[1][0]*matRight[0][0];

		return matNew;
	}

	template <class T1, class T2>
	inline T1 dot(const TNT::Array2D<T1> &m1, const TNT::Array2D<T2> &m2)
	{
		assert(m1.dim2() == 1 && m2.dim2() == 1 && m1.dim1() == m2.dim1());

		double d = 0;
		for(int i=0; i<m1.dim1(); i++)
			d += m1[i][0]*m2[i][0];

		return d;
	}

	template <class T>
	inline Array2D<T> extractDiagonal(const Array2D<T> &mat)
	{
		assert(mat.dim1() == mat.dim2());

		Array2D<T> matNew(mat.dim1(),1);
		for(int i=0; i<mat.dim1(); i++)
			matNew[i][0] = mat[i][i];

		return matNew;
	}

	template <class T>
	inline Array2D<T> createIdentity(T n)
	{
		assert(n > 0);

		Array2D<T> matNew(n,n,0.0);
		for(int i=0; i<n; i++)
			matNew[i][i] = 1.0;

		return matNew;
	}

	template <class T>
	inline Array2D<T> createRotMat(int axis, T angleInRad)
	{
		Array2D<T> rotMat(3,3,0.0);
		T c = cos(angleInRad);
		T s = sin(angleInRad);
		switch(axis)
		{
			case 0:
				rotMat[0][0] = 1;
				rotMat[1][1] = rotMat[2][2] = c;
				rotMat[1][2] = -(rotMat[2][1] = s);
				break;
			case 1:
				rotMat[1][1] = 1;
				rotMat[0][0] = rotMat[2][2] = c;
				rotMat[2][0] = -(rotMat[0][2] = s);
				break;
			case 2:
				rotMat[2][2] = 1;
				rotMat[0][0] = rotMat[1][1] = c;
				rotMat[0][1] = -(rotMat[1][0] = s);
				break;
		}

		return rotMat;
	}

	template <class T>
	inline Array2D<T> createRotMat_ZYX(T aZ, T aY, T aX)
	{
		T c1 = cos(aZ); T s1 = sin(aZ);
		T c2 = cos(aY); T s2 = sin(aY);
		T c3 = cos(aX); T s3 = sin(aX);

		Array2D<T> rotMat(3,3);
		rotMat[0][0] = c1*c2; 	rotMat[0][1] = c1*s2*s3-c3*s1;	rotMat[0][2] = s1*s3+c1*c3*s2;
		rotMat[1][0] = c2*s1;	rotMat[1][1] = c1*c3+s1*s2*s3;	rotMat[1][2] = c3*s1*s2-c1*s3;
		rotMat[2][0] = -s2;		rotMat[2][1] = c2*s3;			rotMat[2][2] = c2*c3;

		return rotMat;
	}

	template <class T>
	inline void printArray(const toadlet::egg::String &prefix, const TNT::Array2D<T> &m)
	{
		toadlet::egg::String str(prefix);
		if(m.dim2() == 1)
		{
			for(int i=0; i<m.dim1(); i++)
				str = str+m[i][0]+"\t";
			str = str+"\n";
		}
		else
		{
			for(int i=0; i<m.dim1(); i++)
			{
				for(int j=0; j<m.dim2(); j++)
					str = str+m[i][j]+"\t";
				str = str+"\n";
			}
		}
		toadlet::egg::Log::alert(str.substr(0, str.length()-1));
	}

	template <class T>
	inline void printArray(const TNT::Array2D<T> &m)
	{ printArray("",m); }

	template <class T1, class T2>
	inline TNT::Array2D<T1> blkdiag(const TNT::Array2D<T1> &m1, const TNT::Array2D<T2> &m2)
	{
		Array2D<T1> m(m1.dim1()+m2.dim1(), m1.dim2()+m2.dim2());
		for(int i=0; i<m.dim1(); i++)
			for(int j=0; j<m.dim2(); j++)
			{
				if(i < m1.dim1() && j < m1.dim2())
					m[i][j] = m1[i][j];
				else if(i >= m1.dim1() && j >=m1.dim1())
					m[i][j] = m2[i-m1.dim1()][j-m1.dim2()];
				else
					m[i][j] = 0;
			}

		return m;
	}


//	template<typename T>
//	TNT::Array2D<T> logSO3(const TNT::Array2D<T> &R, double theta)
//	{
//		Array2D<T> w(3,1);
//		if(abs(theta) > 0)
//		{
//			double sTheta2 = 2.0*sin(theta);
//			w[0][0] = (R[2][1]-R[1][2])/sTheta2;
//			w[1][0] = (R[0][2]-R[2][0])/sTheta2;
//			w[2][0] = (R[1][0]-R[0][1])/sTheta2;
//		}
//		else
//			w = Array2D<T>(3,1,0.0);
//
//		return w;
//	}
}
#endif
