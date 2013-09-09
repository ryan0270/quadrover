/// This file is for useful matrix operations not included in the
// base TNT Array classes
#ifndef TNT_UTILS_h
#define TNT_UTILS_h

#include "TNT/tnt.h"
#include <cmath>
#include <toadlet/egg.h>

namespace TNT{
using namespace toadlet::egg;

//	template <class T>
//	inline void printTNTArray(Array2D<T> array) 
//	{ 
//		int m = array.dim1(); 
//		int n = array.dim2(); 
//		int i,j; 
//		for(i=0; i<m; i++) 
//		{ 
//			for(j=0; j<n; j++) 
//			{ 
//				std::cout.width(8);
//				std::cout << array[i][j]; 
//				if( j < n-1 ) 
//					std::cout << "\t"; 
//			} 
//			std::cout << std::endl; 
//		} 
//	} 

	template <class T>
	inline Array2D<T> transpose(Array2D<T> const &array)
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
	inline T trace(Array2D<T> const &array)
	{
		assert(array.dim1() == array.dim2());

		T tr = 0;
		for(int i=0; i<array.dim1(); i++)
			tr += array[i][i];

		return tr;
	}

	template <class T1, class T2>
	inline Array2D<T1> scalarMult(T2 scalar, Array2D<T1> const &mat)
	{
		Array2D<T1> result(mat.dim1(), mat.dim2());

		for(int i=0; i<mat.dim1(); i++)
			for(int j=0; j<mat.dim2(); j++)
				result[i][j] = scalar*mat[i][j];

		return result;
	}

	template <class T1, class T2>
	inline Array2D<T1> operator*(T2 scalar, Array2D<T1> const &mat)
	{
		return scalarMult(scalar, mat);
	}

	template <class T1, class T2>
	inline Array2D<T1> operator*(Array2D<T1> const &mat, T2 scalar)
	{
		return scalarMult(scalar, mat);
	}

	/*!
	  requires a 1D vector
	  */
	template <class T>
	inline T norm2(Array2D<T> const &mat)
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

	template <class T>
	inline T matmultS(Array2D<T> const &mat1, Array2D<T> const &mat2)
	{
		assert(mat1.dim1() == 1 && mat2.dim2() == 1);
		return matmult(mat1, mat2)[0][0];
	}


	template <class T>
	inline Array2D<T> submat(Array2D<T> const &mat, int rowStart, int rowStop, int colStart, int colStop)
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
	inline void assignRows(Array2D<T> &mat, int rowStart, int rowStop, Array2D<T> const &rows)
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
	inline void assignColumns(Array2D<T> &mat, int colStart, int colStop, Array2D<T> const &cols)
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

	template <class T>
	inline Array2D<T> stackVertical(Array2D<T> const &matTop, Array2D<T> const &matBottom)
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
	inline Array2D<T> stackVertical(Array2D<T> const &matTop, T val)
	{
		assert(matTop.dim2() == 1);
		//	if(matTop.dim2() != 1)
		//		throw("TNT::stackVertical error - matTop.dim2() != 1");

		return stackVertical(matTop, Array2D<T>(1,1,val));
	}

	template <class T>
	inline Array2D<T> stackHorizontal(Array2D<T> const &matLeft, Array2D<T> const &matRight)
	{
		assert(matLeft.dim1() == matRight.dim1());

		Array2D<T> matNew(matLeft.dim1(), matLeft.dim2()+matRight.dim2());
		assignColumns(matNew, 0, matLeft.dim2()-1, matLeft);
		assignColumns(matNew, matLeft.dim2(), matNew.dim2()-1, matRight);

		return matNew;
	}

	template <class T>
	inline Array2D<T> cross(Array2D<T> const &matLeft, Array2D<T> const &matRight)
	{
		assert(matLeft.dim1() == 3 && matRight.dim1() == 3 && 
				matLeft.dim2() == 1 && matRight.dim2() == 1);

		Array2D<T> matNew(3,1);
		matNew[0][0] = matLeft[1][0]*matRight[2][0]-matLeft[2][0]*matRight[1][0];
		matNew[1][0] = matLeft[2][0]*matRight[0][0]-matLeft[0][0]*matRight[2][0];
		matNew[2][0] = matLeft[0][0]*matRight[1][0]-matLeft[1][0]*matRight[0][0];

		return matNew;
	}

	template <class T>
	inline Array2D<T> extractDiagonal(Array2D<T> const &mat)
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
		rotMat[0][0] = c1*c2; 	rotMat[1][0] = c1*s2*s3-c3*s1;	rotMat[2][0] = s1*s3+c1*c3*s2;
		rotMat[0][1] = c2*s1;	rotMat[1][1] = c1*c3+s1*s2*s3;	rotMat[2][1] = c3*s1*s2-c1*s3;
		rotMat[0][2] = -s2;		rotMat[1][2] = c2*s3;			rotMat[2][2] = c2*c3;

		return rotMat;
	}

	template <class T>
	inline void printArray(String str, TNT::Array2D<T> m)
	{
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
				if(i+1 < m.dim1())
					str = str+"\n";
			}
		}
		Log::alert(str.substr(0, str.length()-1));
	}

	template <class T1, class T2>
	inline TNT::Array2D<T1> blkdiag(TNT::Array2D<T1> const &m1, TNT::Array2D<T2> const &m2)
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

	template<typename T>
	TNT::Array2D<T> logSO3(TNT::Array2D<T> const &R, double theta)
	{
		Array2D<T> w(3,1);
		if(abs(theta) > 0)
		{
			double sTheta2 = 2.0*sin(theta);
			w[0][0] = (R[2][1]-R[1][2])/sTheta2;
			w[1][0] = (R[0][2]-R[2][0])/sTheta2;
			w[2][0] = (R[1][0]-R[0][1])/sTheta2;
		}
		else
			w = Array2D<T>(3,1,0.0);

		return w;
	}
}
#endif
