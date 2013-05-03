/// This file is for useful matrix operations not included in the
// base TNT Array classes
#ifndef TNT_UTILS_h
#define TNT_UTILS_h

#include "TNT/tnt.h"
#include <math.h>
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

	inline Array2D<double> scalarMult(double scalar, Array2D<double> const &mat)
	{
		Array2D<double> result(mat.dim1(), mat.dim2());

		for(int i=0; i<mat.dim1(); i++)
			for(int j=0; j<mat.dim2(); j++)
				result[i][j] = scalar*mat[i][j];

		return result;
	}

	inline Array2D<double> scalarMult(Array2D<double> const &mat, double scalar)
	{
		return scalarMult(scalar, mat);
	}

	inline Array2D<double> operator*(double scalar, Array2D<double> const &mat)
	{
		return scalarMult(scalar, mat);
	}

	inline Array2D<double> operator*(Array2D<double> const &mat, double scalar)
	{
		return scalarMult(scalar, mat);
	}

	/*!
	  requires a 1D vector
	  */
	template <class T>
	inline double norm2(Array2D<T> const &mat)
	{
		assert(mat.dim1() == 1 || mat.dim2() == 1);
		double result = 0;
		if(mat.dim1()==1)
			for(int i=0; i<mat.dim2(); i++)
				result += pow(mat[0][i],2);
		else
			for(int i=0; i<mat.dim1(); i++)
				result += pow(mat[i][0],2);

		return sqrt(result);
	}

	inline double matmultS(Array2D<double> const &mat1, Array2D<double> const &mat2)
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
	inline void assignRows(Array2D<double> &mat, int rowStart, int rowStop, Array2D<double> const &rows)
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
	inline void assignRows(Array2D<double> &mat, int rowStart, int rowStop, double val)
	{
		assert(rowStart >= 0 && rowStop < mat.dim1());

		for(int r=rowStart; r<=rowStop; r++)
			for(int c=0; c<mat.dim2(); c++)
				mat[r][c] = val;
	}

	/*!
	  \param mat is modified to have its cols replaces
	  */
	inline void assignColumns(Array2D<double> &mat, int colStart, int colStop, Array2D<double> const &cols)
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
	inline void assignColumns(Array2D<double> &mat, int colStart, int colStop, double val)
	{
		assert(colStart >= 0 && colStop < mat.dim2());

		for(int r=0; r<mat.dim1(); r++)
			for(int c=colStart; c<=colStop; c++)
				mat[r][c] = val;
	}

	inline Array2D<double> stackVertical(Array2D<double> const &matTop, Array2D<double> const &matBottom)
	{
		assert(matTop.dim2() == matBottom.dim2());
		//	if(matTop.dim2() != matBottom.dim2())
		//		throw("TNT::stackVertical error - matrix dimensions do not match");

		Array2D<double> matNew(matTop.dim1()+matBottom.dim1(), matTop.dim2());
		assignRows(matNew,0,matTop.dim1()-1,matTop);
		assignRows(matNew,matTop.dim1(),matNew.dim1()-1,matBottom);

		return matNew;
	}

	inline Array2D<double> stackVertical(Array2D<double> const &matTop, double val)
	{
		assert(matTop.dim2() == 1);
		//	if(matTop.dim2() != 1)
		//		throw("TNT::stackVertical error - matTop.dim2() != 1");

		return stackVertical(matTop, Array2D<double>(1,1,val));
	}

	inline Array2D<double> stackHorizontal(Array2D<double> const &matLeft, Array2D<double> const &matRight)
	{
		assert(matLeft.dim1() == matRight.dim1());

		Array2D<double> matNew(matLeft.dim1(), matLeft.dim2()+matRight.dim2());
		assignColumns(matNew, 0, matLeft.dim2()-1, matLeft);
		assignColumns(matNew, matLeft.dim2(), matNew.dim2()-1, matRight);

		return matNew;
	}

	inline Array2D<double> cross(Array2D<double> const &matLeft, Array2D<double> const &matRight)
	{
		assert(matLeft.dim1() == 3 && matRight.dim1() == 3 && 
				matLeft.dim2() == 1 && matRight.dim2() == 1);

		Array2D<double> matNew(3,1);
		matNew[0][0] = matLeft[1][0]*matRight[2][0]-matLeft[2][0]*matRight[1][0];
		matNew[1][0] = matLeft[2][0]*matRight[0][0]-matLeft[0][0]*matRight[2][0];
		matNew[2][0] = matLeft[0][0]*matRight[1][0]-matLeft[1][0]*matRight[0][0];

		return matNew;
	}

	inline Array2D<double> extractDiagonal(Array2D<double> const &mat)
	{
		assert(mat.dim1() == mat.dim2());

		Array2D<double> matNew(mat.dim1(),1);
		for(int i=0; i<mat.dim1(); i++)
			matNew[i][0] = mat[i][i];

		return matNew;
	}

	inline Array2D<double> createIdentity(int n)
	{
		assert(n > 0);

		Array2D<double> matNew(n,n,0.0);
		for(int i=0; i<n; i++)
			matNew[i][i] = 1.0;

		return matNew;
	}

	template <class T>
	inline Array2D<T> createRotMat(int axis, T angleInRad)
	{
		Array2D<T> rotMat(3,3,0.0);
		double c = cos(angleInRad);
		double s = sin(angleInRad);
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
				str = str+"\n";
			}
		}
		Log::alert(str);
	}
}
#endif
