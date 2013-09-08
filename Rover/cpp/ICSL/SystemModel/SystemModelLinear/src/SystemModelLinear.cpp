#include <sstream>

#include "SystemModelLinear.h"

namespace ICSL{
using namespace std;
using namespace TNT;
	SystemModelLinear::SystemModelLinear() : mA(0,0), mB(0,0), mC(0,0), mD(0,0)
	{
	}

	Array2D<double> SystemModelLinear::calcDriftVector(Array2D<double> const &curState)
	{
		if( curState.dim1() != mNumStates )
			Log::alert("SystemModelLiner::calcDriftVector - state size does not match dynamic matrix");
		return matmult(mA, curState); 
	}

	Array2D<double> SystemModelLinear::calcOutput(Array2D<double> const &curState, Array2D<double> const &curActuation)
	{ 
		if( curState.dim1() != mNumStates )
			Log::alert("SystemModelLinear::calcOutput - state size does not match dynamic matrix.");
		if( curActuation.dim1() != mNumInputs )
			Log::alert("SystemModelLiner::calcStateDerivative - actuator size does not match actuator matrix");
		return matmult(mC, curState) + matmult(mD, curActuation); 
	}

	void SystemModelLinear::setA(Array2D<double> const &mat)
	{
		if( mat.dim1() != mat.dim2() )
			Log::alert("SystemModelLinear::setA - Matrix must be square");

		mNumStates = mat.dim2();
		mA = mat.copy();
	}

	void SystemModelLinear::setB(Array2D<double> const &mat)
	{
		if(mat.dim1() != mNumStates)
			Log::alert("SystemModelLinear::setB - row count must equal the number of states.");

		mNumInputs = mat.dim2();
		mB = mat.copy();
	};

	void SystemModelLinear::setC(Array2D<double> const &mat)
	{
		if(mat.dim2() != mNumStates)
			Log::alert("SystemModelLinear::setC - col count must equal the number of states.");

		mNumOutputs = mat.dim1();
		mC = mat.copy();
	};

	bool SystemModelLinear::loadFromFile(string file)
	{
		FILE *fp;
//		mxml_node_t *tree;

		fp = fopen(file.c_str(),"r");
		if(fp == NULL)
			Log::alert(String()+"SystemModelLinear::loadFromFile - File "+file.c_str()+" not found.");
		mxml_node_t *root = mxmlLoadFile(NULL,fp,MXML_TEXT_CALLBACK);
		fclose(fp);

		mxml_node_t *sysNode = mxmlFindElement(root, root, "system", NULL, NULL, MXML_DESCEND);
		mxml_node_t *numInNode = mxmlFindElement(sysNode, sysNode, "numIn", NULL, NULL, MXML_DESCEND);
		mxml_node_t *numOutNode = mxmlFindElement(sysNode, sysNode, "numOut", NULL, NULL, MXML_DESCEND);
		mxml_node_t *numStateNode = mxmlFindElement(sysNode, sysNode, "numState", NULL, NULL, MXML_DESCEND);

		stringstream ssIn(numInNode->child->value.text.string);
		stringstream ssOut(numOutNode->child->value.text.string);
		stringstream ssState(numStateNode->child->value.text.string);

		int numIn, numOut, numState;
		ssIn >> numIn;
		ssOut >> numOut;
		ssState >> numState;

		mxml_node_t *aNode = mxmlFindElement(sysNode, sysNode, "A", NULL, NULL, MXML_DESCEND);
		mxml_node_t *iNode, *jNode;
		Array2D<double> A = Array2D<double>(numState,numState);
		for(int i=0; i<numState; i++)
		{
			stringstream iStream;
			iStream << "r" << i+1;
			iNode = mxmlFindElement(aNode,aNode,iStream.str().c_str(),NULL,NULL,MXML_DESCEND);
			for(int j=0; j<numState; j++)
			{
				stringstream jStream;
				jStream << "c" << j+1;
				jNode = mxmlFindElement(iNode,iNode,jStream.str().c_str(),NULL,NULL,MXML_DESCEND);
				stringstream valStream(jNode->child->value.text.string);
				valStream >> A[i][j];
			}
		}
		setA(A);

		mxml_node_t *bNode = mxmlFindElement(sysNode, sysNode, "B", NULL, NULL, MXML_DESCEND);
		Array2D<double> B = Array2D<double>(numState,numIn);
		for(int i=0; i<numState; i++)
		{
			stringstream iStream;
			iStream << "r" << i+1;
			iNode = mxmlFindElement(bNode,bNode,iStream.str().c_str(),NULL,NULL,MXML_DESCEND);
			for(int j=0; j<numIn; j++)
			{
				stringstream jStream;
				jStream << "c" << j+1;
				jNode = mxmlFindElement(iNode,iNode,jStream.str().c_str(),NULL,NULL,MXML_DESCEND);
				stringstream valStream(jNode->child->value.text.string);
				valStream >> B[i][j];
			}
		}
		setB(B);

		mxml_node_t *cNode = mxmlFindElement(sysNode, sysNode, "C", NULL, NULL, MXML_DESCEND);
		Array2D<double> C = Array2D<double>(numOut,numState);
		for(int i=0; i<numOut; i++)
		{
			stringstream iStream;
			iStream << "r" << i+1;
			iNode = mxmlFindElement(cNode,cNode,iStream.str().c_str(),NULL,NULL,MXML_DESCEND);
			for(int j=0; j<numState; j++)
			{
				stringstream jStream;
				jStream << "c" << j+1;
				jNode = mxmlFindElement(iNode,iNode,jStream.str().c_str(),NULL,NULL,MXML_DESCEND);
				stringstream valStream(jNode->child->value.text.string);
				valStream >> C[i][j];
			}
		}
		setC(C);

		mxml_node_t *dNode = mxmlFindElement(sysNode, sysNode, "D", NULL, NULL, MXML_DESCEND);
		Array2D<double> D = Array2D<double>(numOut,numIn);
		for(int i=0; i<numOut; i++)
		{
			stringstream iStream;
			iStream << "r" << i+1;
			iNode = mxmlFindElement(dNode,dNode,iStream.str().c_str(),NULL,NULL,MXML_DESCEND);
			for(int j=0; j<numIn; j++)
			{
				stringstream jStream;
				jStream << "c" << j+1;
				jNode = mxmlFindElement(iNode,iNode,jStream.str().c_str(),NULL,NULL,MXML_DESCEND);
				stringstream valStream(jNode->child->value.text.string);
				valStream >> D[i][j];
			}
		}
		setD(D);

		return true;
	}

	int SystemModelLinear::serialize(vector<tbyte> &serial)
	{
		uint16 numIn = mB.dim2();
		uint16 numOut = mC.dim1();
		uint16 numState = mA.dim1();

		int size = 0;
		size += sizeof(uint16); // numIn
		size += sizeof(uint16); // numOut
		size += sizeof(uint16); // numState`
		size += numState*numState*sizeof(double); // A
		size += numState*numIn*sizeof(double); // B
		size += numOut*numState*sizeof(double); // C
		size += numOut*numIn*sizeof(double); //D

		serial.resize(size);
		tbyte *p = &(serial[0]);
		memcpy(p,&numIn,sizeof(uint16)); p += sizeof(uint16);
		memcpy(p,&numOut,sizeof(uint16)); p += sizeof(uint16);
		memcpy(p,&numState,sizeof(uint16)); p += sizeof(uint16);

		// TNT documentation says data is stored "like a conventional C multiarray"
		// which I take to mean that the data is contiguous so I can do a straight memcpy
		memcpy(p,&(mA[0][0]),numState*numState*sizeof(double)); p += numState*numState*sizeof(double);
		memcpy(p,&(mB[0][0]),numState*numIn*sizeof(double)); p += numState*numIn*sizeof(double);
		memcpy(p,&(mC[0][0]),numOut*numState*sizeof(double)); p += numOut*numState*sizeof(double);
		memcpy(p,&(mD[0][0]),numOut*numIn*sizeof(double)); p += numOut*numIn*sizeof(double);

		return size;
	}

	void SystemModelLinear::deserialize(toadlet::egg::Collection<unsigned char> const &serial)
	{
		const tbyte *p = &(serial[0]);
		uint16 numIn, numOut, numState;
		memcpy(&numIn, p, sizeof(uint16)); p += sizeof(uint16);
		memcpy(&numOut, p, sizeof(uint16)); p += sizeof(uint16);
		memcpy(&numState, p, sizeof(uint16)); p += sizeof(uint16);

		Array2D<double> A = Array2D<double>(numState,numState);
		Array2D<double> B = Array2D<double>(numState,numIn);
		Array2D<double> C = Array2D<double>(numOut,numState);
		Array2D<double> D = Array2D<double>(numOut,numIn);

		// TNT documentation says data is stored "like a conventional C multiarray"
		// which I take to mean that the data is contiguous so I can do a straight memcpy
		memcpy(&(A[0][0]),p,numState*numState*sizeof(double)); p += numState*numState*sizeof(double);
		memcpy(&(B[0][0]),p,numState*numIn*sizeof(double)); p += numState*numIn*sizeof(double);
		memcpy(&(C[0][0]),p,numOut*numState*sizeof(double)); p += numOut*numState*sizeof(double);
		memcpy(&(D[0][0]),p,numOut*numIn*sizeof(double)); p += numOut*numIn*sizeof(double);

		setA(A);
		setB(B);
		setC(C);
		setD(D);
	}
}
