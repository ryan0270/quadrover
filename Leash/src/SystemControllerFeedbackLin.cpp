#include "TNT/jama_lu.h"

#include "ICSL/TNT_Utils/TNT_Utils.h"

#include "SystemControllerFeedbackLin.h"

using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{
	using namespace std;
	using namespace ICSL;
	using namespace TNT;

	SystemControllerFeedbackLin::SystemControllerFeedbackLin() :
		mDesiredState(12,1,0.0),
		mDesiredAccel(3,1,0.0),
		mGain(12,1,0.0),
		mGainInt(12,1,0.0),
		mErrInt(12,1,0.0),
		mErrIntLimit(12,1,0.0)
	{
		mDeltaT = 0;
		mUseFilteredStates = false;
	}

	Array2D<double> SystemControllerFeedbackLin::calcControl()
	{
		if(mDeltaT <= 0)
			throw(SystemControllerFeedbackLinException("Invalid mDeltaT (did you set it?)."));

		mMutex_DataAccess.lock();
		Array2D<double> curState, error;
		curState = mDynamicModel->getCurState();
		error = curState-mDesiredState;
		if(mUseFilteredStates && mErrorFilterSys.isInitialized())
		{
			Array2D<double> x = mErrorFilterSys.simulateRK4(error,mDeltaT);

			// use the filtered error instead
			error.inject(matmult(mErrorFilterSys.getC(),x) + matmult(mErrorFilterSys.getD(),error)); 

			// use the filtered state for subsequent calculations
			curState = ((SystemModelQuadrotor*)mDynamicModel)->getCurStateFiltered();
		}

		// restrict angle range
		if(error[0][0] > 180*DEG2RAD)  error[0][0] -= 360*DEG2RAD;
		if(error[0][0] < -180*DEG2RAD) error[0][0] += 360*DEG2RAD;
		if(error[1][0] > 180*DEG2RAD)  error[1][0] -= 360*DEG2RAD;
		if(error[1][0] < -180*DEG2RAD) error[1][0] += 360*DEG2RAD;
		if(error[2][0] > 180*DEG2RAD)  error[2][0] -= 360*DEG2RAD;
		if(error[2][0] < -180*DEG2RAD) error[2][0] += 360*DEG2RAD;
		
		//Transformation from inertial to body coords
		Array2D<double> rotVicon2Quad = createRotMat(0, PI);
		Array2D<double> rotBody2Inertial = matmult(createRotMat(2, curState[2][0]), matmult(createRotMat(1, curState[1][0]), createRotMat(0, curState[0][0])));

		assignRows(error,6,8,matmult(rotVicon2Quad, submat(error, 6, 8, 0, 0)));
		assignRows(error,9,11,matmult(rotVicon2Quad, submat(error, 9, 11, 0, 0)));
//		Array2D<double> posErr = matmult(rotVicon2Quad, submat(error, 6, 8, 0, 0));
//		Array2D<double> velErr = matmult(transpose(rotBody2Inertial), matmult(rotVicon2Quad, submat(error, 9, 11, 0, 0)));

		// Integrators
		for(int i=0; i<mErrInt.dim1(); i++)
			mErrInt[i][0] = constrain(mErrInt[i][0] + mDeltaT*mGainInt[i][0]*error[i][0],-mErrIntLimit[i][0],mErrIntLimit[i][0]);

		// make these explicit so its easier to read
		double kPhi = mGain[0][0];
		double kTheta = mGain[1][0];
		double kPsi = mGain[2][0];
		double kx = mGain[6][0];
		double ky = mGain[7][0];
		double kz = mGain[8][0];
		double kxdot = mGain[9][0];
		double kydot = mGain[10][0];
		double kzdot = mGain[11][0];

		double sPhi = sin(curState[0][0]); double cPhi = cos(curState[0][0]);
		double sTheta = sin(curState[1][0]); double cTheta = cos(curState[1][0]);
		double sPsi = sin(curState[2][0]); double cPsi = cos(curState[2][0]);

		double uT = mDynamicModel->getMass()/cPhi/cTheta*(kz*error[8][0]+kzdot*error[11][0]+mErrInt[8][0]+GRAVITY);
		Array2D<double> Binv(3,3);
		Binv[0][0] = 1; Binv[0][1] = 0; 	Binv[0][2] = -sTheta;
		Binv[1][0] = 0; Binv[1][1] = cPhi; 	Binv[1][2] = sPhi*cTheta;
		Binv[2][0] = 0; Binv[2][1] = -sPhi;	Binv[2][2] = cPhi*cTheta;

		Array2D<double> s(3,1);
		s[0][0] = -kPhi*error[0][0]  +sPsi*(kx*error[6][0]+kxdot*error[9][0]+mErrInt[6][0])-cPsi*(ky*error[7][0]+kydot*error[10][0]+mErrInt[7][0]);
		s[1][0] = -kTheta*error[1][0]+cPsi*(kx*error[6][0]+kxdot*error[9][0]+mErrInt[6][0])+sPsi*(ky*error[7][0]+kydot*error[10][0]+mErrInt[7][0]);
		s[2][0] = -kPsi*error[2][0]-mErrInt[2][0];

		Array2D<double> w = matmult(Binv,s);
		
		Array2D<double> u(4, 1);
		double forceScaling = ((SystemModelQuadrotor*)mDynamicModel)->getAvgForceScaling();
		u[0][0] = uT/forceScaling/4.0;
		u[1][0] = w[0][0];
		u[2][0] = w[1][0];
		u[3][0] = w[2][0];

//		mErrorFilterSysFilename = "";

		mMutex_DataAccess.unlock();

		return u;
	}

	void SystemControllerFeedbackLin::setDesiredState(Array2D<double> const &x)
	{
		mMutex_DataAccess.lock();
		mDesiredState.inject(x);
		mMutex_DataAccess.unlock();
	}

	void SystemControllerFeedbackLin::setDesiredAccel(Array2D<double> const &a)
	{
		mMutex_DataAccess.lock();
		mDesiredAccel.inject(a);
		mMutex_DataAccess.unlock();
	}

	void SystemControllerFeedbackLin::populateConfigTree(QTreeWidgetItem *root)
	{
		QString names[] = {"roll","pitch","yaw","rollRate","pitchRate","yawRate","x","y","z","xVel","yVel","zVel"};

		for(int i=0; i<12; i++)
		{
			if(i == 3) // skip these states
				i = 6;
			root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(names[i])));
				root->child(root->childCount()-1)->setText(1,QString::number(mGain[i][0]));
				root->child(root->childCount()-1)->setText(2,QString::number(mGainInt[i][0]));
				root->child(root->childCount()-1)->setText(3,QString::number(mErrIntLimit[i][0]));
		}
	}

	void SystemControllerFeedbackLin::applyConfigTree(QTreeWidgetItem *root)
	{
		cout << "Applying config" << endl;
		for(int i=0; i<12; i++)
		{
			if(i == 3) // skip these states
				i = 6;
			QTreeWidgetItem *item = root->takeChild(0);
			mGain[i][0] = item->text(1).toDouble();
			mGainInt[i][0] = item->text(2).toDouble();
			mErrIntLimit[i][0] = item->text(3).toDouble();
		}

		mErrorFilterSysFilename = ((SystemModelQuadrotor*)mDynamicModel)->getFilterSysFilename();
		mErrorFilterSys.loadFromFile(mErrorFilterSysFilename.c_str());
		mErrorFilterSys.setCurState(Array2D<double>(mErrorFilterSys.getNumStates(),1,0.0));
		mErrorFilterSys.setCurActuation(Array2D<double>(mErrorFilterSys.getNumActuators(),1,0.0));
	}

	void SystemControllerFeedbackLin::saveConfig(mxml_node_t *root)
	{
		string names[] = {"roll","pitch","yaw","rollRate","pitchRate","yawRate","x","y","z","xVel","yVel","zVel"};
		mxml_node_t *gainRoot = mxmlNewElement(root, "Gain");
		mxml_node_t *gainIntRoot = mxmlNewElement(root, "GainInt");
		mxml_node_t *errIntLimitRoot = mxmlNewElement(root, "ErrIntLimit");
		for(int i=0; i<12; i++)
		{
			if(i == 3) // skip these states
				i = 6;
			mxmlNewReal(mxmlNewElement(gainRoot, names[i].c_str()), mGain[i][0]);
			mxmlNewReal(mxmlNewElement(gainIntRoot, names[i].c_str()), mGainInt[i][0]);
			mxmlNewReal(mxmlNewElement(errIntLimitRoot, names[i].c_str()), mErrIntLimit[i][0]);
		}
	}

	void SystemControllerFeedbackLin::loadConfig(mxml_node_t *root)
	{
 		if(root == NULL)
 			return;

		string names[] = {"roll","pitch","yaw","rollRate","pitchRate","yawRate","x","y","z","xVel","yVel","zVel"};
		mxml_node_t *gainRoot = mxmlFindElement(root,root,"Gain",NULL,NULL,MXML_DESCEND);
		mxml_node_t *gainIntRoot = mxmlFindElement(root,root,"GainInt",NULL,NULL,MXML_DESCEND);
		mxml_node_t *errIntLimitRoot = mxmlFindElement(root,root,"ErrIntLimit",NULL,NULL,MXML_DESCEND);
		for(int i=0; i<12; i++)
		{
			if(i == 3) // skip these states
				i = 6;

			mxml_node_t* gainNode;
			if(gainRoot != NULL) gainNode = mxmlFindElement(gainRoot,gainRoot,names[i].c_str(),NULL,NULL,MXML_DESCEND);
			if(gainNode != NULL) mGain[i][0] = QString(gainNode->child->value.text.string).toDouble();

			mxml_node_t* gainIntNode;
			if(gainIntRoot != NULL) gainIntNode = mxmlFindElement(gainIntRoot,gainIntRoot,names[i].c_str(),NULL,NULL,MXML_DESCEND);
			if(gainIntNode != NULL) mGainInt[i][0] = QString(gainIntNode->child->value.text.string).toDouble();

			mxml_node_t* errIntLimitNode;
			if(errIntLimitRoot != NULL) errIntLimitNode = mxmlFindElement(errIntLimitRoot,errIntLimitRoot,names[i].c_str(),NULL,NULL,MXML_DESCEND);
			if(errIntLimitNode != NULL) mErrIntLimit[i][0] = QString(errIntLimitNode->child->value.text.string).toDouble();
		}

		mErrorFilterSysFilename = ((SystemModelQuadrotor*)mDynamicModel)->getFilterSysFilename();
		try{
			mErrorFilterSys.loadFromFile(mErrorFilterSysFilename.c_str());
			mErrorFilterSys.setCurState(Array2D<double>(mErrorFilterSys.getNumStates(),1,0.0));
			mErrorFilterSys.setCurActuation(Array2D<double>(mErrorFilterSys.getNumActuators(),1,0.0));
			cout << "Error filter loaded: " << mErrorFilterSysFilename <<  endl;
		}
		catch(exception e)
		{	cout << e.what() << endl; }
		catch(...)
		{	cout << "Failed to load error filter: " << mErrorFilterSysFilename << endl; }
	}
}
}
