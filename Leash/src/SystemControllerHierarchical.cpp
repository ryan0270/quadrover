#include "TNT/jama_lu.h"

#include "ICSL/TNT_Utils/TNT_Utils.h"

#include "SystemControllerHierarchical.h"

using namespace ICSL::Constants;

namespace ICSL{
namespace Quadrotor{
	using namespace std;
	using namespace ICSL;
	using namespace TNT;

	SystemControllerHierarchical::SystemControllerHierarchical() :
		mDesiredState(12,1,0.0),
		mAccelRef(3,1,0.0)
	{
		mGainP = mGainYaw = mGainThrust =mGainV =0.0;
//		mDesiredState = Array2D<double>(12,1,0.0);
//		mAccelRef = Array2D<double>(3, 1);
		mAccelRef[0][0] = 0;
		mAccelRef[1][0] = 0;
		mAccelRef[2][0] = GRAVITY;
		mDeltaT = 0;
		mAccelPosLimit = 1;
		mErrorFilterSysFilename = "";
	}

	Array2D<double> SystemControllerHierarchical::calcControl()
	{

		if(mDeltaT <= 0)
			throw(SystemControllerHierarchicalException("Invalid mDeltaT (did you set it?)."));

		mMutex_dataAccess.lock();
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

		while(error[0][0] > 180*DEG2RAD)  error[0][0] -= 360*DEG2RAD;
		while(error[0][0] < -180*DEG2RAD) error[0][0] += 360*DEG2RAD;
		while(error[1][0] > 180*DEG2RAD)  error[1][0] -= 360*DEG2RAD;
		while(error[1][0] < -180*DEG2RAD) error[1][0] += 360*DEG2RAD;
		while(error[2][0] > 180*DEG2RAD)  error[2][0] -= 360*DEG2RAD;
		while(error[2][0] < -180*DEG2RAD) error[2][0] += 360*DEG2RAD;
		
		//Transformation from inertial to body coords
		Array2D<double> rotVicon2Quad = createRotMat(0, PI);
		Array2D<double> rotBody2Inertial = matmult(createRotMat(2, curState[2][0]), matmult(createRotMat(1, curState[1][0]), createRotMat(0, curState[0][0])));
		Array2D<double> posErr = matmult(rotVicon2Quad, submat(error, 6, 8, 0, 0));
		Array2D<double> velErr = matmult(transpose(rotBody2Inertial), matmult(rotVicon2Quad, submat(error, 9, 11, 0, 0)));
		Array2D<double> posErrTot = posErr + mIntSys.getVal().copy();
		Array2D<double> velErrTot = velErr + matmult(transpose(rotBody2Inertial), mIntSys.getValDot().copy());
		Array2D<double> accErrTotInertial =  mAccelRef + integrationLimiter(norm2(posErrTot)*norm2(posErrTot), mAccelPosLimit)*posErrTot + mIntSys.getValDDot();
		Array2D<double> accErrTot = matmult(transpose(rotBody2Inertial), accErrTotInertial);

		double accErrNorm =  norm2(accErrTot);
		double den = (accErrNorm + accErrTot[2][0]) * (accErrNorm + accErrTot[2][0]);
		double ua = accErrTot[2][0] + accErrNorm*mGainThrust*velErrTot[2][0];
		double wx = -accErrNorm*mGainV*velErrTot[1][0] - (accErrNorm*mGainP*accErrTot[1][0])/den;
		double wy = accErrNorm*mGainV*velErrTot[0][0] + (accErrNorm*mGainP*accErrTot[0][0])/den;
		double wz = -mGainYaw*error[2][0];
		
		Array2D<double> u(4, 1);
		double forceScaling = ((SystemModelQuadrotor*)mDynamicModel)->getAvgForceScaling();
		double mass = mDynamicModel->getMass();
		u[0][0] = mDynamicModel->getMass()*ua/forceScaling/4.0; 
		u[1][0] = wx;
		u[2][0] = wy;
		u[3][0] = wz;
		mIntSys.update(posErr, mDeltaT);

		mMutex_dataAccess.unlock();
		return u;
	}

	void SystemControllerHierarchical::setDesiredState(Array2D<double> const &x)
	{
		mMutex_dataAccess.lock();
		mDesiredState = x.copy();
		mMutex_dataAccess.unlock();
	}


	void SystemControllerHierarchical::populateConfigTree(QTreeWidgetItem *root)
	{
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Thrust Gain"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mGainThrust));
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("P Gain"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mGainP));
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("V Gain"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mGainV));
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Yaw Gain"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mGainYaw));
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Int Gain"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mIntSys.getGain()));
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Int Sat Limit"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mIntSys.getSatLimit()));
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Int Pos Limit"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mIntSys.getPosLimit()));
		root->addChild(new QTreeWidgetItem((QTreeWidget*)0,QStringList(QString("Accel Pos Limit"))));
			root->child(root->childCount()-1)->setText(1,QString::number(mAccelPosLimit));

	}

	void SystemControllerHierarchical::applyConfigTree(QTreeWidgetItem *root)
	{
		while(root->childCount() > 0)
		{
			QTreeWidgetItem *item = root->takeChild(0);
			if(item->text(0) == "Thrust Gain")
				mGainThrust = item->text(1).toDouble();
			else if(item->text(0) == "V Gain")
				mGainV = item->text(1).toDouble();
			else if(item->text(0) == "Yaw Gain")
				mGainYaw = item->text(1).toDouble();
			else if(item->text(0) == "P Gain")
				mGainP = item->text(1).toDouble();
			else if(item->text(0) == "Int Gain")
				mIntSys.setGain(item->text(1).toDouble());
			else if(item->text(0) == "Int Sat Limit")
				mIntSys.setSatLimit(item->text(1).toDouble());
			else if(item->text(0) == "Int Pos Limit")
				mIntSys.setPosLimit(item->text(1).toDouble());
			else if(item->text(0) == "Accel Pos Limit")
				mAccelPosLimit = item->text(1).toDouble();
			else
			{
				cout << "Unknown controller config item: " << item->text(0).toUtf8().constData() << endl;;
				throw(SystemControllerHierarchicalException("Unknown controller config item: " + item->text(0).toStdString()));
			}
		}

		mErrorFilterSysFilename = ((SystemModelQuadrotor*)mDynamicModel)->getFilterSysFilename();
		mErrorFilterSys.loadFromFile(mErrorFilterSysFilename.c_str());
		mErrorFilterSys.setCurState(Array2D<double>(mErrorFilterSys.getNumStates(),1,0.0));
		mErrorFilterSys.setCurActuation(Array2D<double>(mErrorFilterSys.getNumActuators(),1,0.0));
	}

	void SystemControllerHierarchical::saveConfig(mxml_node_t *root)
	{
		mxmlNewReal(mxmlNewElement(root, "pGain"), mGainP);
		mxmlNewReal(mxmlNewElement(root, "vGain"), mGainV);
		mxmlNewReal(mxmlNewElement(root, "thrustGain"), mGainThrust);
		mxmlNewReal(mxmlNewElement(root, "yawGain"), mGainYaw);
		mxmlNewReal(mxmlNewElement(root, "intGain"), mIntSys.getGain());
		mxmlNewReal(mxmlNewElement(root, "intSatLimit"), mIntSys.getSatLimit());
		mxmlNewReal(mxmlNewElement(root, "intPosLimit"), mIntSys.getPosLimit());
		mxmlNewReal(mxmlNewElement(root, "accPosLimit"), mAccelPosLimit);
	}

	void SystemControllerHierarchical::loadConfig(mxml_node_t *root)
	{
 		if(root == NULL)
 			return;
 		
		mxml_node_t *pRoot = mxmlFindElement(root,root,"pGain",NULL,NULL,MXML_DESCEND);
		mxml_node_t *vRoot = mxmlFindElement(root,root,"vGain",NULL,NULL,MXML_DESCEND);
		mxml_node_t *thrustRoot = mxmlFindElement(root,root,"thrustGain",NULL,NULL,MXML_DESCEND);
		mxml_node_t *yawRoot = mxmlFindElement(root,root,"yawGain",NULL,NULL,MXML_DESCEND);
		mxml_node_t *posIGainRoot = mxmlFindElement(root,root,"intGain",NULL,NULL,MXML_DESCEND);
		mxml_node_t *intSatLimitRoot = mxmlFindElement(root,root,"intSatLimit",NULL,NULL,MXML_DESCEND);
		mxml_node_t *intPosLimitRoot = mxmlFindElement(root,root,"intPosLimit",NULL,NULL,MXML_DESCEND);
		mxml_node_t *accPosLimitRoot = mxmlFindElement(root,root,"accPosLimit",NULL,NULL,MXML_DESCEND);

		if(pRoot != NULL) mGainP = QString(pRoot->child->value.text.string).toDouble();
		if(vRoot != NULL) mGainV = QString(vRoot->child->value.text.string).toDouble();
		if(thrustRoot != NULL) mGainThrust = QString(thrustRoot->child->value.text.string).toDouble();
		if(yawRoot != NULL) mGainYaw = QString(yawRoot->child->value.text.string).toDouble();
		if(posIGainRoot != NULL) mIntSys.setGain(QString(posIGainRoot->child->value.text.string).toDouble());
		if(intSatLimitRoot != NULL) mIntSys.setSatLimit(QString(intSatLimitRoot->child->value.text.string).toDouble());
		if(intPosLimitRoot != NULL) mIntSys.setPosLimit(QString(intPosLimitRoot->child->value.text.string).toDouble());
		if(accPosLimitRoot != NULL) mAccelPosLimit = QString(accPosLimitRoot->child->value.text.string).toDouble();

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
