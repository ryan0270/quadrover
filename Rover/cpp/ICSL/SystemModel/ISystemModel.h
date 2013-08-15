#ifndef CLASS_SYSTEMMODEL
#define CLASS_SYSTEMMODEL

#include <string>

#include "toadlet/egg/Mutex.h"
#include "TNT/tnt.h"

#include "TNT_Utils.h"

using namespace std;
using namespace TNT;

/*
 Abstract class to define objects that represent system models (standard dynamics, SVR, GPR, etc)
*/
namespace ICSL {
class ISystemModel
{
	public:
	explicit ISystemModel(){};
	ISystemModel(int numStates, int numAct) : mCurState(numStates,1,0.0), mCurActuator(numAct,1,0.0){};
	virtual ~ISystemModel(){};

	virtual void setCurState(Array2D<double> state){mMutex_DataAccess.lock(); mCurState = state.copy(); mMutex_DataAccess.unlock();}
	virtual void setCurActuation(Array2D<double> control)
	{
		mMutex_DataAccess.lock();
		mCurActuator = control.copy();
		mMutex_DataAccess.unlock();
	};
	virtual void setName(string s){mName = s;}
	virtual double getMass(){return mMass;}
	virtual int getNumStates(){return mNumStates;}
	virtual int getNumInputs(){return mNumInputs;}
	virtual int getNumOutputs(){return mNumOutputs;}
	virtual string getName(){return mName;};
	virtual Array2D<double> getCurState()
	{
		mMutex_DataAccess.lock();
		Array2D<double> temp = mCurState.copy();
		mMutex_DataAccess.unlock();
		return temp;
	};
	virtual Array2D<double> const getCurActuator(){return mCurActuator;};

	/*!
	 \param curState
	 \param curActuation
	 \return the system state derivative evaluated at curState and curActuation
	 */
	virtual Array2D<double> calcStateDerivative(Array2D<double> const &curState, Array2D<double> const &curActuation)=0;

	/*!
	 * \return The state after dt seconds
	 */
	virtual const Array2D<double> simulateEuler(Array2D<double> const &curActuation, double dt)
	{
		Array2D<double> dx(mNumStates,1);
		mMutex_DataAccess.lock();		
		dx.inject(calcStateDerivative(mCurState, curActuation));
		mCurState += dt*dx;
		Array2D<double> tempState = mCurState.copy();
		mMutex_DataAccess.unlock();
		return tempState;
	}

	virtual const Array2D<double> simulateRK2(Array2D<double> const &curActuation, double dt)
	{
		mMutex_DataAccess.lock();
		Array2D<double> dx1(mNumStates,1), dx2(mNumStates,1);
		dx1.inject(dt*calcStateDerivative(mCurState, curActuation));
		dx2.inject(dt*calcStateDerivative(mCurState+0.5*dx1, curActuation));
		mCurState = dx2;
		Array2D<double> tempState = mCurState.copy();
		mMutex_DataAccess.unlock();
		return tempState;
	}

	virtual const Array2D<double> simulateRK3(Array2D<double> const &curActuation, double dt)
	{
		mMutex_DataAccess.lock();
		Array2D<double> dx1(mNumStates,1), dx2(mNumStates,1), dx3(mNumStates,1);
		dx1.inject(dt*calcStateDerivative(mCurState, curActuation));
		dx2.inject(dt*calcStateDerivative(mCurState+0.5*dx1, curActuation));
		dx3.inject(dt*calcStateDerivative(mCurState-dx1+2*dx2, curActuation));
		mCurState += 1.0/6.0*(dx1+4.0*dx2+dx3);
		Array2D<double> tempState = mCurState.copy();
		mMutex_DataAccess.unlock();
		return tempState;
	}

	virtual const Array2D<double> simulateRK4(Array2D<double> const &curActuation, double dt)
	{
		mMutex_DataAccess.lock();
		Array2D<double> dx1(mNumStates,1), dx2(mNumStates,1), dx3(mNumStates,1), dx4(mNumStates,1);
		dx1.inject(dt*calcStateDerivative(mCurState, curActuation));
		dx2.inject(dt*calcStateDerivative(mCurState+0.5*dx1, curActuation));
		dx3.inject(dt*calcStateDerivative(mCurState+0.5*dx2, curActuation));
		dx4.inject(dt*calcStateDerivative(mCurState+dx3, curActuation));
		mCurState += 1.0/6.0*(dx1+2.0*dx2+2.0*dx3+dx4);
		Array2D<double> tempState = mCurState.copy();
		mMutex_DataAccess.unlock();

		return tempState;
	}

	virtual void reset()
	{
		mMutex_DataAccess.lock();
		for(int i=0; i<mCurState.dim1(); i++)
			mCurState[i][0] = 0;
		for(int i=0; i<mCurActuator.dim1(); i++)
			mCurActuator[i][0] = 0;
		mMutex_DataAccess.unlock();
	}

    protected:
	int mNumStates, mNumInputs, mNumOutputs;
	double mMass;
	string mName;
	Array2D<double> mCurState, mCurActuator;//, mLastStateDeriv;
	toadlet::egg::Mutex mMutex_DataAccess;
};
}
#endif
