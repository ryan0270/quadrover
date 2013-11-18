#ifndef CLASS_SYSTEMMODEL
#define CLASS_SYSTEMMODEL

#include <toadlet/toadlet.h>
#include <TNT/tnt.h>


using namespace TNT;

/*
 Abstract class to define objects that represent system models (standard dynamics, SVR, GPR, etc)
*/
namespace ICSL {
class ISystemModel
{
	public:
	explicit ISystemModel(){};
	virtual ~ISystemModel(){};

	virtual void setCurState(Array2D<double> state){mCurState = state.copy();}
	virtual void setCurActuation(Array2D<double> control)
	{
		mMutex_DataAccess.lock();
		mCurActuator = control.copy();
		mMutex_DataAccess.unlock();
	};
	virtual void setName(String s){mName = s;}
	virtual double getMass(){return mMass;}
	virtual int getNumStates(){return mNumStates;}
	virtual int getNumInputs(){return mNumInputs;}
	virtual int getNumOutputs(){return mNumOutputs;}
	virtual String getName(){return mName;};
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
		mMutex_DataAccess.lock();
		Array2D<double> curState = mCurState.copy();
		mMutex_DataAccess.unlock();

		Array2D<double> dx = calcStateDerivative(curState, curActuation);
		curState += dt*dx;

		mMutex_DataAccess.lock();
		mCurState.inject(curState);
		mCurActuator.inject(curActuation);
		mMutex_DataAccess.unlock();
		return curState;
	}

	virtual const Array2D<double> simulateRK2(Array2D<double> const &curActuation, double dt)
	{
		mMutex_DataAccess.lock();
		Array2D<double> curState = mCurState.copy();
		mMutex_DataAccess.unlock();
		Array2D<double> dx1 = dt*calcStateDerivative(curState, curActuation);
		Array2D<double> dx2 = dt*calcStateDerivative(curState+0.5*dx1, curActuation);
		curState = dx2;

		mMutex_DataAccess.lock();
		mCurState.inject(curState);
		mCurActuator.inject(curActuation);
		mMutex_DataAccess.unlock();

		return curState;
	}

	virtual const Array2D<double> simulateRK3(Array2D<double> const &curActuation, double dt)
	{
		mMutex_DataAccess.lock();
		Array2D<double> curState = mCurState.copy();
		mMutex_DataAccess.unlock();
		Array2D<double> dx1 = dt*calcStateDerivative(curState, curActuation);
		Array2D<double> dx2 = dt*calcStateDerivative(curState+0.5*dx1, curActuation);
		Array2D<double> dx3 = dt*calcStateDerivative(curState-dx1+2.0*dx2, curActuation);
		curState += 1.0/6.0*(dx1+4.0*dx2+dx3);

		mMutex_DataAccess.lock();
		mCurState.inject(curState);
		mCurActuator.inject(curActuation);
		mMutex_DataAccess.unlock();

		return curState;
	}

	virtual const Array2D<double> simulateRK4(Array2D<double> const &curActuation, double dt)
	{
		
		mMutex_DataAccess.lock();
		Array2D<double> curState = mCurState.copy();
		mMutex_DataAccess.unlock();
		Array2D<double> dx1 = dt*calcStateDerivative(curState, curActuation);
		Array2D<double> dx2 = dt*calcStateDerivative(curState+0.5*dx1, curActuation);
		Array2D<double> dx3 = dt*calcStateDerivative(curState+0.5*dx2, curActuation);
		Array2D<double> dx4 = dt*calcStateDerivative(curState+dx3, curActuation);
		curState += 1.0/6.0*(dx1+2.0*dx2+2.0*dx3+dx4);

		mMutex_DataAccess.lock();
		mCurState.inject(curState);
		mCurActuator.inject(curActuation);
		mMutex_DataAccess.unlock();

		return curState;
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
	String mName;
	Array2D<double> mCurState, mCurActuator;//, mLastStateDeriv;
	toadlet::egg::Mutex mMutex_DataAccess;
};
}
#endif
