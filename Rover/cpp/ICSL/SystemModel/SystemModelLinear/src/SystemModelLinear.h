#include <mxml.h>
#include <vector>

#include "TNT/tnt.h"
#include "toadlet/toadlet.h"

#include "ICSL/SystemModel/ISystemModelAffine.h"

#ifndef ICSL_SYSTEMMODEL_LINEAR
#define ICSL_SYSTEMMODEL_LINEAR

namespace ICSL
{
using namespace std;
using namespace TNT;
	class SystemModelLinear : public ISystemModelAffine
	{
		public:
			explicit SystemModelLinear();
			virtual ~SystemModelLinear(){};

			/*!
			 * \return A*x
			 */
			Array2D<double> calcDriftVector(Array2D<double> const &curState);

			/*!
			 * \return B matrix
			 */ 
			Array2D<double> calcActuatorGainMatrix(Array2D<double> const &curState) { return mB; };

			/*!
			 * \return y = C*x+D*u
			 */
			Array2D<double> calcOutput(Array2D<double> const &curState, Array2D<double> const &curActuation);
		
			/*!
			 * \param mat A matrix
			 */
			void setA(Array2D<double> const &mat);

			/*!
			 * \param mat B matrix
			 */
			void setB(Array2D<double> const &mat);

			/*!
			 * \param mat C matrix
			 */
			void setC(Array2D<double> const &mat);

			/*!
			 * \param mat D matrix
			 */
			void setD(Array2D<double> const &mat){mD = mat.copy();};

			const Array2D<double> getA(){return mA;};
			const Array2D<double> getB(){return mB;};
			const Array2D<double> getC(){return mC;};
			const Array2D<double> getD(){return mD;};

			bool loadFromFile(string file);
			int serialize(vector<tbyte> &serial);
			void deserialize(toadlet::egg::Collection<unsigned char> const &serial);

			bool isInitialized(){return mA.dim1()>0 && mB.dim1()>0 && mC.dim1()>0 && mD.dim1()>0;};
		protected:
			Array2D<double> mA, mB, mC, mD;
	};
};

#endif
