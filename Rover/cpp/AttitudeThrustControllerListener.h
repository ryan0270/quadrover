#ifndef ATTITUDETHRUSTCONTROLLERLISTENER
#define ATTITUDETHRUSTCONTROLLERLISTENER

// need this separated out to avoid circular head file includes

namespace ICSL {
namespace Quadrotor{
class AttitudeThrustControllerListener
{
	public:
	AttitudeThrustControllerListener(){};
	virtual ~AttitudeThrustControllerListener(){};

	virtual void onAttitudeThrustControllerCmdsSent(double const cmds[4])=0;
};
}}
#endif
