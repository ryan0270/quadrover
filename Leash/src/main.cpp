#pragma warning(disable : 4996)
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <exception>

#include <QApplication>

#include "TNT/tnt.h"

//#include "C:/Advantech/DAQNavi/Inc/bdaqctrl.h"
#include "ICSL/constants.h"
#include "ICSL/ControlTimer/src/ControlTimer.h"
#include "ICSL/Timer/src/Timer.h"
#include "ICSL/TNT_Utils//TNT_Utils.h"

#include "Quadrotor/quadrotor_config.h"
#include "Quadrotor/TelemetryVicon/src/TelemetryVicon.h"

#include "FlightInterface.h"

using namespace std;
using namespace ICSL;
using namespace ICSL::Quadrotor;

int getKeypressInstant();
void clearScreen();
void doEmergencyShutdown();

/*! 
 documenting the main function

 \param argv chad
 \param argc is bad
 \return status

 \todo make this function useful
*/
int main(int argc, char **argv)
{
	cout << "start chadding" << endl;
	double deltaT = 0.1;

	srand(time(NULL));

	QApplication qtApp(argc, argv);
	FlightInterface *flightInterface;
	flightInterface = new FlightInterface();
	flightInterface->initialize();
	flightInterface->setDeltaT(deltaT);

	ControlTimer cntlTimer;
	cntlTimer.setSelectedDevice(0);
	cntlTimer.setChannel(0);
	cntlTimer.setFrequency(1.0/deltaT);
	cntlTimer.addListener(flightInterface);
	cntlTimer.setEnabled(true);

	flightInterface->show();
	flightInterface->run();
	qtApp.exec();
	
	cntlTimer.setEnabled(false);
	string dir = "../runData";
	string filename = "data.csv";
	flightInterface->saveData(dir, filename);;

	delete flightInterface;
	cout << "chad accomplished" << endl;
    return 0;
}
