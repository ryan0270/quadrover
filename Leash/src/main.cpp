#pragma warning(disable : 4996)
#include <fstream>
#include <iostream>
#include <sys/stat.h>
#include <exception>

#include <QApplication>

#include "TNT/tnt.h"

#include "ICSL/constants.h"
#include "ICSL/Timer/src/Timer.h"
#include "ICSL/TNT_Utils/TNT_Utils.h"

#include "TelemetryVicon/src/TelemetryVicon.h"

//#include "FlightInterface.h"
#include "Leash.h"

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

	srand(time(NULL));

	QApplication qtApp(argc, argv);
	Leash *leash = new Leash();
	leash->initialize();

	leash->show();
	leash->run();
	qtApp.exec();
	
	string dir = "../runData";
	string filename = "pcData.txt";
	leash->saveLogData(dir, filename);

	delete leash;
	cout << "chad accomplished" << endl;
    return 0;
}
