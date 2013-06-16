#include <QtGui/QApplication>
#include <QMainWindow>
#include <QMetaType>

#include "quadrotor_config.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include "TelemetryViconDisplay.h"
#include "TelemetryVicon.h"
// #include "Timer/src/Timer.h"

#include "toadlet/egg/Thread.h" 
#include "toadlet/egg/Mutex.h" 
#include "toadlet/egg/WaitCondition.h"
#ifdef HAVE_CONIO_H
	#include <conio.h> // this is a Windows lib
#else // assume I am on Linux
	#include <termios.h> 
#endif

using namespace std;
using namespace Quadrotor;
using namespace ICSL;

struct ThreadSignals
{
	bool close;
};
class Chad : public toadlet::egg::Thread
{
	public:
		toadlet::egg::Mutex *mMutex;
		ThreadSignals *mSignals;
		TelemetryVicon *mTelem;

	protected:
		void run();

};

#define PI 2*acos(0.0)
#define RAD2DEG 180.0/2/acos(0.0)
#define DEG2RAD 2*cos(0.0)/180.0

int getKeypressInstant();
void clearScreen();
void createDir(string dirName);
//void runTelemAcquisition(toadlet::egg::Mutex* mutex, ThreadSignals* signal);

//////////////////////////////////////////////////////////////////////////
/*! 
documenting the main function

\param argv chad
\param argc is bad
\return status

\todo make this function useful
*/
//////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	cout << "start chadding" << endl;

	// Setup Qt framework
	QApplication qtApp(argc, argv);
	QMainWindow qtWin;
	TelemetryViconDisplay display(&qtWin);
	qtWin.setCentralWidget(&display);
	display.show();

	TelemetryVicon telem;
	try
	{
		telem.setRefreshDelay(1000);
		telem.initializeMonitor();
		telem.connect("localhost:801");
		telem.startMonitor();
	}
	catch(TelemetryViconException ex)
	{
		cout << ex.what() << endl;
		getKeypressInstant();
		return -1;
	}
	
	Array2D<double> targetPos(6,1,0.0);
	display.setTargetPos(targetPos);
	qRegisterMetaType<Quadrotor::TelemetryViconDataRecord>("TelemetryViconDataRecord");
	QObject::connect(&telem, SIGNAL(newRecordAvailable(const TelemetryViconDataRecord&)),
			&display, SLOT(populateTable(const TelemetryViconDataRecord&)));
	qtApp.exec();

	return 0;

	toadlet::egg::Mutex mutex_TelemAcquisition;
	ThreadSignals signal_TelemAcquistion;
	signal_TelemAcquistion.close = false;

	Chad chad;
	chad.mMutex = &mutex_TelemAcquisition;
	chad.mSignals = &signal_TelemAcquistion;
	chad.mTelem = &telem;

	chad.start();
//	runTelemAcquisition(&mutex_TelemAcquisition, &signal_TelemAcquistion);
	qtApp.exec();
	getKeypressInstant();
	mutex_TelemAcquisition.lock();
	{
		signal_TelemAcquistion.close = true;
	}
	mutex_TelemAcquisition.unlock();
	chad.join();

    return 0;
}

void Chad::run()
{
// 	try
// 	{
// 		mTelem->startMonitor();
// 
// 		cout << "Sensor data: " << endl;
// 
// 		vector<int> sensorData;
// 		string sensorNames[] = {"x    ", "y    ", "z    ", "roll ", "pitch", "yaw  "};
// 		Timer tmr;
// 
// 		bool stop = false;
// 		mTelem->setHomePosition();
// 		while(!stop)
// 		{
// 			TelemetryViconDataRecord sensorRecord = mTelem->getLatestSensorData();
// 			clearScreen();
// 			cout << "time" << "\t" << sensorRecord.time/1000.0 << endl;
// 			cout << sensorNames[0] << "\t" << sensorRecord.x << endl;
// 			cout << sensorNames[1] << "\t" << sensorRecord.y << endl;
// 			cout << sensorNames[2] << "\t" << sensorRecord.z << endl;
// 			cout << sensorNames[3] << "\t" << sensorRecord.roll*RAD2DEG << endl;
// 			cout << sensorNames[4] << "\t" << sensorRecord.pitch*RAD2DEG << endl;
// 			cout << sensorNames[5] << "\t" << sensorRecord.yaw*RAD2DEG << endl;
// 			cout << "Press any key to stop." << endl;
// 
// 			mMutex->lock();
// 			{
// 				stop = mSignals->close;
// 			}
// 			mMutex->unlock();
// 			tmr.sleepMS(25);
// 		}
// 		mTelem->stopMonitor();
// 	}
// 	catch(TelemetryViconException ex)
// 	{
// 		cout << ex.what() << endl;
// 		mTelem->stopMonitor();
// 	}
}

/*!
 Function to get a single keypress instantaneously (i.e. without waiting
 for the terminal buffer). It does wait indefinitely for a keypress.

 \return ASCII code of first key pressed

 \todo make this work for windows too
 */
int getKeypressInstant()
{
#ifdef ICSL_OS_WIN
	int keypress = (int)_getch();
#else
	struct termios ttystate;

	// disables terminal buffering
	tcgetattr(STDIN_FILENO, &ttystate);
	ttystate.c_lflag &= ~ICANON;
	ttystate.c_lflag &= ~ECHO;
	tcsetattr(STDIN_FILENO, TCSANOW, & ttystate);

	int keypress = cin.get();

	// reenable terminal buffering
	tcgetattr(STDIN_FILENO, &ttystate);
	ttystate.c_lflag |= ICANON;
	ttystate.c_lflag |= ECHO;
	tcsetattr(STDIN_FILENO, TCSANOW, & ttystate);
#endif

	return keypress;
}

/*!
 Function to clear the console screen

 \todo do this in a better way if we are going to stick with consoles
 */
void clearScreen()
{
#ifdef ICSL_OS_WIN
	system("cls");
#else
	system("clear");
#endif
}
