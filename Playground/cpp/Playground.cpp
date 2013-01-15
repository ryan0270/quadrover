#include "Playground.h"

using namespace std;
using namespace TNT;
using namespace toadlet;
using namespace ICSL::Constants;
using toadlet::uint64;
using toadlet::egg::String;

namespace ICSL {
namespace Quadrotor {
Playground::Playground()
{
	mRunning = false;
	mDone = true;

	mNumCpuCores = 1;
}

Playground::~Playground()
{
}

void Playground::initialize()
{
	mVisionProcessor.setStartTime(mStartTime);
	mVisionProcessor.setQuadLogger(&mQuadLogger);
	mVisionProcessor.start();

	this->start(); // this should spawn a separate thread running the run() function

	Log::alert("Initialized");
}

void Playground::shutdown()
{
	mRunning = false;
//	this->join(); // join doesn't work correctly in NDK
	toadlet::egg::System sys;
	while(!mDone)
	{
		Log::alert("Main waiting");
		sys.msleep(10);
	}


	mVisionProcessor.shutdown();

	Log::alert(String("----------------- really dead -------------"));
}


void Playground::run()
{
	mRunning= true;
	System sys;
	mDone = false;

	Array2D<int> cpuUsagePrev(1,7,0.0), cpuUsageCur(1,7,0.0);
	Time mLastCpuUsageTime;

	while(mRunning) 
	{
		if(mLastCpuUsageTime.getElapsedTimeMS() > 100)
		{
			cpuUsageCur = getCpuUsage();
			mLastCpuUsageTime.setTime();
			if(cpuUsagePrev.dim1() != cpuUsageCur.dim1())
				cpuUsagePrev = Array2D<int>(cpuUsageCur.dim1(), cpuUsageCur.dim2(),0.0);
			double maxTotal= 0;
			Collection<double> usage;
			for(int i=1; i<cpuUsageCur.dim1(); i++)
			{
				if(cpuUsageCur[i][0] == 0 || cpuUsagePrev[i][0] == 0) // this cpu is turned off
					usage.push_back(0);
				else
				{
					double total = 0;
					for(int j=0; j<cpuUsageCur.dim2(); j++)
						total += cpuUsageCur[i][j] - cpuUsagePrev[i][j];
					double used = 0;
					for(int j=0; j<3; j++)
						used += cpuUsageCur[i][j] - cpuUsagePrev[i][j];

					maxTotal = max(maxTotal, total);
					usage.push_back(used/total);
				}
			}
			String str = String()+" "+mStartTime.getElapsedTimeMS()+"\t-2000\t";
			if(cpuUsageCur[0][0] != 0 && cpuUsagePrev[0][0] != 0)
			{
				// overall total has to be handled separately since it only counts cpus that were turned on
				// Assume that the total avaible was 4*maxTotal
				double used = 0;
				for(int j=0; j<3; j++)
					used += cpuUsageCur[0][j]-cpuUsagePrev[0][j];
				str = str+(used/maxTotal/(double)mNumCpuCores)+"\t";

				// finish making log string
				for(int i=0; i<usage.size(); i++)
					str = str+usage[i]+"\t";
				mQuadLogger.addLine(str,PC_UPDATES);
			}
			cpuUsagePrev.inject(cpuUsageCur);
		}

		sys.msleep(10);
	}

	Log::alert(String("----------------- Playground runner dead -------------"));
	mDone = true;
}

void Playground::startLogging(){
	mQuadLogger.setFilename("log.txt");
	mQuadLogger.start();
}

void Playground::stopLogging()
{
	mQuadLogger.close();
}

void Playground::setLogFilename(String name)
{
	mQuadLogger.setFilename(name);
}

void Playground::setLogDir(String dir)
{
	mQuadLogger.setDir(dir);
	Log::alert("Log dir set to: " + mQuadLogger.getFullPath());
}

void Playground::copyImageData(cv::Mat *m)
{
	if(!mRunning) // use this as an indicator that we are shutting down
		return;

	mVisionProcessor.getLastImage(m);
}

// return array stores results obtained from /proc/stat
// cpu	user	nice	system	idle	iowait	irq		softirq
// cpu0	user0	nice0	system0	idle0	iowait0	irq0	softirq0
// cpu1	user1	nice1	system1	idle1	iowait1	irq1	softirq1
// ...
Array2D<int> Playground::getCpuUsage()
{
	Collection<String> lines;
	string line;
	ifstream file("/proc/stat");
	if(file.is_open())
	{
		while(file.good())
		{
			string tok;
			getline(file, line);
			stringstream stream(line);
			stream >> tok;
			if(strncmp("cpu",tok.c_str(),3) == 0) // this is a cpu line
				lines.push_back(String(line.c_str()));
		}
	}
	else
	{
		Log::alert("Failed to open /proc/stat");
	}

	// Note that since mobile phones tend to turn cores off for power, the number of 
	// cpu lines found may not actually match the number of cores on the device
	Array2D<int> data(mNumCpuCores+1, 7,0.0);
	for(int i=0; i<lines.size(); i++)
	{
		String tok;
		int index;
		int tokEnd= lines[i].find(' ');
		tok = lines[i].substr(0,tokEnd); // tok won't include the space
		if(tok.length() < 3)
		{
			Log::alert(String()+"getCpuUsage: token too short -- length = " + tok.length());
			return data;
		}
		else if(tok.length() == 3)
			index = 0;
		else
			index = tok.substr(3,tok.length()-3).toInt32()+1;
		String remainder = lines[i].substr(tokEnd+1,lines[i].length()-tokEnd);
		while(remainder.c_str()[0] == ' ')
			remainder = remainder.substr(1,remainder.length()-1);

		int j=0;
		tokEnd = remainder.find(' '); 
		while(tokEnd != String::npos && j < data.dim2())
		{
			tok = remainder.substr(0,tokEnd); // tok won't include the space
			data[index][j++] = tok.toInt32();
			remainder = remainder.substr(tokEnd+1,remainder.length()-tokEnd);
			while(remainder.c_str()[0] == ' ')
				remainder = remainder.substr(1,remainder.length()-1);
			tokEnd = remainder.find(' ');
		}
	}

	return data;
}

} // namespace Quadrotor
} // namespace ICSL
