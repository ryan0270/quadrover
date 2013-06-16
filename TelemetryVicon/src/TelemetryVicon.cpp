#include "TelemetryVicon.h" 
#include <sstream> 
#include <iostream> 

#include "TNT/jama_lu.h"

#include "ICSL/TNT_Utils/TNT_Utils.h"

namespace ICSL{
namespace Quadrotor{
using namespace std; 
using namespace TNT;
using namespace ICSL;

TelemetryVicon::TelemetryVicon() : mOrigin(3,1,0.0)
{
	mConnected = false; 
	mThreadSignals.beginSuspend = true; 
	mThreadSignals.close = false; 
	mThreadSignals.queueInUse = false; 
	mThreadSignals.setHomePos = true;
	mThreadSignals.startTime = 0;
	mMonitor.parent = this;
	mMonitor.mSleeping = false;
	mMonitor.mInitialized = false;
	mMonitor.mRefreshDelay = 5;
	mMonitor.mPollingDelay = 5;

	mNumQuads = 0;
//	mOrigin = Array2D<double>(3,1,0.0);
} 
 
void TelemetryVicon::initializeMonitor()
{
	mThreadSignals.startTime = mTmr.getCurTimeMS(); 
	mMonitor.start();
	mMutexSuspendReady.lock();
	{		
		mWCSuspendReady.wait(&mMutexSuspendReady);
	}
	mMutexSuspendReady.unlock();
	mVicon.setSimulationMode(false);
	//mVicon.setSimulationMode(true);
	mQueueMaxSize = 10;
}
 
TelemetryVicon::~TelemetryVicon() 
{ 
	if(mConnected)
		disconnect();

	mWCSuspendAwaken.notify();
	mMutexDataAccess.lock();
	{
		mThreadSignals.close = true;
		mThreadSignals.beginSuspend = false;
	}
	mMutexDataAccess.unlock();

	mMonitor.join();
}

bool TelemetryVicon::connect(string source)
{
	if(!mMonitor.mInitialized)
		throw(TelemetryViconException("Vicon Monitor not initialized"));

	mConnected = false;
	bool result = false;

	mVicon.setSourceID(source);
	ConnectionStatus connResult = mVicon.connectToSource();
	if(connResult.getStatus() != ConnectionStatus::Connected)
	{
		cout << endl << "\t!!!!!!!!!!!!!!!!!!WARNING!!!!!!!!!!!!!!!!!!" << endl;
		cout << "\tUnable to connect to Vicon source: " << source << endl;
		cout << "\tSwitching to simulated Vicon" << endl;
		cout << "\t!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
		mVicon.setSimulationMode(true);
//		throw(TelemetryViconException("Unable to connect to Vicon source: " + source));
	}
	else
		result = true;

	mConnected = true;
	return result;
}
 
void TelemetryVicon::disconnect()
{
	if(!mMonitor.mInitialized)
		throw(TelemetryViconException("Vicon Monitor not initialized"));

	mVicon.disconnectFromSource();
	mConnected = false;
}
 
vector<TelemetryViconDataRecord> TelemetryVicon::getLatestSensorData() 
{ 
	if(!mMonitor.mInitialized)
		throw(TelemetryViconException("Vicon Monitor not initialized"));

	vector<TelemetryViconDataRecord> data(mNumQuads);
	for(int i=0; i<mNumQuads; i++)
		data[i].isEmpty = true;

	mMutexDataAccess.lock();
	{
		if(mDataQueue.size() > 0)
			data = mDataQueue.back();
	}
	mMutexDataAccess.unlock();

	for(int i=0; i<(int)data.size(); i++)
	{
		if (data[i].isEmpty)
		{
			data[i].x = 0; data[i].y = 0; data[i].z = 0;
			data[i].roll = 0; data[i].pitch = 0; data[i].yaw = 0;
			data[i].time = 0;
			data[i].isEmpty = true;
		}
	}

	return data; 
} 
 
 
void TelemetryVicon::startMonitor() 
{ 
	if(!mMonitor.mInitialized)
		throw(TelemetryViconException("Vicon Monitor not initialized"));

	mWCSuspendAwaken.notify();
	// I shouldn't need to use the critical section here, but to be safe... 
	mMutexDataAccess.lock(); 
	{ 
		mThreadSignals.beginSuspend = false; 
	} 
	mMutexDataAccess.unlock(); 
}  

void TelemetryVicon::stopMonitor() 
{ 
	if(!mMonitor.mInitialized)
		throw(TelemetryViconException("Vicon Monitor not initialized"));

//	bool suspendReady = false; 
	mMutexDataAccess.lock(); 
	{ 
		mThreadSignals.beginSuspend = true; 
	} 
	mMutexDataAccess.unlock();

	if (mMonitor.mSleeping)
		return;

	mMutexSuspendReady.lock();
	{
		mWCSuspendReady.wait(&mMutexSuspendReady);
	}
	mMutexSuspendReady.unlock();
} 

void TelemetryVicon::syncStartTimeMS(unsigned long time) 
{ 
	if(!mMonitor.mInitialized)
		throw(TelemetryViconException("Vicon Monitor not initialized"));

	mMutexDataAccess.lock();
	{
		mThreadSignals.startTime = time;
	}
	mMutexDataAccess.unlock();

	if (!mMonitor.mSleeping)
	{
		mMutexRecordReady.lock();
		{
			mWCRecordReady.wait(&mMutexRecordReady);
		}
		mMutexRecordReady.unlock();
	}
} 

void TelemetryVicon::setOriginPosition(Array2D<double> const &origin)
{
	mOrigin.inject(origin);
//	if(!mMonitor.mInitialized)
//		throw(TelemetryViconException("Vicon Monitor not initialized"));

//	mMutexDataAccess.lock();
//	{
//		mThreadSignals.setOrigin = true;
//	}
//	mMutexDataAccess.unlock();

//	if (!mMonitor.mSleeping)
//	{
//		mMutexHomePosSet.lock();
//		{
//			mWCHomePosSet.wait(&mMutexHomePosSet);
//		}
//		mMutexHomePosSet.unlock();
//	}
}

void TelemetryVicon::addTrackedQuadrotor(string name)
{
	vector<string> markerNames;
	markerNames.push_back("x_head");
	markerNames.push_back("x_tail");
	markerNames.push_back("y_head");
	markerNames.push_back("y_tail");
	markerNames.push_back("body1");
	mVicon.addTrackingObject(name, markerNames, 4);
	mNumQuads++;
}

/////////////////////////////// private functions ///////////////////////////////////////////// 
void TelemetryVicon::Monitor::run() 
{ 
	bool suspend = false; 
	bool close = false; 
	vector<bool> noHomePos;
	Timer tmr; 

	vector<Array2D<double> > A;
	Array2D<double> origin(3,1);
	vector<JAMA::LU<double> > luList;
	vector<string> nameList;

	Array2D<double> rot180X(3,3,0.0); // to account for vicon axes being rotated relative to mine
	rot180X[0][0] = 1; rot180X[1][1] = -1; rot180X[2][2] = -1;

	tmr.sleepMS(100); // hack to try make sure the parent has time to get to the wait point before I notify

	unsigned long refreshStartTime = 0; // this should force a refresh the first time through
	vector<TelemetryViconDataRecord> oldRecords;
	//	oldRecord.isEmpty = true;
	while(true) 
	{ 
		parent->mMutexDataAccess.lock(); 
		{ 
			suspend = parent->mThreadSignals.beginSuspend; 
		} 
		parent->mMutexDataAccess.unlock(); 
		if( suspend ) 
		{ 			
			parent->mWCSuspendReady.notify();

			mSleeping = true;
			mInitialized = true;
			parent->mMutexSuspendAwaken.lock();
			{
				parent->mWCSuspendAwaken.wait(&parent->mMutexSuspendAwaken);
			}
			parent->mMutexSuspendAwaken.unlock();

			mSleeping = false;
		} 

		// Not suspended so keep doing what you were doing 

		parent->mMutexDataAccess.lock(); 
		{ 
			close = parent->mThreadSignals.close; 
		} 
		parent->mMutexDataAccess.unlock(); 
		if(close) 
			break; 

		vector<TrackedObject> trackedObjects;
		try
		{
			trackedObjects= parent->mVicon.updateMeasurements2();
		}
		catch(char* exStr)
		{
			cout << "vicon monitor debug: a" << endl;
			QMessageBox(QMessageBox::Critical, "Vicon Measurement Error", exStr);
		}

		// quadID is the local index in the current set of found objects
		// quadGlobalIndex is the index in globally stored varaibles (such as origin)
		vector<TelemetryViconDataRecord> newRecords(trackedObjects.size());
		for(int quadID=0; quadID<(int)newRecords.size(); quadID++)
		{
//			A.push_back(Array2D<double>(3,5));
			bool nameFound = false;
			int quadGlobalIndex = -1;
			for(int j=0; j<(int)nameList.size(); j++)
				if(strcmp(trackedObjects[quadID].name.c_str(),nameList[j].c_str())==0)
				{
					quadGlobalIndex = j;
					nameFound = true;
				}
			if(!nameFound)
			{
				nameList.push_back(trackedObjects[quadID].name);
				noHomePos.push_back(true);
				Array2D<double> temp(3,5,0.0);
				luList.push_back(JAMA::LU<double>(temp));
				quadGlobalIndex = (int)nameList.size()-1;
			}

			newRecords[quadID].objectID = trackedObjects[quadID].name;

			try
			{
//				TrackedObject *trackObj = parent->mVicon.getTrackedObject(quadID);
				Array2D<double> xHeadPos, yHeadPos, xTailPos, yTailPos, bodyPos;
				int occludedCnt = 0;
				for(int i=0; i<(int)trackedObjects[quadID].markers.size(); i++)
					occludedCnt += trackedObjects[quadID].markers[i].isOccluded ? 1 : 0;
				if(occludedCnt > 1)
				{
					cout << trackedObjects[quadID].name << endl;
					cout << "vicon monitor debug: " << occludedCnt << " occluded markers " << endl;
					throw(TelemetryViconException("Too many occluded markers"));
				}

				xHeadPos = 1.0/1000.0*trackedObjects[quadID].markers.at(0).position;
				xTailPos = 1.0/1000.0*trackedObjects[quadID].markers.at(1).position;
				yHeadPos = 1.0/1000.0*trackedObjects[quadID].markers.at(2).position;
				yTailPos = 1.0/1000.0*trackedObjects[quadID].markers.at(3).position;
				bodyPos = 1.0/1000.0*trackedObjects[quadID].markers.at(4).position;

//				parent->mMutexDataAccess.lock();
				{
					if(noHomePos[quadID])// || parent->mThreadSignals.setHomePos)
					{
						Array2D<double> home(3,1);
						home[0][0] = 1.0/2.0*(xHeadPos[0][0]+xTailPos[0][0]);
						home[1][0] = 1.0/2.0*(xHeadPos[1][0]+xTailPos[1][0]);
						home[2][0] = 1.0/2.0*(xHeadPos[2][0]+xTailPos[2][0]);
						Array2D<double> xHat = 1/norm2(xHeadPos-xTailPos)*(xHeadPos-xTailPos);
						//					xHat[2][0] = 0; // Set z to be flat
						Array2D<double> rot90(3,3,0.0);
						rot90[0][1] = 1; rot90[1][0] = -1; rot90[2][2] = 1;
						Array2D<double> yHat = matmult(rot90, xHat);
						Array2D<double> zHat = cross(xHat, yHat);
						Array2D<double> xHeadHome(3,1), xTailHome(3,1), yHeadHome(3,1), yTailHome(3,1), bodyHome(3,1);
						xHeadHome[0][0] = matmultS(transpose(xHeadPos-home),xHat);
						xHeadHome[1][0] = matmultS(transpose(xHeadPos-home),yHat);
						xHeadHome[2][0] = matmultS(transpose(xHeadPos-home),zHat);
						xTailHome[0][0] = matmultS(transpose(xTailPos-home),xHat);
						xTailHome[1][0] = matmultS(transpose(xTailPos-home),yHat);
						xTailHome[2][0] = matmultS(transpose(xTailPos-home),zHat);
						yHeadHome[0][0] = matmultS(transpose(yHeadPos-home),xHat);
						yHeadHome[1][0] = matmultS(transpose(yHeadPos-home),yHat);
						yHeadHome[2][0] = matmultS(transpose(yHeadPos-home),zHat);
						yTailHome[0][0] = matmultS(transpose(yTailPos-home),xHat);
						yTailHome[1][0] = matmultS(transpose(yTailPos-home),yHat);
						yTailHome[2][0] = matmultS(transpose(yTailPos-home),zHat);
						bodyHome[0][0] = matmultS(transpose(bodyPos-home),xHat);
						bodyHome[1][0] = matmultS(transpose(bodyPos-home),yHat);
						bodyHome[2][0] = matmultS(transpose(bodyPos-home),zHat);

						//Array2D<double> chad1(3,3);
						//assignColumns(chad1,0,0,xHat);
						//assignColumns(chad1,1,1,yHat);
						//assignColumns(chad1,2,2,zHat);
						//printTNTArray(chad1);
						//cout << endl << "---------------------------" << endl << endl;
						//printTNTArray(rot90);
						//cout << endl << "---------------------------" << endl << endl;
						//Array2D<double> chad(3,5);
						//assignColumns(chad,0,0,xHeadPos);
						//assignColumns(chad,1,1,xTailPos);
						//assignColumns(chad,2,2,yHeadPos);
						//assignColumns(chad,3,3,yTailPos);
						//assignColumns(chad,4,4,bodyPos);
						//printTNTArray(chad);
						//cout << endl << "---------------------------" << endl << endl;
						//assignColumns(chad,0,0,xHeadHome);
						//assignColumns(chad,1,1,xTailHome);
						//assignColumns(chad,2,2,yHeadHome);
						//assignColumns(chad,3,3,yTailHome);
						//assignColumns(chad,4,4,bodyHome);
						//printTNTArray(chad);

						// Rotate things down to a flat plane
						double pitch = atan2(xHeadHome[2][0]-xTailHome[2][0],xHeadHome[0][0]-xTailHome[0][0]);
						double roll = atan((yTailHome[2][0]-yHeadHome[2][0])/(yTailHome[1][0]-yHeadHome[1][0])); // my z and y axes are reversed from vicon
						Array2D<double> rotPitch(3,3,0.0), rotRoll(3,3,0.0);
						rotPitch[0][0] = cos(pitch); rotPitch[0][2] = -sin(pitch);
						rotPitch[1][1] = 1;
						rotPitch[2][0] = sin(pitch); rotPitch[2][2] = cos(pitch);
						rotRoll[0][0] = 1;
						rotRoll[1][1] = cos(roll); rotRoll[1][2] = -sin(roll);
						rotRoll[2][1] = sin(roll); rotRoll[2][2] = cos(roll);
						xHeadHome = matmult(transpose(rotRoll),matmult(transpose(rotPitch),xHeadHome));
						xTailHome = matmult(transpose(rotRoll),matmult(transpose(rotPitch),xTailHome));
						yHeadHome = matmult(transpose(rotRoll),matmult(transpose(rotPitch),yHeadHome));
						yTailHome = matmult(transpose(rotRoll),matmult(transpose(rotPitch),yTailHome));
						bodyHome = matmult(transpose(rotRoll),matmult(transpose(rotPitch),bodyHome));

						///					origin = 0.5*(xHeadPos+xTailPos);
						origin[0][0] = 0.0;
						origin[1][0] = 0.0;
						origin[2][0] = 0.0;

						//cout << endl << "---------------------------" << endl << endl;
						//printTNTArray(rotPitch);
						//cout << endl << "---------------------------" << endl << endl;
						//printTNTArray(rotRoll);
						//cout << endl << "---------------------------" << endl << endl;
						//assignColumns(chad,0,0,xHeadHome);
						//assignColumns(chad,1,1,xTailHome);
						//assignColumns(chad,2,2,yHeadHome);
						//assignColumns(chad,3,3,yTailHome);
						//assignColumns(chad,4,4,bodyHome);
						//printTNTArray(chad);

//						assignColumns(homePos[quadID],0,0,home+xHeadHome);
//						assignColumns(homePos[quadID],1,1,home+xTailHome);
//						assignColumns(homePos[quadID],2,2,home+yHeadHome);
//						assignColumns(homePos[quadID],3,3,home+yTailHome);
//						assignColumns(homePos[quadID],4,4,home+bodyHome);

						Array2D<double> A(3,5,0.0);
						assignColumns(A,0,0,xHeadHome-xTailHome);
						assignColumns(A,1,1,xTailHome-yHeadHome);
						assignColumns(A,2,2,yHeadHome-yTailHome);
						assignColumns(A,3,3,yTailHome-bodyHome);
						assignColumns(A,4,4,bodyHome-xHeadHome);

//						lu.push_back(JAMA::LU<double>(transpose(A[quadID])));
						luList[quadGlobalIndex] = JAMA::LU<double>(transpose(A));
						if(!luList[quadGlobalIndex].isNonsingular())
						{
							cout << "Issue trying to set Vicon home position. Matrix is singular." << endl;
							continue;
						}

						noHomePos[quadID] = false;
						parent->mWCHomePosSet.notify();
					}
					parent->mThreadSignals.setHomePos = false;
				}
				origin.inject(parent->mOrigin);
//				parent->mMutexDataAccess.unlock();

				newRecords[quadID].x = 0.5*(xHeadPos[0][0]+xTailPos[0][0])-origin[0][0];
				newRecords[quadID].y = 0.5*(xHeadPos[1][0]+xTailPos[1][0])-origin[1][0];
				newRecords[quadID].z = 0.5*(xHeadPos[2][0]+xTailPos[2][0])-origin[2][0];
//				printTNTArray(transpose(xTailPos));
//				toadlet::egg::Logger::alert(String()+xTailPos[0][0]+"\t"+xTailPos[1][0]+"\t"+xTailPos[2][0]);

				Array2D<double> B(3,5);
				assignColumns(B,0,0,xHeadPos-xTailPos);
				assignColumns(B,1,1,xTailPos-yHeadPos);
				assignColumns(B,2,2,yHeadPos-yTailPos);
				assignColumns(B,3,3,yTailPos-bodyPos);
				assignColumns(B,4,4,bodyPos-xHeadPos);


				B = matmult(rot180X,B);

				Array2D<double> R;
				R = transpose(luList[quadGlobalIndex].solve(transpose(B)));

				newRecords[quadID].roll = atan2(R[2][1], R[2][2]);
				newRecords[quadID].pitch = atan2(-R[2][0], sqrt(pow(R[0][0],2)+pow(R[1][0],2)));
				newRecords[quadID].yaw = atan2(R[1][0], R[0][0]);
				/*if(abs(newRecords[quadID].yaw) > 0.1)
					cout << newRecords[quadID].yaw << endl;*/

//				while(oldRecords.size() < newRecords.size())
//					oldRecords.push_back(newRecords[oldRecords.size()]);
//				if(oldRecords[quadID].isEmpty)
//					oldRecords[quadID] = newRecords[quadID];

				// see if we had the same object the last time around
				int lastRecIndex = -1;
				for(int i=0; i<(int)oldRecords.size(); i++)
					if(strcmp(newRecords[quadID].objectID.c_str(),oldRecords[i].objectID.c_str())==0)
						lastRecIndex = i;

				// make sure it hasn't changed too much
				if(lastRecIndex < 0 || // if it wasn't there last time assume it's good this time
					(abs(newRecords[quadID].roll -oldRecords[lastRecIndex].roll) < 1 &&
					 abs(newRecords[quadID].pitch-oldRecords[lastRecIndex].pitch) < 1 &&
					 abs(newRecords[quadID].yaw	-oldRecords[lastRecIndex].yaw) < 1 &&
					 abs(newRecords[quadID].x	-oldRecords[lastRecIndex].x) < 50 &&
					 abs(newRecords[quadID].y	-oldRecords[lastRecIndex].y) < 50 &&
					 abs(newRecords[quadID].z	-oldRecords[lastRecIndex].z) < 50))
				{
					newRecords[quadID].invalidRead = false;
				}
				else
					newRecords[quadID].invalidRead = true;
				newRecords[quadID].isEmpty = false;
			}
			catch(...)
			{
				cout << "vicon monitor debug: c" << endl;
				newRecords[quadID].invalidRead = true;
				newRecords[quadID].isEmpty = true;
//				luList.push_back(JAMA::LU<double>(transpose(A[quadID])));
			}
			newRecords[quadID].time = tmr.getCurTimeMS()-parent->mThreadSignals.startTime; 
			oldRecords.clear();
			for(int i=0; i<(int)newRecords.size(); i++)
				oldRecords.push_back(newRecords[i]);
//			if(quadID == 0)
//				newRecords[quadID].objectID = "quad_v2";
//			else
//				newRecords[quadID].objectID = "quad_v3";
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////

		// push a record on the back to tell simulated quads to update
		TelemetryViconDataRecord rec;
		rec.objectID= "sims";
		rec.x = rec.y = rec.z = 0;
		rec.roll = rec.pitch = rec.yaw = 0;
		newRecords.push_back(rec);

		parent->mMutexDataAccess.lock();
		{ 
			parent->mThreadSignals.queueInUse = true; 
			parent->mDataQueue.push(newRecords); 
			while((int)parent->mDataQueue.size() > parent->mQueueMaxSize) 
				parent->mDataQueue.pop(); 
			parent->mThreadSignals.queueInUse = false; 
			parent->mWCRecordReady.notify();
			if(tmr.getCurTimeMS()-refreshStartTime > mRefreshDelay)
			{
				for(int i=0; i<(int)newRecords.size(); i++)
				{
					for(int j=0; j<(int)parent->mListeners.size(); j++)
						parent->mListeners[j]->onTelemetryUpdated(newRecords[i]);
//					emit parent->newRecordAvailable( newRecords[i] );
//					cout << "Emitting " << newRecords[i].objectID << endl;
				}
				refreshStartTime = tmr.getCurTimeMS();
			}
		} 
		parent->mMutexDataAccess.unlock(); 

		// Vicon itself only updates at 100Hz
//		tmr.sleepMS(mPollingDelay);
		tmr.sleepMS(1);
	}
} 
}
}
