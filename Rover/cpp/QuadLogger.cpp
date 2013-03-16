#include "QuadLogger.h"

namespace ICSL{
namespace Quadrotor{
using namespace std;
QuadLogger::QuadLogger()
{
	mDir = ".";
	mFilename = "quadLog.txt";
	mLogStream = NULL;
	mTypeMask = 0;
	mTypeMask = LOG_FLAG_PC_UPDATES ;
//	mTypeMask |= LOG_FLAG_STATE;
//	mTypeMask |= LOG_FLAG_STATE_DES;
//	mTypeMask |= LOG_FLAG_MOTORS;
//	mTypeMask |= LOG_FLAG_OBSV_UPDATE;
//	mTypeMask |= LOG_FLAG_OBSV_BIAS;
//	mTypeMask |= LOG_FLAG_MAGNOMETER;
	mTypeMask |= LOG_FLAG_ACCEL;
//	mTypeMask |= LOG_FLAG_GYRO;
//	mTypeMask |= LOG_FLAG_PRESSURE;
	mTypeMask |= LOG_FLAG_CAM_RESULTS;
//	mTypeMask |= LOG_FLAG_CAM_IMAGES;
	mTypeMask |= LOG_FLAG_PHONE_TEMP;

	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);

	mRunning = false;;
}

QuadLogger::~QuadLogger()
{
	close();
}

void QuadLogger::addLine(String const &str, uint16 type)
{
	if(mTypeMask & type)
	{
		mMutex_logQueue.lock();
		mLogQueue.push(str);
		mMutex_logQueue.unlock();
	}

//	mMutex_file.lock();
//	if(mTypeMask & type)
//	{
//		String str2= str + "\n";
//		if(mLogStream != NULL)
//			mLogStream->write((tbyte*)str2.c_str(), str2.length());
//	}
//	mMutex_file.unlock();
}

void QuadLogger::run()
{
	mRunning = true;
	{
		String s = String() +"Starting logs at " + mDir+"/"+mFilename;
		Log::alert(s);
	}

	generateMatlabHeader();

	mMutex_file.lock();
	mLogStream = FileStream::ptr(new FileStream(mDir+"/"+mFilename, FileStream::Open_BIT_WRITE));

//	String str = "1\t2\t3\t4\t5\t6\t7\t8\t9\t10\n";
	String str = "";
	for(int i=0; i<20; i++)
		str = str+i+"\t";
	str = str+"\n";
	mLogStream->write((tbyte*)str.c_str(),str.length());

	// for easy indexing while post processing
	str = String() + "0\t-500\n";
	mLogStream->write((tbyte*)str.c_str(),str.length());
	mMutex_file.unlock();

	System sys;
	String line;
	while(mRunning)
	{
		mMutex_logQueue.lock(); int size = mLogQueue.size(); mMutex_logQueue.unlock();
		while(size > 0)
		{
			mMutex_logQueue.lock();
			line = mLogQueue.front();
			mLogQueue.pop();
			size = mLogQueue.size();
			mMutex_logQueue.unlock();

			mMutex_file.lock();
			line = line+"\n";
			if(mLogStream != NULL)
				mLogStream->write((tbyte*)line.c_str(), line.length());
			mMutex_file.unlock();
		}

		sys.msleep(1);
	}

	if(mLogStream != NULL)
	{
		mMutex_file.lock();
		mLogStream->close();
		mMutex_file.unlock();
		mLogStream = NULL;
	}
}

void QuadLogger::close()
{
	mRunning = false;
	this->join();
}

void QuadLogger::saveImageBuffer(list<shared_ptr<SensorDataImage> > const &dataBuffer,
								 list<shared_ptr<ImageMatchData> > const &matchDataBuffer)
{
	list<shared_ptr<SensorDataImage> >::const_iterator dataIter = dataBuffer.begin();
	int id = 0;
	mxml_node_t *xml = mxmlNewXML("1.0");
	mxml_node_t *root = mxmlNewElement(xml,"root");
	mxml_node_t *imgDataNode = mxmlNewElement(root,"ImgData");
	while(dataIter != dataBuffer.end())
	{
		shared_ptr<SensorDataImage> data = static_pointer_cast<SensorDataImage>(*dataIter);
		data->lock();
		shared_ptr<cv::Mat> mat = data->img; 
		String filename = mDir+"/images/img_"+id+".bmp";
		cv::imwrite(filename.c_str(), *mat);

		stringstream ss; ss << "img_" << id;
		mxml_node_t *imgNode = mxmlNewElement(imgDataNode,ss.str().c_str());
			mxmlNewInteger(mxmlNewElement(imgNode,"time"),Time::calcDiffMS(mStartTime, data->timestamp));
			mxml_node_t *attNode = mxmlNewElement(imgNode,"att");
				mxmlNewReal(mxmlNewElement(attNode,"roll"),data->att[0][0]);
				mxmlNewReal(mxmlNewElement(attNode,"pitch"),data->att[1][0]);
				mxmlNewReal(mxmlNewElement(attNode,"yaw"),data->att[2][0]);
			mxml_node_t *startAngularVelNode = mxmlNewElement(imgNode,"startAngularVel");
				mxmlNewReal(mxmlNewElement(startAngularVelNode,"x"),data->startAngularVel[0][0]);
				mxmlNewReal(mxmlNewElement(startAngularVelNode,"y"),data->startAngularVel[1][0]);
				mxmlNewReal(mxmlNewElement(startAngularVelNode,"z"),data->startAngularVel[2][0]);
			mxml_node_t *endAngularVelNode = mxmlNewElement(imgNode,"endAngularVel");
				mxmlNewReal(mxmlNewElement(endAngularVelNode,"x"),data->endAngularVel[0][0]);
				mxmlNewReal(mxmlNewElement(endAngularVelNode,"y"),data->endAngularVel[1][0]);
				mxmlNewReal(mxmlNewElement(endAngularVelNode,"z"),data->endAngularVel[2][0]);
		data->unlock();

		id++;
		dataIter++;
	}
	mxmlNewInteger(mxmlNewElement(imgDataNode,"NumImages"),id);

	list<shared_ptr<ImageMatchData> >::const_iterator matchDataIter = matchDataBuffer.begin();
	mxml_node_t *matchDataNode = mxmlNewElement(root,"MatchData");
	id = 0;
	while(matchDataIter != matchDataBuffer.end())
	{
		stringstream ss; ss << "match_" << id;
		shared_ptr<ImageMatchData> matchData = static_pointer_cast<ImageMatchData>(*matchDataIter);
		matchData->lock();
		mxml_node_t *matchNode = mxmlNewElement(matchDataNode,ss.str().c_str());
			mxmlNewReal(mxmlNewElement(matchNode, "dt"), matchData->dt);
			mxml_node_t *numPtsNode = mxmlNewElement(matchNode,"NumMatches");
			if(matchData->featurePoints.size() == 0 || matchData->featurePoints[0].size() == 0)
				mxmlNewInteger(numPtsNode, 0);
			else
			{
				mxmlNewInteger(numPtsNode, matchData->featurePoints[0].size());
				mxml_node_t *pts0Node = mxmlNewElement(matchNode,"FeaturePoints_prev");
				mxml_node_t *pts1Node = mxmlNewElement(matchNode,"FeaturePoints_cur");
				for(int i=0; i<matchData->featurePoints[0].size(); i++)
				{
					cv::Point2f pt0 = matchData->featurePoints[0][i];
					cv::Point2f pt1 = matchData->featurePoints[1][i];
	
					mxml_node_t *p0Node = mxmlNewElement(pts0Node,"pt0");
					mxml_node_t *p1Node = mxmlNewElement(pts1Node,"pt1");
					mxmlNewReal(mxmlNewElement(p0Node,"x"),pt0.x);
					mxmlNewReal(mxmlNewElement(p0Node,"y"),pt0.y);
					mxmlNewReal(mxmlNewElement(p1Node,"x"),pt1.x);
					mxmlNewReal(mxmlNewElement(p1Node,"y"),pt1.y);
				}
			}
			matchData->unlock();

		id++;
		matchDataIter++;
	}

	FILE *fp = fopen((mDir+"/images/data.xml").c_str(),"w");
	mxmlSaveFile(xml, fp, MXML_NO_CALLBACK);
	fclose(fp);
}

void QuadLogger::generateMatlabHeader()
{
	FileStream::ptr logStream = FileStream::ptr(new FileStream(mDir+"/log_ids.m", FileStream::Open_BIT_WRITE));

	if(!logStream->closed())
	{
		String str;
		str = String()+"LOG_ID_ACCEL="+LOG_ID_ACCEL+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_GYRO="+LOG_ID_GYRO+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MAGNOMETER="+LOG_ID_MAGNOMETER+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_PRESSURE="+LOG_ID_PRESSURE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_PHONE_TEMP="+LOG_ID_PHONE_TEMP+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CPU_USAGE="+LOG_ID_CPU_USAGE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_TIME_SYNC="+LOG_ID_TIME_SYNC+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_GYRO_BIAS="+LOG_ID_GYRO_BIAS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_SET_YAW_ZERO="+LOG_ID_SET_YAW_ZERO+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_ANG_RESET="+LOG_ID_OBSV_ANG_RESET+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_ANG_GAINS_UPDATED="+LOG_ID_OBSV_ANG_GAINS_UPDATED+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OPTIC_FLOW="+LOG_ID_OPTIC_FLOW+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS="+LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OPTIC_FLOW_LS="+LOG_ID_OPTIC_FLOW_LS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_TRANS_ATT_BIAS="+LOG_ID_OBSV_TRANS_ATT_BIAS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_TRANS_FORCE_GAIN="+LOG_ID_OBSV_TRANS_FORCE_GAIN+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_BAROMETER_HEIGHT="+LOG_ID_BAROMETER_HEIGHT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_MOTOR_CMDS="+LOG_ID_MOTOR_CMDS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_DES_ATT="+LOG_ID_DES_ATT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CUR_ATT="+LOG_ID_CUR_ATT+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_DES_TRANS_STATE="+LOG_ID_DES_TRANS_STATE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_CUR_TRANS_STATE="+LOG_ID_CUR_TRANS_STATE+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IMG_PROC_TIME="+LOG_ID_IMG_PROC_TIME+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IBVS_ENABLED="+LOG_ID_IBVS_ENABLED+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_IBVS_DISABLED="+LOG_ID_IBVS_DISABLED+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_RECEIVE_VICON="+LOG_ID_RECEIVE_VICON+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_KALMAN_ERR_COV="+LOG_ID_KALMAN_ERR_COV+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_NUM_FEATURE_POINTS="+LOG_ID_NUM_FEATURE_POINTS+";\n"; logStream->write((tbyte*)str.c_str(),str.length());
		str = String()+"LOG_ID_OBSV_ANG_INNOVATION="+LOG_ID_OBSV_ANG_INNOVATION+";\n"; logStream->write((tbyte*)str.c_str(),str.length());

		logStream->close();
	}
}

}
}
