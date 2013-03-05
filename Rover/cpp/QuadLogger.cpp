#include "QuadLogger.h"

using namespace std;

namespace ICSL{
namespace Quadrotor{
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
//	mTypeMask |= LOG_FLAG_ACCEL;
//	mTypeMask |= LOG_FLAG_GYRO;
//	mTypeMask |= LOG_FLAG_PRESSURE;
	mTypeMask |= LOG_FLAG_CAM_RESULTS;
//	mTypeMask |= LOG_FLAG_CAM_IMAGES;
	mTypeMask |= LOG_FLAG_PHONE_TEMP;
	mPaused = false;
}

QuadLogger::~QuadLogger()
{
	close();
}

void QuadLogger::addLine(String const &str, uint16 type)
{
	if(mPaused)
		return;

	mMutex.lock();
	if(mTypeMask & type)
	{
		String str2= str + "\n";
		if(mLogStream != NULL)
			mLogStream->write((tbyte*)str2.c_str(), str2.length());
	}
	mMutex.unlock();
}
void QuadLogger::start()
{
	{
		String s = String() +"Starting logs at " + mDir+"/"+mFilename;
		Log::alert(s);
	}
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
}

void QuadLogger::close()
{
	if(mLogStream != NULL)
	{
		mLogStream->close();
		mLogStream = NULL;
	}
}

void QuadLogger::saveImageBuffer(list<shared_ptr<SensorDataImage> > const &dataBuffer)
{
	list<shared_ptr<SensorDataImage> >::const_iterator iter = dataBuffer.begin();
	int id = 0;
	mxml_node_t *xml = mxmlNewXML("1.0");
	mxml_node_t *root = mxmlNewElement(xml,"root");
	while(iter != dataBuffer.end())
	{
		shared_ptr<SensorDataImage> data = static_pointer_cast<SensorDataImage>(*iter);
		data->lock();
		shared_ptr<cv::Mat> mat = data->img; 
		String filename = mDir+"/images/img_"+id+".bmp";
		cv::imwrite(filename.c_str(), *mat);

		stringstream ss; ss << "img_" << id;
		mxml_node_t *imgNode = mxmlNewElement(root,ss.str().c_str());
			mxmlNewInteger(mxmlNewElement(imgNode,"time"),Time::calcDiffMS(mStartTime, data->timestamp));
			mxml_node_t *attNode = mxmlNewElement(imgNode,"att");
				mxmlNewReal(mxmlNewElement(attNode,"roll"),data->att[0][0]);
				mxmlNewReal(mxmlNewElement(attNode,"pitch"),data->att[1][0]);
				mxmlNewReal(mxmlNewElement(attNode,"yaw"),data->att[2][0]);
			mxml_node_t *angularVelNode = mxmlNewElement(imgNode,"angularVel");
				mxmlNewReal(mxmlNewElement(angularVelNode,"x"),data->angularVel[0][0]);
				mxmlNewReal(mxmlNewElement(angularVelNode,"y"),data->angularVel[1][0]);
				mxmlNewReal(mxmlNewElement(angularVelNode,"z"),data->angularVel[2][0]);
		data->unlock();

		id++;
		iter++;
	}

	mxmlNewInteger(mxmlNewElement(root,"NumImages"),id);

	FILE *fp = fopen((mDir+"/images/data.xml").c_str(),"w");
	mxmlSaveFile(xml, fp, MXML_NO_CALLBACK);
	fclose(fp);
}

}
}
