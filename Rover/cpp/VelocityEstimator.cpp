#include "VelocityEstimator.h"

namespace ICSL {
namespace Quadrotor {
using namespace std;
using namespace TNT;

VelocityEstimator::VelocityEstimator() : 
	mRotPhoneToCam(3,3,0.0),
	mRotCamToPhone(3,3,0.0)
{
	mDone = true;
	mRunning = false;

	mNewImageDataAvailable = false;
}

VelocityEstimator::~VelocityEstimator()
{
}

void VelocityEstimator::shutdown()
{
	Log::alert("------------------------- VelocityEstimator shutdown started");
	mRunning = false;
	this->join();
	Log::alert("------------------------- VelocityEstimator shutdown done");
}

void VelocityEstimator::initialize()
{
	mScheduler = SCHED_NORMAL;
	mThreadPriority = sched_get_priority_min(SCHED_NORMAL);
}

void VelocityEstimator::run()
{
	mDone = false;
	mRunning = true;

	sched_param sp;
	sp.sched_priority = mThreadPriority;
	sched_setscheduler(0, mScheduler, &sp);

	shared_ptr<ImageMatchData> imgMatchData;
	while(mRunning)
	{
		if(mNewImageDataAvailable)
		{
			mMutex_imageData.lock();
			imgMatchData = mImageMatchData;
			mMutex_imageData.unlock();

			doVelocityEstimate(imgMatchData);

			mNewImageDataAvailable = false;
		}

		System::msleep(1);
	}

	mDone = true;
}

void VelocityEstimator::onImageProcessed(shared_ptr<ImageMatchData> const data)
{
	mMutex_imageData.lock();
	mImageMatchData = data;	
	mMutex_imageData.unlock();

	mNewImageDataAvailable = true;
}

// See eqn 98 in the Feb 25, 2013 notes
void VelocityEstimator::doVelocityEstimate(shared_ptr<ImageMatchData> const &matchData)
{
	if(matchData->featurePoints[0].size() < 5)
	{
		String str = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW_INSUFFICIENT_POINTS+"\t";
		mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);
		return;
	}

	double dt = Time::calcDiffNS( matchData->imgData0->timestamp, matchData->imgData1->timestamp)/1.0e9;
	if(dt < 1e-3)
		return;

//	mMutex_data.lock();
	Array2D<double> Sn = 300*300*createIdentity((double)2);
	Array2D<double> SnInv(2,2,0.0);
	SnInv[0][0] = 1.0/Sn[0][0]; SnInv[1][1] = 1.0/Sn[1][1];

	Array2D<double> state = mObsvTranslational->estimateStateAtTime(matchData->imgData0->timestamp);
	Array2D<double> errCov = mObsvTranslational->estimateErrCovAtTime(matchData->imgData0->timestamp);
	Array2D<double> mu_v1 = submat(state,3,5,0,0);
	Array2D<double> Sv = submat(errCov,3,5,3,5);
	JAMA::LU<double> SvLU(Sv);
	Array2D<double> SvInv1 = SvLU.solve(createIdentity((double)3));

//	mOpticFlowVel.timestamp.setTime(matchData->imgData1->timestamp);
	Time img1Time = matchData->imgData1->timestamp;
//	double z = IData::interpolate(img1Time, mHeightDataBuffer);
//	z -= 0.060; // offset between markers and camera
	
	double z = max(state[2][0]-0.060,0.150);

	// Rotate prior velocity information to camera coords
	Array2D<double> mu_v = matmult(mRotPhoneToCam, mu_v1);
	Array2D<double> SvInv = matmult(mRotPhoneToCam, matmult(SvInv1, mRotCamToPhone));

//	mMutex_data.unlock();
//
	Array2D<double> A(1,3,0.0);
	Array2D<double> B(3,3,0.0);
	Array2D<double> R1 = createRotMat_ZYX(matchData->imgData0->att[2][0], matchData->imgData0->att[1][0], matchData->imgData0->att[0][0]);
	Array2D<double> R2 = createRotMat_ZYX(matchData->imgData1->att[2][0], matchData->imgData1->att[1][0], matchData->imgData1->att[0][0]);
	Array2D<double> attChange = matmult(transpose(R2), R1);
	Array2D<double> omega = matmult( mRotPhoneToCam, logSO3(attChange, dt));
	Array2D<double> q1a(3,1), q2a(3,1);
	Array2D<double> q1(2,1), q2(2,1);
	Array2D<double> Lv(2,3), Lv1(2,3), Lv2(2,3);
	Array2D<double> Lw(2,3), Lw1(2,3), Lw2(2,3);
	double f1= matchData->imgData0->focalLength;
	double f2 = matchData->imgData1->focalLength;
	double f1Inv = 1.0/f1;
	double f2Inv = 1.0/f2;
	double cx = matchData->imgData0->img->cols/2;
	double cy = matchData->imgData0->img->rows/2;
	double x1, y1, x2, y2;
	for(int i=0; i<matchData->featurePoints[0].size(); i++)
	{
		q1[0][0] = matchData->featurePoints[0][i].x-cx;
		q1[1][0] = matchData->featurePoints[0][i].y-cy;
		q2[0][0] = matchData->featurePoints[1][i].x-cx;
		q2[1][0] = matchData->featurePoints[1][i].y-cy;

		x1 = q1[0][0]; y1 = q1[1][0];
		x2 = q2[0][0]; y2 = q2[1][0];

		// Velocity jacobian
		Lv1[0][0] = -f1; 	Lv1[0][1] = 0; 		Lv1[0][2] = x1;
		Lv1[1][0] = 0; 		Lv1[1][1] = -f1; 	Lv1[1][2] = y1;

		Lv2[0][0] = -f2; 	Lv2[0][1] = 0; 		Lv2[0][2] = x2;
		Lv2[1][0] = 0; 		Lv2[1][1] = -f2; 	Lv2[1][2] = y2;

		Lv.inject(Lv1);

		Lw1[0][0] = f1Inv*x1*y1; 		Lw1[0][1] = -(f1+f1Inv*x1*x1); 	Lw1[0][2] = y1;
		Lw1[1][0] = f1+f1Inv*y1*y1;		Lw1[1][1] = -f1Inv*x1*y1;		Lw1[1][2] = -x1;
		
		Lw2[0][0] = f2Inv*x2*y2; 		Lw1[0][1] = -(f2+f2Inv*x2*x2); 	Lw1[0][2] = y2;
		Lw2[1][0] = f2+f2Inv*y2*y2;		Lw1[1][1] = -f2Inv*x2*y2;		Lw1[1][2] = -x2;
		
		Lw.inject(Lw1);


		A += matmult(transpose(q2-q1-matmult(Lw,omega)),matmult(SnInv, Lv));
//		A += matmult(transpose(q2-q1),matmult(SnInv, Lv));
		B += matmult(transpose(Lv), matmult(SnInv, Lv));
	}
	int maxPoints = 50;
	int numPoints = matchData->featurePoints[0].size();
	double scale = min((double)maxPoints, (double)numPoints)/numPoints;
	A = scale*A;
	B = scale*B;
	Array2D<double> temp1 = (dt/z)*A+matmult(transpose(mu_v), SvInv);
	Array2D<double> temp2 = ((dt*dt)/(z*z))*B+SvInv;
	JAMA::LU<double> temp2_TQR(transpose(temp2));
	Array2D<double> vel1 = temp2_TQR.solve(transpose(temp1));

	JAMA::LU<double> B_TLU(transpose(B));
	Array2D<double> velLS1 = z/dt*B_TLU.solve(transpose(A)); // least squares

	// Finally, convert the velocity from camera to phone coords
	Array2D<double> vel = matmult(mRotCamToPhone, vel1);
	Array2D<double> velLS = matmult(mRotCamToPhone, velLS1);

	if(vel.dim1() == 3)
	{
		String str = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW+"\t";
		for(int i=0; i<vel.dim1(); i++)
			str = str+vel[i][0]+"\t";
		str = str+matchData->imgData0->timestamp.getElapsedTimeMS()+"\t";
		mQuadLogger->addLine(str,LOG_FLAG_CAM_RESULTS);

		shared_ptr<DataVector<double> > velData(new DataVector<double>());
		velData->type = DATA_TYPE_OPTIC_FLOW_VEL;
		velData->timestamp.setTime(img1Time);
		velData->data = vel.copy();

		for(int i=0; i<mListeners.size(); i++)
			mListeners[i]->onVelocityEstimator_newEstimate(velData);
	}
	else
	{
		Log::alert("Why is the optical flow vel corrupted?");
		Log::alert("++++++++++++++++++++++++++++++++++++++++++++++++++");
		Log::alert(String()+"dt: "+dt);
		Log::alert(String()+"z: "+z);
		printArray("A:\n",A);
		printArray("B:\n",B);
		printArray("SvInv:\n",SvInv);
		printArray("temp1:\n",temp1);
		printArray("temp2:\n",temp2);
		printArray("mu_v:\n",mu_v);
		printArray("vel1:\n",vel1);
		printArray("vel:\n",vel);
	}

	String str2 = String()+mStartTime.getElapsedTimeMS() + "\t"+LOG_ID_OPTIC_FLOW_LS+"\t";
	for(int i=0; i<velLS.dim1(); i++)
		str2 = str2+velLS[i][0]+"\t";
	mQuadLogger->addLine(str2,LOG_FLAG_CAM_RESULTS);

printArray("vel:\t",vel);
}

} // namespace Rover
} // namespace ICSL
