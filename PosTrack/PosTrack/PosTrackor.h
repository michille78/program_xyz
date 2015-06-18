#ifndef POSTRACKOR_H
#define POSTRACKOR_H

#ifdef POSTRACK_EXPORTS
#define POSTRACK_API __declspec(dllexport)
#else
#define POSTRACK_API __declspec(dllimport)
#endif

#include "NatNetDataHandler.h"
#include "Definitions.h"
#include "Eigen"
using namespace Eigen;

class NatNetConnector;
class PosTrackCalibration;

typedef void (CALLBACK* RecievePosTrackorHandle)(void * pOwner, float* data);
typedef void (CALLBACK* RecieveOtherMarkHandle)(void * pOwner, float* otherMarkData, int nOtherMarkers, float fLatency, unsigned int Timecode, unsigned int TimecodeSubframe, double fTimestamp);
class POSTRACK_API PosTrackor : public NatNetDataHandler
{
	RecievePosTrackorHandle recievePostrackHandle;
	void* pOwner;
	RecieveOtherMarkHandle recieveOtherMarkHandle;
	void* pOtherMarkOwner;

	NatNetConnector* connector;
	PosTrackCalibration* cali;
	
	Point3D_t CalibratedPos;

	// 光学系统测得的实时位置
	Point3D_t OpticsPosition;

	//void DataHandle(RigidBodyData* data);
	void DataHandle(sFrameOfMocapData* data);
	
public:
	PosTrackor(void);
	~PosTrackor(void);

	//////////////////////////////////////////////////////////////////////////
    FILE* filePos;
	BOOL Init();
	BOOL ConnectTo(char* ip, int port);
	void Disconnect();
	BOOL IsEnabled();
	void Release();
	
	BOOL IsCalibrating;
	BOOL IsCalibrated;
	void SetToStartCalibration();    
	void SetRecievePosTrackHandle(void* pOwner, RecievePosTrackorHandle handle);
	void SetRecieveOtherMarkHandle(void* pOwner, RecieveOtherMarkHandle handle); 
	// 将这个函数的地址赋值，在另一个地方给这个函数定义。当前类中只提供定义
	//////////////////////////////////////////////////////////////////////////


    //////////////////////////////////////////////////////////////////////////
	void AddCalibrationData(Point3D_t inertiaPosition);    
    // add光学测试
    void AddCalibrationData(float* inertiaPosition);

    double GetCalibrationPercent();	

	// 惯性系归零的平移矢量
	Point3D_t Trans1;

	// 光学系到惯性系的平移矢量
	Point3D_t Trans2;

	// 光学系统转换到惯性系统的旋转矩阵
	MatrixXd R;
	//////////////////////////////////////////////////////////////////////////
};

#endif
