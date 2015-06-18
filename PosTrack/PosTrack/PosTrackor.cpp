#include "PosTrackor.h"

#include "NatNetConnector.h"
#include "PosTrackCalibration.h"


PosTrackor::PosTrackor(void):
    R(2,2)
{
	printf("Constructing PositionTrackor\n");

	connector = NULL;
	cali = NULL;

	IsCalibrating = FALSE;
	IsCalibrated = FALSE;
    recievePostrackHandle = NULL;
	pOwner = NULL;
	recieveOtherMarkHandle = NULL;
	pOtherMarkOwner = NULL;
    filePos = NULL;
}


PosTrackor::~PosTrackor(void)
{
	IsCalibrating = FALSE;
	IsCalibrated = FALSE;
}

// 得到光学数据之后，调用此函数
//void PosTrackor::DataHandle(MocapData* data)
void PosTrackor::DataHandle(sFrameOfMocapData* data)
{
	if (recieveOtherMarkHandle)
	{
		recieveOtherMarkHandle(pOwner, (float*)data->OtherMarkers, data->nOtherMarkers, data->fLatency, data->Timecode, data->TimecodeSubframe, data->fTimestamp);
	}

	/*
	/// 高鹏
// 	OpticsPosition.X = data->data[0];
// 	OpticsPosition.Y = data->data[1];
// 	OpticsPosition.Z = data->data[2];
    if (data->nRigidBodies)
    {
		OpticsPosition.X = data->RigidBodies[0].x;
		OpticsPosition.Y = data->RigidBodies[0].y;
		OpticsPosition.Z = data->RigidBodies[0].z;

		// 将光学采集到的数据放到PNLib中
		if (recievePostrackHandle)
		{
			recievePostrackHandle(pOwner, OpticsPosition.get());
		}
    }
	// 如果OtherMarkets不为空，则毁掉处理函数
	//if (data->nOtherMarkers)
	//{
		if (recieveOtherMarkHandle)
		{
			recieveOtherMarkHandle(pOwner, (float*)data->OtherMarkers, data->nOtherMarkers);
		}
	//}
	*/
}

BOOL PosTrackor::Init()
{
	Release();

	cali = new PosTrackCalibration();

	return TRUE;
}
BOOL PosTrackor::ConnectTo(char* ip, int port)
{
	if(connector)
	{
		connector->Release();
		delete connector;
	}

	connector = new NatNetConnector();
	connector->SetDataHandler(this);
	return connector->Connect(ip, port);
}

void PosTrackor::Disconnect()
{
	if(connector)
	{
		connector->Release();
		delete connector;
		connector = NULL;
		
		IsCalibrated = FALSE;
	}
}

BOOL PosTrackor::IsEnabled()
{
	return (connector && connector->IsConnected);
}

void PosTrackor::SetToStartCalibration()
{
	ASSERT(cali);

	cali->ClearBuff();
    // 写校准数据
    filePos = fopen("D:\\posCaliData.txt", "w+");

    if (filePos == NULL)
    {
        return ;
    }


	IsCalibrating = TRUE;
	IsCalibrated = FALSE;
}

void PosTrackor::AddCalibrationData(float* inertiaPosition)
{
    Point3D_t position(inertiaPosition[0],inertiaPosition[1],inertiaPosition[2]);
    AddCalibrationData(position);
}

void PosTrackor::AddCalibrationData(Point3D_t inertiaPosition)
{
	ASSERT(cali);

	cali->BufferingData(inertiaPosition, OpticsPosition);
    SYSTEMTIME time;
    GetLocalTime( &time );
    char tmpstr[300];
    sprintf(tmpstr, "%02d-%02d-%02d-%02d   %0.3f %0.3f %0.3f   %0.3f %0.3f %0.3f\n", 
        time.wHour, time.wMinute, time.wSecond, time.wMilliseconds, 
        inertiaPosition.X, inertiaPosition.Y, inertiaPosition.Z,
        OpticsPosition.X, OpticsPosition.Y, OpticsPosition.Z); //  inert X Y Z 
    fwrite(tmpstr, strlen(tmpstr), 1, filePos);
	if(cali->BufferedDataPercent>=1.0)
	{
        if (filePos)
        {
            fclose(filePos);
            filePos = NULL;
        }

        IsCalibrating = FALSE;
        
		// 校准，以获取光学系到惯性系的旋转矢量和平移矢量
		cali->Calculate();
		R      = cali->R;
		Trans2 = cali->Trans2;
		Trans1 = cali->Trans1;
		Trans2 = cali->Trans2;

		// 校准完毕标志
		IsCalibrated = TRUE;
	}
}

void PosTrackor::SetRecievePosTrackHandle(void* pOwner, RecievePosTrackorHandle handle)
{
    this->pOwner = pOwner;
    this->recievePostrackHandle = handle;
}

void PosTrackor::SetRecieveOtherMarkHandle(void* pOwner, RecieveOtherMarkHandle handle)
{
	this->pOtherMarkOwner = pOwner;
	this->recieveOtherMarkHandle = handle;
}

double PosTrackor::GetCalibrationPercent()
{
	return cali->BufferedDataPercent;
}

void PosTrackor::Release()
{
	IsCalibrating = FALSE;
	IsCalibrated = FALSE;

	if(connector)
	{
		connector->Release();
		delete connector;
		connector = NULL;
	}
	if(cali)
	{
		delete cali;
		cali = NULL;
	}
}
