// CommandControl.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include <stdio.h>

#include "CommandControl.h"
#include "CRC8.h"


SerialControl::SerialControl(InterfaceSerialPort* communicaor)
{
	ASSERT(communicaor);

	memset(msg, '\0', sizeof(msg));

	cmdParser.Comm = communicaor;
}

BOOL SerialControl::SensorWakeup()
{
	ASSERT(cmdParser.Comm);
	if (!cmdParser.Comm->IsOpen())
	{
		sprintf(msg, "%s", "串口未打开");
		return FALSE;
	}

	memset(msg, '\0', sizeof(msg));

	//cmdParser.ClearComm();

	// Wakeup
	BYTE cmd[] = { 0xfd, 0x01, 0x00, 0xa0, 0xb0, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0xfe };
	cmd[11] = CRC8::GetParity(cmd, 11);
	cmdParser.Comm->Write(cmd, sizeof(cmd));

 //   // 读取应答帧
	//BYTE response[200];
 //   if (!cmdParser.ReadPack(Response, response, msg))
 //   {
 //       goto exit;
 //   }

 //   // 读取结果帧
	//BYTE result[13];
 //   if (!cmdParser.ReadPack(Result, response, msg))
 //   {
 //       goto exit;
 //   }

	return TRUE;

exit:
	return FALSE;
}

BOOL SerialControl::StartCapture()
{
	ASSERT(cmdParser.Comm);
	if (!cmdParser.Comm->IsOpen())
	{
		sprintf(msg, "%s", "串口未打开");
		return FALSE;
	}

	memset(msg, '\0', sizeof(msg));


	//cmdParser.ClearComm();

	// 采集
	//BYTE cmd[] = { 0xfd, 0x01, 0x00, 0xa0, 0xb0, 0x01, 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0xfe };
	BYTE cmd[] = { 0xFD, 0x01, 0x00, 0xA0, 0xB0, 0x01, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFE };
	cmd[11] = CRC8::GetParity(cmd, 11);
	cmdParser.Comm->Write(cmd, sizeof(cmd));

    // 读取应答帧
	BYTE response[200];
	if (!cmdParser.ReadPack(Response, response, msg))
    {
        goto exit;
    }

    // 读取结果帧
READRESULT:
	msg[0] = '\0';// result[13];
    if (!cmdParser.ReadPack(Result, response, msg))
    {
        // 失败时有数据帧，报告哪些节点没启动，成功时无数据帧
		BYTE sensors[200];
		if (!cmdParser.ReadPack(LongData, sensors, msg))
		{
			goto exit;
		}
        goto exit;
    }

    // 过滤结果帧
    if (response[4] != 0xA0)
    {
        goto READRESULT;
    }

	return TRUE;

exit:
	return FALSE;
}

// 休眠
BOOL SerialControl::SensorSleep()
{
	ASSERT(cmdParser.Comm);

	memset(msg, '\0', sizeof(msg));

	if (!cmdParser.Comm->IsOpen())
	{
		sprintf(msg, "%s", "串口未打开");
		return FALSE;
	}
	
	// 休眠
	BYTE cmd[] = { 0xFD, 0x01, 0x00, 0xA0, 0xB0, 0x01, 0x03, 0x07, 0x00, 0x00, 0x00, 0x6B, 0xFE};
	cmd[11] = CRC8::GetParity(cmd, 11);
	cmdParser.Comm->Write(cmd, sizeof(cmd));

	return TRUE;
}

// 关闭子节点
BOOL SerialControl::SensorPowerOff()
{
	ASSERT(cmdParser.Comm);

	memset(msg, '\0', sizeof(msg));

	if (!cmdParser.Comm->IsOpen())
	{
		sprintf(msg, "%s", "串口未打开");
		return FALSE;
	}

    // 关闭子节点
	BYTE cmd[] = { 0xfd, 0x01, 0x00, 0xa0, 0xb0, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0xfe };
	cmd[11] = CRC8::GetParity(cmd, 11);
	cmdParser.Comm->Write(cmd, sizeof(cmd));
    
    return TRUE;
}

// 查询主节点版本号
BOOL SerialControl::QueryReceiverNodeVersion(char* buff, int buffLen)
{
	ASSERT(cmdParser.Comm);

	memset(msg, '\0', sizeof(msg));

	if (!cmdParser.Comm->IsOpen())
	{
		sprintf(msg, "%s", "串口未打开");
		return FALSE;
	}

    // 主节点版本查询指令
	BYTE cmd[] = { 0xFD, 0x01, 0x00, 0xA0, 0xB0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE };
	cmd[11] = CRC8::GetParity(cmd, 11);
	cmdParser.Comm->Write(cmd, sizeof(cmd));
			
    // 读取应答帧
	BYTE response[200];
	if (!cmdParser.ReadPack(Response, response, msg))
    {
        goto exit;
    }

    // 读取数据帧
	BYTE result[13];
    if (!cmdParser.ReadPack(LongData, response, msg))
    {
        goto exit;
    }

	sprintf_s(buff, buffLen, "%s", msg);

	return TRUE;

exit:
	return FALSE;
}

// 修改通讯频率
BOOL SerialControl::SetFrequentChannel(int freq)
{
	ASSERT(cmdParser.Comm);

	memset(msg, '\0', sizeof(msg));

	if (!cmdParser.Comm->IsOpen())
	{
		sprintf(msg, "%s", "串口未打开");
		return FALSE;
	}

    // 修改通讯频率
	BYTE cmd[] = { 0xfd, 0x01, 0x00, 0xa0, 0xb0, 0x01, 0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0xfe};
	cmd[8] = (freq-2400);
	cmd[11] = CRC8::GetParity(cmd, 11);
	cmdParser.Comm->Write(cmd, sizeof(cmd));
	
    // 读取应答帧
	BYTE response[200];
	if (!cmdParser.ReadPack(Response, response, msg))
    {
        goto exit;
    }

    // 读取结果帧
READRESULT:
	BYTE result[13];
    if (!cmdParser.ReadPack(Result, response, msg))
    {
        // 失败时有数据帧，报告哪些节点没被修改，成功时无数据帧
		BYTE sensors[200];
		if (!cmdParser.ReadPack(LongData, sensors, msg))
		{
			goto exit;
		}
        goto exit;
    }

    // 过滤结果帧
    if (response[4] != 0xA0)
    {
        goto READRESULT;
    }

	return TRUE;

exit:
	return FALSE;

}

// 进入数据模式
BOOL SerialControl::SensorToDataMode()
{
    ASSERT(cmdParser.Comm);
    memset(msg, '\0', sizeof(msg));

    // 切换到数据模式，中位机开始发数据
    BYTE cmd[] = { 0xFD, 0x01, 0x00, 0xA0, 0xB0, 0x01, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0xFE };
    cmd[11] = CRC8::GetParity(cmd, 11);
    cmdParser.Comm->Write(cmd, sizeof(cmd));
    return TRUE;
}

// 查询通讯频率
BOOL SerialControl::QueryFrequentChannel(int& result)
{
	ASSERT(cmdParser.Comm);

	memset(msg, '\0', sizeof(msg));

	if (!cmdParser.Comm->IsOpen())
	{
		sprintf(msg, "%s", "串口未打开");
		return FALSE;
	}

    // 查询通讯频率
	BYTE cmd[] = { 0xfd, 0x01, 0x00, 0xa0, 0xb0, 0x02, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe};
	cmd[11] = CRC8::GetParity(cmd, 11);
	cmdParser.Comm->Write(cmd, sizeof(cmd));

    // 读取应答帧
	BYTE response[200];
	if (!cmdParser.ReadPack(Response, response, msg))
    {
        goto exit;
    }

    // 读取数据帧
	BYTE dataFrame[200];
    if (!cmdParser.ReadPack(LongData, dataFrame, msg))
    {
        goto exit;
    }

	result = dataFrame[7] + 2400;

	return TRUE;

exit:
	return FALSE;
}

char* SerialControl::GetMessage()
{
	return msg;
}

// 获取版本号，以字符串方式返回每个传感器的当前固件版本
char* GetSensorFirmwareVersion()
{
	return NULL;
}

// 获取版本号，以字符串方式返回每个传感器的当前电量
char* GetPowerLevel()
{

	return NULL;
}
