#pragma once

class ISerialPort;

union UnionFloat
{
    float value;
    char buff[4];
};

enum DeviceType
{
    MCP,
    Golf,
};

enum PackType
{
    Command = 0x01,  // 命令帧
    Response,        // 应答帧
    Result,          // 结果帧
    LongData,        // 长数据帧
    ShortData,       // 短数据帧
    CompressedData,  // 压缩帧
    Warnning,        // 警告帧
    Reserved,        // 保留
};

#pragma warning(disable:4996) //关掉C4996

class Communication
{	
    static char PackLen[10];

	BOOL CheckResponse(BYTE* buff, int len, char* errmsg);
	BOOL CheckResult(BYTE* buff, int len, char* errmsg);
	BOOL CheckLongData(BYTE* buff, int len, char* msg);
	BOOL CheckShortData(BYTE* buff, char* errmsg);
	BOOL CheckWarning(BYTE* buff, char* msg);
	void CheckWarningDetail1(BYTE* buff, char* msg);
	void CheckWarningDetail2(BYTE* buff, char* msg);
	
    void PrintPack(BYTE* buff, int len);
	
public:
    DeviceType DevType;

	InterfaceSerialPort* Comm;

    int ClearComm();

    BOOL WritePack(BYTE* buff, int len);
    BOOL ReadPack(PackType readingPackType, BYTE* buff, char* errmsg);
};

#pragma warning(default : 4996) // 恢复C4996
