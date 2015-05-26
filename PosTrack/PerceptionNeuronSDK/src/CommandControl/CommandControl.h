#ifdef COMMANDCONTROL_EXPORTS
#define COMMANDCONTROL_API __declspec(dllexport)
#else
#define COMMANDCONTROL_API __declspec(dllimport)
#endif

class ISerialPort
{
public:
	virtual int Read(BYTE* buff, int bytesToRead) = 0;
	virtual int Write(BYTE* buff, int bytesToWrite) = 0;
	virtual BOOL IsOpen() = 0;
	virtual int BytesToRead() = 0;
}; 

class COMMANDCONTROL_API CCommandControl
{
	char msg[200];

public:
	CCommandControl(ISerialPort* communicaor);
	
	// 查询主节点版本号
	BOOL QueryReceiverNodeVersion(char* buff, int buffLen);
	// 使传感器进入采集状态
	BOOL StartCapture();
	// 休眠	
	BOOL SensorSleep();
	// 休眠唤醒
	BOOL SensorWakeup();
	// 关闭子节点
	BOOL SensorPowerOff();
	// 修改通讯频率
	BOOL SetFrequentChannel(int freq);
	// 查询通讯频率
	BOOL QueryFrequentChannel(int& result);

    // 进入数据模式
    BOOL SensorToDataMode();

	// 获取操作结果信息
	char* GetMessage();
};

