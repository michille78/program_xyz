#pragma once

#include "../NeuronDataReader.h"
#include "sock.h"
#include "IDataParseDelegate.hpp"

class SockClient:public Sock
{
private:
#ifdef __OS_XUN__
    int sock;
#else
	SOCKET sock;
#endif
    
    std::thread* pThread;
	static unsigned tDataRecvThread(void* param);
	void DataProc();

	BOOL isConnected;
	BOOL isReading;
	
	char lastMsg[300];

	SocketDataReceived receivedHandel;

public:
	SockClient(void);
	~SockClient(void);

	char strIP[50];
	int nPort;

	// read and write socket
	int Read(unsigned char* buff, int buffLen);
	int Write(unsigned char* buff, int len);

	char* GetLastMsg();

	void SetDataReceivedCallback(SocketDataReceived handle);

	BOOL ConnectTo(char* ipAddr, int port);
	void Disconnect();

	BOOL IsConnected();
};

