#pragma once
#include "../NeuronDataReader.h"
#include "sock.h"

class UDPDataReceiver:public Sock
{
	SOCKET sockfd;

	std::thread* pThread;

	static unsigned tDataRecvThread(void* param);
	void dataProc();

	SocketDataReceived receivedHandel;
public:
	UDPDataReceiver();
	~UDPDataReceiver();

	void SetDataReceivedCallback(SocketDataReceived handle);

	BOOL ListenAt(int nPort);

	int nPort;
	BOOL IsRunning;

	void Stop();

	int Read(char* buff, int buffLen);

	int Write(char* buff, int len);
};

