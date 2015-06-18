#include "stdafx.h"
#include "Error.h"
#include "UDPDataReceiver.h"

#ifndef __OS_XUN__
#include <winsock2.h>
#endif

UDPDataReceiver::UDPDataReceiver():
sockfd(0),
nPort(0),
IsRunning(FALSE),
pThread(NULL),
receivedHandel(NULL)
{
}


UDPDataReceiver::~UDPDataReceiver()
{
}

void UDPDataReceiver::SetDataReceivedCallback(SocketDataReceived handle)
{
	this->receivedHandel = handle;
}

BOOL UDPDataReceiver::ListenAt(int nPort)
{
	// stop pre service
	Stop();

	// 创建套接字
	struct sockaddr_in my_addr;           // 自身的地址信息
	if ((sockfd = ::socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		char lastMsg[100];
		sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to initial UDP service.");

		Error::Instance()->SetLastError(lastMsg);
		return FALSE;
	}

	// 绑定套接字
	memset(&my_addr, 0, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(nPort);      // 网络字节顺序
	my_addr.sin_addr.s_addr = INADDR_ANY; // 自动填本机IP
	if (::bind(sockfd, (struct sockaddr *)&my_addr, sizeof(my_addr)) == -1)
	{
		char lastMsg[100];
		sprintf_s(lastMsg, sizeof(lastMsg), "Failed to bind UDP service to port %d.", nPort);
		Error::Instance()->SetLastError(lastMsg);
		return FALSE;
	}

	pThread = new std::thread(tDataRecvThread, this);
	if (!pThread)
	{
		char lastMsg[100];
		sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to start UDP service.");
		Error::Instance()->SetLastError(lastMsg);
		return FALSE;
	}

	this->nPort = nPort;
	this->IsRunning = TRUE;

	return TRUE;
}

void UDPDataReceiver::Stop()
{
	if (sockfd != 0)
	{
		//shutdown(sock, SD_BOTH);
		closesocket(sockfd);
	}

	this->sockfd = 0;
	this->nPort = 0;
	this->IsRunning = FALSE;
}

unsigned UDPDataReceiver::tDataRecvThread(void* param)
{
	((UDPDataReceiver*)param)->dataProc();
	return 0;
}

void UDPDataReceiver::dataProc()
{
	IsRunning = TRUE;

	unsigned char buff[40961];
	memset(buff, sizeof(buff), 0);

	int len = sizeof(buff) - 1;
	int retval = 0;

	struct sockaddr_in their_addr;        // 连接对方的地址信息

#if __OS_XUN__
#define SOCKLEN uint
#else
#define SOCKLEN int
#endif

	SOCKLEN sin_size = sizeof(their_addr);

	// 主循环
	while (IsRunning)
	{
		retval = recvfrom(sockfd, (char*)buff, len, 0, (struct sockaddr *)&their_addr, &sin_size);
		if (retval <= 0)
		{
			char lastMsg[100];
			sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to read data.");
			Error::Instance()->SetLastError(lastMsg);
			closesocket(sockfd);
			break;
		}

		if (retval <= len && receivedHandel)
		{
			receivedHandel(this, buff, retval);
		}
	}

	IsRunning = FALSE;
}

int UDPDataReceiver::Read(char* buff, int buffLen)
{
#if __OS_XUN__
#define SOCKLEN uint
#else
#define SOCKLEN int
#endif

	SOCKLEN sin_size = 0;
	struct sockaddr_in their_addr;        // 连接对方的地址信息

	// 接收数据
	int ret = recvfrom(sockfd, buff, buffLen, 0, (struct sockaddr *)&their_addr, &sin_size);
	if (ret == SOCKET_ERROR)
	{
		char lastMsg[100];

#ifdef __OS_XUN__
		sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Read data error.");
		Error::Instance()->SetLastError(lastMsg);
		return 0;
#else
		DWORD err = GetLastError();
		switch (err)
		{
		case WSAETIMEDOUT:
			sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Reading data time out.");
			Error::Instance()->SetLastError(lastMsg);
			return 0;

		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
			break;
		}
		sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to receive data.");
		return ret;
#endif
	}

	return ret;
}

int UDPDataReceiver::Write(char* buff, int len)
{
	// 发送数据
	int sendlen = 0;
	int ret = 0;
	while (sendlen<len)
	{
		ret = send(sockfd, buff + sendlen, len - sendlen, 0);
		if (ret == SOCKET_ERROR)
		{
			char lastMsg[100];
			sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to sned data.");
			Error::Instance()->SetLastError(lastMsg);
			return ret;
		}

		sendlen += ret;
	}
	return sendlen;
}
