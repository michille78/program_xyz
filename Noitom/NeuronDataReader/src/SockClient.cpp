#include "stdafx.h"
#include "Error.h"
#include "SockClient.h"
#include "SocketClientsManager.h"
#include "DataParser.h"

SockClient::SockClient(void):
    pThread(NULL)
{
	sock = 0;

	strIP[0] = '0';
	nPort = 0;

	isConnected = FALSE;
	isReading = FALSE;
	receivedHandel = NULL;

	dataParser = new DataParser();
}


SockClient::~SockClient(void)
{
	Disconnect();
	if (dataParser)
	{
		delete dataParser;
		dataParser = NULL;
	}
}

char* SockClient::GetLastMsg()
{
	return lastMsg;
}

BOOL SockClient::IsConnected()
{
	return isConnected;
}

void SockClient::SetDataReceivedCallback(SocketDataReceived handle)
{
	this->receivedHandel = handle;
}

BOOL SockClient::ConnectTo(char* ipAddr, int port)
{
	// Disconnect pre-link
	if(isConnected)
	{
		Disconnect();
	}

	// reset last message buffer
	memset(lastMsg, 0, sizeof(lastMsg));

	// save params
	sprintf_s(strIP, sizeof(strIP), "%s", ipAddr);
	nPort = port;

	if (SocketClientsManager::fnSocketStatusCallback)
    {
        char msg[100];
        sprintf(msg, "Connecting to %s:%d ...\n", strIP, port);
		SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, CS_Starting, msg);
    }

	// 创建套接字
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
#ifdef __OS_XUN__
    if(sock < 0)
    {
        sprintf_s(lastMsg, sizeof(lastMsg), "Create socket failed");
        Error::Instance()->SetLastError(lastMsg);
        
		if(SocketClientsManager::fnSocketStatusCallback)
        {
			SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this,CS_Starting, lastMsg);
        }

        return FALSE;
    }
#else
    if(sock == INVALID_SOCKET)
	{
        sprintf_s(lastMsg, sizeof(lastMsg), "Create socket failed.");
        Error::Instance()->SetLastError(lastMsg);

		if (SocketClientsManager::fnSocketStatusCallback)
        {
			SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, CS_Starting, lastMsg);
        }
        return FALSE;
    }
#endif

	// 发送设置超时
	struct timeval timeout = {30,0}; 
	setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,sizeof(struct timeval));


	// Mac的默认为阻塞函数因此没有设置
#ifndef __OS_XUN__

	// 设置读取为阻塞模式
	unsigned long ul = 0;
	ioctlsocket(sock, FIONBIO, &ul);
#endif
	
    // 建立连接
    struct sockaddr_in serverAddress;
    memset(&serverAddress,0,sizeof(sockaddr_in));

    serverAddress.sin_family=AF_INET; 
    serverAddress.sin_port = htons(port);

	//IP解析赋值
	if (inet_addr(ipAddr) == INADDR_NONE)
	{
		HOSTENT *h = gethostbyname(ipAddr);
		serverAddress.sin_addr.s_addr = *((unsigned long*)h->h_addr);
		//TRACE("IP: %s", inet_ntoa(serverAddress.sin_addr));
	}
	else
	{
		serverAddress.sin_addr.s_addr = inet_addr(ipAddr);
	}

	int ret = connect(sock,(sockaddr*)&serverAddress,sizeof(serverAddress));
    if(ret == SOCKET_ERROR)
	{
        sprintf_s(lastMsg, sizeof(lastMsg), "Failed to connect to '%s:%d'.", strIP, port);
        Error::Instance()->SetLastError(lastMsg);
        
		if (SocketClientsManager::fnSocketStatusCallback)
        {
			SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, CS_Starting, lastMsg);
        }

        return FALSE;
    }

	pThread = new std::thread(tDataRecvThread, this);
	if(!pThread)
	{
		sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to start dispatch thread.");
        Error::Instance()->SetLastError(lastMsg);

		if (SocketClientsManager::fnSocketStatusCallback)
        {
			SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, CS_Starting, lastMsg);
        }

        return FALSE;
	}

	if (SocketClientsManager::fnSocketStatusCallback)
    {
        sprintf_s(lastMsg, sizeof(lastMsg), "Successfully connected to '%s:%d'.", strIP, port);
		SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, CS_Running, lastMsg);
    }

	isConnected = TRUE;

	return TRUE;
}

void SockClient::Disconnect()
{
	isConnected = FALSE;
	isReading = FALSE;

	if (sock)
	{
		//shutdown(sock, SD_BOTH);
		closesocket(sock);

		sock = 0;

		if (SocketClientsManager::fnSocketStatusCallback)
		{
			SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, CS_OffWork, "Service is stoped\n");
		}
	}
	if(pThread)
	{
		Sleep(50);	

		pThread->detach();

        delete pThread;

        pThread = NULL;
	}
}

unsigned SockClient::tDataRecvThread(void* param)
{
	((SockClient*)param)->DataProc();
	

	if (SocketClientsManager::fnSocketStatusCallback)
    {
		SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, param, ((SockClient*)param)->isConnected ? CS_Running : CS_OffWork, "Data thread exited.\n");
    }

	return 0;
}

void SockClient::DataProc()
{
	isReading = TRUE;

	unsigned char buff[40961];
	memset(buff, sizeof(buff), 0);

	int len = sizeof(buff)-1;
	int ret = 0;
	//DWORD err;
    while(isReading)
	{
		ret = Read(buff, len);
		if (ret == 0)
		{
#ifdef __OS_XUN__
			usleep(50000);
#else
			Sleep(50);
#endif
		}
		else if(ret<0)
		{
			sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to read data.");
            Error::Instance()->SetLastError(lastMsg);

			if (SocketClientsManager::fnSocketStatusCallback)
            {
				SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, isConnected ? CS_Running : CS_OffWork, lastMsg);
            }

			break; // break while
		}
		else if (ret <= len && receivedHandel)
		{
			receivedHandel(this, buff, ret);
		} 
    }
}

int SockClient::Read(unsigned char* buff, int buffLen)
{
    // 接收数据
	int ret = recv(sock, (char*)buff, buffLen, 0);
	if(ret==SOCKET_ERROR)
	{
#ifdef __OS_XUN__
        return 0;
#else
		DWORD err = GetLastError();
		switch(err)
		{
		case WSAETIMEDOUT:
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

		return -1;
#endif
	}

	return ret;
}

int SockClient::Write(unsigned char* buff, int len)
{
    // 发送数据
	int sendlen = 0;
	int ret = 0;
	while(sendlen<len)
	{
		ret = send(sock, (char*)buff + sendlen, len - sendlen, 0);
		if(ret==SOCKET_ERROR)
		{
			sprintf_s(lastMsg, sizeof(lastMsg), "%s", "Failed to sned data.");
			Error::Instance()->SetLastError(lastMsg);
        
			if (SocketClientsManager::fnSocketStatusCallback)
            {
				SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, this, isConnected ? CS_Running : CS_OffWork, lastMsg);
            }
			return ret;
		}

		sendlen += ret;
	}
	return sendlen;
}