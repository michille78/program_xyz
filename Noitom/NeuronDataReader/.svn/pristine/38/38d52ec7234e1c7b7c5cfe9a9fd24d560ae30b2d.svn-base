#pragma once

#include "IDataParseDelegate.hpp"

union VERSION
{
	UINT32 _version;
	struct
	{
		UCHAR BuildNumb;     // Build number
		UCHAR Revision;      // Revision number
		UCHAR Minor;         // Subversion number
		UCHAR Major;         // Major version number
	};
};

class SockClient;
class UDPDataReceiver;
class SocketClientsManager
{
private:
	SocketClientsManager();
	~SocketClientsManager();

	static void __stdcall _fnSocketDataReceived(void* clientObj, unsigned char* data, int len);

	vector<SockClient*> sockClients;
	vector<UDPDataReceiver*> udpReceivers;



public:
	static void*               customedObject;
	static FrameDataReceived   fnBvhDataCallback;

    static void*               customedOptDataObject;
    static OptDataReceived     fnOptDataCallback;

	static void*               customedCommandDataObject;
	static CommandDataReceived fnCommandDataCallback;

	static void*               socketStatusCustomedObject;
	static SocketStatusChanged fnSocketStatusCallback;

public:
	static SocketClientsManager* Instance;

	void SetBVHFrameDataCallback(void* customObj, FrameDataReceived fn)
	{
		SocketClientsManager::customedObject = customObj;
		SocketClientsManager::fnBvhDataCallback = fn;
	}; 

    void SetOptDataCallback(void* customObj, OptDataReceived fn)
    {
        SocketClientsManager::customedOptDataObject = customObj;
        SocketClientsManager::fnOptDataCallback = fn;
    };
	
	void SetCommandDataCallback(void* customObj, CommandDataReceived handle)
	{
		SocketClientsManager::customedCommandDataObject = customObj;
		SocketClientsManager::fnCommandDataCallback = handle;
	}

	void SetSocketStatusCallback(void* customObj, SocketStatusChanged fn)
	{
		SocketClientsManager::socketStatusCustomedObject = customObj;
		SocketClientsManager::fnSocketStatusCallback = fn;
	};

	//=========================TCP Client================================
	// 创建一个客户端
	SOCKET_REF RegisterTCPClient(char* serverIP, int nPort);

	BOOL FetchAvatarDataFromServer(SOCKET_REF sockRef, int avatarIndex, CmdId cmdId);

	BOOL FetchDataFromServer(SOCKET_REF sockRef, CmdId cmdId);

	BOOL AutoSyncCommands(SOCKET_REF sockRef, CmdId cmdId, bool isRegister);
	// Check status
	SocketStatus GetSocketStatus(SOCKET_REF sockRef);

	void Disconnect(SOCKET_REF connectorRef);

	void DisconnectAllClients();
	//===================================================================


	//=========================TCP Client================================
	// Start a UDP service to receive data at 'nPort'
	SOCKET_REF StartUDPServiceAt(int nPort);

	void StopAllUDPService();
	//===================================================================


	//=========================Version====================================
	void GetVersion(UINT32* version)
	{
		VERSION vs;
		vs.Major = 1;
		vs.Minor = 0;
		vs.Revision = 0;
		vs.BuildNumb = 2;
		*version = vs._version;
	}

};

