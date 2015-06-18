// NeuronDataReader.cpp : 定义 DLL 应用程序的导出函数。
//

#include "stdafx.h"
#include "../NeuronDataReader.h"

#include "SockClient.h"
#include "UDPDataReceiver.h"
#include "Error.h"
#include "SocketClientsManager.h"

#include "ParsingStringData.h"
#include "DataParser.h"

// Initialize library
BDR_API void BRRegisterFrameDataCallback(void* customedObj, FrameDataReceived handle)
{
	SocketClientsManager::Instance->SetBVHFrameDataCallback(customedObj, handle);
}

// Register command data callback
BDR_API void BRRegisterCommandDataCallback(void* customedObj, CommandDataReceived handle)
{
	SocketClientsManager::Instance->SetCommandDataCallback(customedObj, handle);
}

// Register socket status callback
BDR_API void BRRegisterSocketStatusCallback(void* customedObj, SocketStatusChanged handle)
{
	SocketClientsManager::Instance->SetSocketStatusCallback(customedObj, handle);
}

// Register receiving and parsed frame opt data callback
BDR_API void BRRegisterOptDataCallback(void* customedObj, OptDataReceived handle)
{
    SocketClientsManager::Instance->SetOptDataCallback(customedObj, handle);
}

// Connect to server
BDR_API SOCKET_REF BRConnectTo(char* serverIP, int nPort)
{
	return SocketClientsManager::Instance->RegisterTCPClient(serverIP, nPort);
}

// Send command to get bone size from server by tcp/ip. command data will be received by command callback.
// Return FALSE if any error occured. UDP service is not supported now.
BDR_API BOOL BRCommandFetchAvatarDataFromServer(SOCKET_REF sockRef, int avatarIndex, CmdId cmdId)
{
	return SocketClientsManager::Instance->FetchAvatarDataFromServer(sockRef, avatarIndex, cmdId);
}

BDR_API BOOL BRCommandFetchDataFromServer(SOCKET_REF sockRef, CmdId cmdId)
{
	return SocketClientsManager::Instance->FetchDataFromServer(sockRef, cmdId);
}

// Register paramemt(s) to server for automatically notifying status changed.
BDR_API BOOL BRRegisterAutoSyncParmeter(SOCKET_REF sockRef, CmdId cmdId)
{
	return SocketClientsManager::Instance->AutoSyncCommands(sockRef, cmdId, true);
}

BDR_API BOOL BRUnregisterAutoSyncParmeter(SOCKET_REF sockRef, CmdId cmdId)
{
	return SocketClientsManager::Instance->AutoSyncCommands(sockRef, cmdId, false);
}

// Check status
BDR_API SocketStatus BRGetSocketStatus(SOCKET_REF sockRef)
{
	if (sockRef == NULL) return CS_OffWork;

	return SocketClientsManager::Instance->GetSocketStatus(sockRef);
}

// Disconnect from server
BDR_API void BRCloseSocket(SOCKET_REF sockRef)
{
	if (sockRef == NULL) return;

	SocketClientsManager::Instance->Disconnect(sockRef);
}

// If any error, you can call 'BRGetLastErrorMessage' to get error information
BDR_API char* BRGetLastErrorMessage()
{
    return Error::Instance()->GetLastError();

#if 0
    // follow code is not platform-cross, so throw away.

#ifdef __OS_XUN__
    return Error::Instance()->GetLastError();
#else
    char* msg = Error::Instance()->GetLastError();
    int len = strlen(msg);

    char* pszReturn = (char*)::CoTaskMemAlloc(len);
    // Copy the contents of szSampleString
    // to the memory pointed to by pszReturn.
    strcpy(pszReturn, msg);
    // Return pszReturn.
	return pszReturn;
#endif
#endif
}

// Start a UDP service to receive data at 'nPort'
BDR_API SOCKET_REF BRStartUDPServiceAt(int nPort)
{
	return SocketClientsManager::Instance->StartUDPServiceAt(nPort);
}
