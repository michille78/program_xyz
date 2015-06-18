#include "stdafx.h"
#include "Error.h"
#include "Sock.h"
#include "SockClient.h"
#include "UDPDataReceiver.h"
#include "SocketClientsManager.h"
#include "../SocketCommand.h"
#include "../NeuronDataReader.h"
#include "DataParser.h"

SocketClientsManager* SocketClientsManager::Instance = new SocketClientsManager();

void* SocketClientsManager::customedObject = NULL;
FrameDataReceived SocketClientsManager::fnBvhDataCallback = NULL;

void* SocketClientsManager::customedOptDataObject = NULL;
OptDataReceived SocketClientsManager::fnOptDataCallback = NULL;

void* SocketClientsManager::customedCommandDataObject = NULL;
CommandDataReceived SocketClientsManager::fnCommandDataCallback = NULL;

void* SocketClientsManager::socketStatusCustomedObject = NULL;
SocketStatusChanged SocketClientsManager::fnSocketStatusCallback = NULL;



void SocketClientsManager::_fnSocketDataReceived(void* sockRef, unsigned char* data, int len)
{
	((Sock*)sockRef)->dataParser->Parse(sockRef, data, len);
}

SocketClientsManager::SocketClientsManager()
{
	
}

SocketClientsManager::~SocketClientsManager()
{
	
}

// 创建一个客户端
SOCKET_REF SocketClientsManager::RegisterTCPClient(char* serverIP, int nPort)
{
	// 检查是否已经存在
	int count = sockClients.size();
	for (int i = 0; i < count; i++)
	{
		SockClient* s = sockClients[i];
		if (strcmp(s->strIP, serverIP) == 0 && s->nPort == nPort)
		{
			return s;
		}
	}

	// 创建一个新的客户端
	SockClient* s = new SockClient();
	s->dataParser = new DataParser();
	if (s->ConnectTo(serverIP, nPort))
	{
		s->SetDataReceivedCallback(_fnSocketDataReceived);

		sockClients.push_back(s);
		return s;
	}
	else
	{
		delete s;
	}

	return NULL;
}

BOOL SocketClientsManager::AutoSyncCommands(SOCKET_REF sockRef, CmdId cmdId, bool isRegister)
{
	int count = sockClients.size();
	for (int i = 0; i < count; i++)
	{
		SockClient* s = sockClients[i];
		if (s == sockRef)
		{
			CommandPack cr;
			cr.DataCount = 0;
			cr.DataLength = sizeof(CommandPack);
			cr.Token1 = 0xAAFF;
			cr.Token2 = 0xBBFF;
			GetVersion(&cr.DataVersion);

			if (isRegister)
			{
				cr.CommandId = Cmd_RegisterEvent;
			}
			else
			{
				cr.CommandId = Cmd_UnRegisterEvent;
			}
			memcpy(cr.CmdParaments, &cmdId, sizeof(cmdId));

			cr.Reserved1 = 0;

			return s->Write((unsigned char*)&cr, sizeof(CommandPack));
		}
	}

	count = udpReceivers.size();
	for (int i = 0; i < count; i++)
	{
		UDPDataReceiver* s = udpReceivers[i];
		if (s == sockRef)
		{
			char* lastMsg = "Command is not support in UDP service.";
			Error::Instance()->SetLastError(lastMsg);

			if (SocketClientsManager::fnSocketStatusCallback)
			{
				SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, sockRef, CS_OffWork, lastMsg);
			}
		}
	}

	return FALSE;
}


BOOL SocketClientsManager::FetchDataFromServer(SOCKET_REF sockRef, CmdId cmdId)
{
	return FetchAvatarDataFromServer(sockRef, 0, cmdId);
}

BOOL SocketClientsManager::FetchAvatarDataFromServer(SOCKET_REF sockRef, int avatarIndex, CmdId cmdId)
{
	int count = sockClients.size();
	for (int i = 0; i < count; i++)
	{
		SockClient* s = sockClients[i];
		if (s == sockRef)
		{
			CommandPack cr;
			cr.DataCount = 0;
			cr.DataLength = sizeof(CommandPack);
			cr.Token1 = 0xAAFF;
			cr.Token2 = 0xBBFF;
			cr.CommandId = cmdId;
			GetVersion(&cr.DataVersion);
			memcpy(cr.CmdParaments, &avatarIndex, sizeof(avatarIndex));

			cr.Reserved1 = 0;

			return s->Write((unsigned char*)&cr, sizeof(CommandPack));
		}
	}

	count = udpReceivers.size();	
	for (int i = 0; i < count; i++)
	{
		UDPDataReceiver* s = udpReceivers[i];
		if (s == sockRef)
		{
			char* lastMsg = "Command is not support in UDP service.";
			Error::Instance()->SetLastError(lastMsg);

			if (SocketClientsManager::fnSocketStatusCallback)
			{
				SocketClientsManager::fnSocketStatusCallback(SocketClientsManager::socketStatusCustomedObject, sockRef, CS_OffWork, lastMsg);
			}
		}
	}

	return FALSE;
}

// Check status
SocketStatus SocketClientsManager::GetSocketStatus(SOCKET_REF sockRef)
{
	// 检查是否已经存在
	int count = sockClients.size();
	for (int i = 0; i < count; i++)
	{
		SockClient* s = sockClients[i];
		if (s == sockRef)
		{
			return s->IsConnected() ? CS_Running : CS_OffWork;
		}
	}

	count = udpReceivers.size();
	for (int i = 0; i < count; i++)
	{
		UDPDataReceiver* u = udpReceivers[i];
		if ((SOCKET_REF)u == sockRef)
		{
			return u->IsRunning ? CS_Running : CS_OffWork;
		}
	}

	return CS_OffWork;
}

void SocketClientsManager::Disconnect(SOCKET_REF connectorRef)
{
	int count = sockClients.size();
	for (int i = 0; i < count; i++)
	{
		SockClient* s = sockClients[i];
		if (s == connectorRef)
		{
			// 断开连接
			s->Disconnect();

			// 移除
			sockClients.erase(sockClients.begin() + i);

			return;
		}
	}

	count = udpReceivers.size();
	for (int i = 0; i < count; i++)
	{
		UDPDataReceiver* u = udpReceivers[i];
		if ((SOCKET_REF)u == connectorRef)
		{
			u->Stop();
			return;
		}
	}
}

void SocketClientsManager::DisconnectAllClients()
{
	int count = sockClients.size();
	for (int i = 0; i < count; i++)
	{
		sockClients[i]->Disconnect();
	}
	
	sockClients.clear();
}

// Start a UDP service to receive data at 'nPort'
SOCKET_REF SocketClientsManager::StartUDPServiceAt(int nPort)
{
	int count = udpReceivers.size();
	for (int i = 0; i < count; i++)
	{
		UDPDataReceiver* u = udpReceivers[i];
		if (u->nPort == nPort)
		{
			return (SOCKET_REF)u;
		}
	}

	UDPDataReceiver* u = new UDPDataReceiver();
	u->dataParser = new DataParser();
	if (u->ListenAt(nPort))
	{
		u->SetDataReceivedCallback(_fnSocketDataReceived);
		
		udpReceivers.push_back(u);

		return (SOCKET_REF)u;
	}
	else
	{
		delete u;
		return NULL;
	}
}

void SocketClientsManager::StopAllUDPService()
{
	int count = udpReceivers.size();
	for (int i = 0; i < count; i++)
	{
		UDPDataReceiver* u = udpReceivers[i];
		u->Stop();
	}

	udpReceivers.clear();
}
