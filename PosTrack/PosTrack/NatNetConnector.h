#pragma once
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "NatNetDataHandler.h"

class NatNetConnector
{
    int iConnectionType;
	char szMyIPAddress[128];
	char szServerIPAddress[128];

	unsigned int MyServersDataPort;
	unsigned int MyServersCommandPort;
	
	// Establish a NatNet Client connection
	int createClient(int iConnectionType);

	static void DataHandler(sFrameOfMocapData* data, void* pUserData);

	NatNetClient* theClient;
	NatNetDataHandler* theHandler;

public:
	NatNetConnector();
	~NatNetConnector();
	
	BOOL Connect();
	BOOL Connect(char* ipaddress, int port);
	void SetDataHandler(NatNetDataHandler* handler);

	BOOL IsConnected;

	void Reset();
	void Release();
};