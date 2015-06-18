// dllmain.cpp : 定义 DLL 应用程序的入口点。
#include "stdafx.h"
#include "../NeuronDataReader.h"
#include "SocketClientsManager.h"

#ifdef __OS_XUN__

#else

#include <winsock2.h>

WORD wVersionRequested;
WSADATA wsaData;


BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	    if (WSAStartup (MAKEWORD(2,0), &wsaData) != 0) 
	    {
		    printf("Sockets Could Not Be Initialized");
	    }
        break;
	case DLL_THREAD_ATTACH:
        break;
	case DLL_THREAD_DETACH:
        break;
	case DLL_PROCESS_DETACH:
		SocketClientsManager::Instance->DisconnectAllClients();
	    WSACleanup();
		break;
	}
	return TRUE;
}

#endif