// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#ifdef __OS_XUN__
typedef int                  SOCKET;
#define sprintf_s            snprintf
#define Sleep(t)             usleep(t*1000)
#define closesocket(sock)    close(sock)
#define HOSTENT              struct hostent

/*
 * This is used instead of -1, since the
 * SOCKET type is unsigned.
 */
#define INVALID_SOCKET  (SOCKET)(~0)
#define SOCKET_ERROR            (-1)


#else
// 包括 SDKDDKVer.h 将定义可用的最高版本的 Windows 平台。
// 如果要为以前的 Windows 平台生成应用程序，请包括 WinSDKVer.h，并将
// WIN32_WINNT 宏设置为要支持的平台，然后再包括 SDKDDKVer.h。
#include <SDKDDKVer.h>
#define WIN32_LEAN_AND_MEAN  //  从 Windows 头文件中排除极少使用的信息
#include <windows.h>         // Windows 头文件:
#endif


#ifdef __OS_XUN__
#include <sys/types.h>   // Types used in sys/socket.h and netinet/in.h
#include <netinet/in.h>  // Internet domain address structures and functions
#include <sys/socket.h>  // Structures and functions used for socket API
#include <netdb.h>       // Used for domain/DNS hostname lookup
#include <arpa/inet.h>   // inet_addr
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#else
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib")
#endif


// socket received data
typedef void(__stdcall *SocketDataReceived)(void* sockRef, unsigned char* data, int len);



#ifdef _DEBUG
   #define DEBUG_NEW   new( _CLIENT_BLOCK, __FILE__, __LINE__)
   #define new DEBUG_NEW

   #define ASSERT(e) if(e==NULL || e==false || e==FALSE) DbgRaiseAssertionFailure();
#else
   #define DEBUG_NEW

   #define ASSERT(e)         // nothing
#endif

#include <stdlib.h>          // atof, atoi
#include <thread>            // std::thread
#include <mutex>             // std::mutex
#include <vector>
using namespace std;
