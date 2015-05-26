//#include "stdafx.h"
#include "Definitions.h"
#include <stdio.h>    
#include <time.h>    

double PI = 3.14159265;
double PI2 = PI/2;
double sinPI2 = sin(PI/2);
double cosPI2 = cos(PI/2);
double PIdiv = 180.0/PI;

// 返回从程序开启后的毫秒数
int getCurrentTime()    
{    
    return clock() / CLOCKS_PER_SEC;    
/*   timeval tv;
   gettimeofday(&tv,NULL);    
   return tv.tv_sec * 1000 + tv.tv_usec / 1000;   */ 
}   

void printpak(unsigned char* data, int len)
{
	char* buff = new char[len*3+3];
	memset(buff, '\0', len*3+3);
	char hex[4];
	for( int i = 0; i<len; i++)
	{
		sprintf_s(hex, "%02x ", data[i]);
		memcpy(buff + i * 3, hex, 3);
	}
	printf(buff);
	printf("\n");

	delete[] buff;
}

// 更科学的帧率统计
double FrameRate[23];
int FrameCounter[23];
void FramePercentCalcu(unsigned char* data)
{
	// 按照多少帧内某模块丢了几帧来计算得包率

	static unsigned char frmType;
	static int calculateFrames;

	if(frmType!=data[2]) // 帧号发生变化
	{
		if(calculateFrames==96)
		{
			for(int I=0;I<23;I++)
			{
				FrameRate[I] = (double)FrameCounter[I] / calculateFrames;
				FrameCounter[I] = 0;
			}
			calculateFrames = 0;
		}
		calculateFrames++;
		
		frmType = data[2];
	}
}

bool IsNumber(double x) 
{
    // This looks like it should always be true, 
    // but it's false if x is a NaN.
    return (x == x); 
}
bool IsFiniteNumber(double x) 
{
    return (x <= DBL_MAX && x >= -DBL_MAX); 
}  

//bool IsNAN(double x) 
//{
//    if( (x <= DBL_MAX && x >= -DBL_MAX))
//		return true;
//    return (x != x); 
//}  
bool IsNaN(double x) 
{
//#define _FPCLASS_SNAN   0x0001  /* signaling NaN */
//#define _FPCLASS_QNAN   0x0002  /* quiet NaN */
//#define _FPCLASS_NINF   0x0004  /* negative infinity */
//#define _FPCLASS_NN     0x0008  /* negative normal */
//#define _FPCLASS_ND     0x0010  /* negative denormal */
//#define _FPCLASS_NZ     0x0020  /* -0 */
//#define _FPCLASS_PZ     0x0040  /* +0 */
//#define _FPCLASS_PD     0x0080  /* positive denormal */
//#define _FPCLASS_PN     0x0100  /* positive normal */
//#define _FPCLASS_PINF   0x0200  /* positive infinity */

	// 用掩码与（&）操作判断是否NAN
	return (_fpclass(x)&0x0207)>0;

	switch(_fpclass(x))
	{
	case _FPCLASS_NN:  /* negative normal */
	case _FPCLASS_ND:  /* negative denormal */
	case _FPCLASS_NZ:  /* -0 */
	case _FPCLASS_PZ:  /* +0 */
	case _FPCLASS_PD:  /* positive denormal */
	case _FPCLASS_PN:  /* positive normal */
		return false;
	case _FPCLASS_SNAN:  /* signaling NaN */
	case _FPCLASS_QNAN:  /* quiet NaN */
	case _FPCLASS_NINF:  /* negative infinity */
	case _FPCLASS_PINF:  /* positive infinity */
		return true;
	}
	return false;
}  

#define MAXSTRLENGTH    255
BOOL Char2Wchar(TCHAR* pDest, char* pSrc, int nDestStrLen)
{
     int nSrcStrLen = 0;
     int nOutputBuffLen = 0;
     int retcode = 0;

     if(pDest == NULL || pSrc == NULL)
     {
          //SysDebug(MID_EXCEPTION, "Char2Wchar: Input Args NULL\n");
          return FALSE;
     }

     nSrcStrLen = strlen(pSrc);
     if(nSrcStrLen == 0)
     {  
          //SysDebug(MID_EXCEPTION, "Char2Wchar: Strlen zero\n");
          return FALSE;
     }

     nDestStrLen = nSrcStrLen;

     if (nDestStrLen > MAXSTRLENGTH - 1)
     {
          //SysDebug(MID_EXCEPTION, "Char2Wchar: Check nSrcStrLen\n");
          return FALSE;
     }
     memset(pDest,0,sizeof(TCHAR)*nDestStrLen);
     nOutputBuffLen = MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, pSrc,
     nSrcStrLen, pDest, nDestStrLen);
 
     if (nOutputBuffLen == 0)
     {
          retcode = GetLastError();
          //Sysdebug("Char2Wchar : MultiByteToWideChar returned ERROR \n",0,0);  
          return FALSE;
     }

     pDest[nOutputBuffLen] = '\0';
     return TRUE;
}


//-------------------------------------------------------------------------------------
//Description:
// This function maps a character string to a wide-character (Unicode) string
//
//Parameters:
// lpcszStr: [in] Pointer to the character string to be converted 
// lpwszStr: [out] Pointer to a buffer that receives the translated string. 
// dwSize: [in] Size of the buffer
//
//Return Values:
// TRUE: Succeed
// FALSE: Failed
// 
//Example:
// MByteToWChar(szA,szW,sizeof(szW)/sizeof(szW[0]));
//---------------------------------------------------------------------------------------
BOOL MByteToWChar(LPCSTR lpcszStr, LPWSTR lpwszStr, DWORD dwSize)
{
	// Get the required size of the buffer that receives the Unicode 
	// string. 
	DWORD dwMinSize;
	dwMinSize = MultiByteToWideChar (CP_ACP, 0, lpcszStr, -1, NULL, 0);

	if(dwSize < dwMinSize)
	{
		return FALSE;
	}
	// Convert headers from ASCII to Unicode.
	MultiByteToWideChar (CP_ACP, 0, lpcszStr, -1, lpwszStr, dwMinSize);  
	return TRUE;
}

//-------------------------------------------------------------------------------------
//Description:
// This function maps a wide-character string to a new character string
//
//Parameters:
// lpcwszStr: [in] Pointer to the character string to be converted 
// lpszStr: [out] Pointer to a buffer that receives the translated string. 
// dwSize: [in] Size of the buffer
//
//Return Values:
// TRUE: Succeed
// FALSE: Failed
// 
//Example:
// MByteToWChar(szW,szA,sizeof(szA)/sizeof(szA[0]));
//---------------------------------------------------------------------------------------
BOOL WCharToMByte(LPCWSTR lpcwszStr, LPSTR lpszStr, DWORD dwSize)
{
	DWORD dwMinSize;
	dwMinSize = WideCharToMultiByte(CP_OEMCP,NULL,lpcwszStr,-1,NULL,0,NULL,FALSE);
	if(dwSize < dwMinSize)
	{
		return FALSE;
	}
	WideCharToMultiByte(CP_OEMCP,NULL,lpcwszStr,-1,lpszStr,dwSize,NULL,FALSE);
	return TRUE;
}
