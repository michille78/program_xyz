// stdafx.cpp : 只包括标准包含文件的源文件
// EKFLib.pch 将作为预编译头
// stdafx.obj 将包含预编译类型信息
#include "stdafx.h"

// TODO: 在 STDAFX.H 中
// 引用任何所需的附加头文件，而不是在此文件中引用

double PI2 = PI/2;
double sinPI2 = sin(PI/2);
double cosPI2 = cos(PI/2);
double PIdiv = 180.0/PI;


// 默认掌骨中的相邻骨骼夹角
const double InHandFingerAngle    = 6.0;
const double InHandThumbAngle     = 60.0;
const double ThumbAngle           = 75.0;
const double ThumbFaceAngle       = 60.0;
const double HandXLimitAngle      = 45;
const double HandYLimitAngle      = 90;
const double Finger1LimitXPAngle  = 100; // 正向100度
const double Finger1LimitXNAngle  = 45;  // 反向45度
const double Finger1LimitYAngle   = 5;   // 20 -> 5
const double FingerNLimitXAngle   = 5;
const double FingerPLimitXAngle   = 105;
const double Thumb1NLimitYAngle   = 5;
const double Thumb1PLimitYAngle   = 80;



// 返回从程序开启后的毫秒数
unsigned long getCurrentTime()
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
		sprintf_s(hex, sizeof(hex), "%02x ", data[i]);
		memcpy(buff + i * 3, hex, 3);
	}
	printf("%s", buff);
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

PNBOOL IsNumber(double x) 
{
    // This looks like it should always be true, 
    // but it's false if x is a NaN.
    return (x == x); 
}
PNBOOL IsFiniteNumber(double x) 
{
    return (x <= DBL_MAX && x >= -DBL_MAX); 
}  

//bool IsNAN(double x) 
//{
//    if( (x <= DBL_MAX && x >= -DBL_MAX))
//		return true;
//    return (x != x); 
//}  
PNBOOL IsNaN(double x) 
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
    
#ifdef __OS_XUN__
    return !(x == x);
#else
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
#endif
}

#define MAXSTRLENGTH    255
PNBOOL Char2Wchar(wchar_t* pDest, char* pSrc, int nDestStrLen)
{
    /*
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
     */
     return TRUE;
}

/*

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
PNBOOL MByteToWChar(LPCSTR lpcszStr, LPWSTR lpwszStr, DWORD dwSize)
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
PNBOOL WCharToMByte(LPCWSTR lpcwszStr, LPSTR lpszStr, DWORD dwSize)
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
 */



#define GRAVITY      256

	
// 约束到一个小范围内
float g_radio = 0.3f; // 0.3M
// 位移衰减系数
float g_att_S = 1.0f;
// 速度衰减系数
float g_att_V = 0.98f;
// 是否积分的速度门限
float g_limit_gate_V = 0.2f; // 米每秒

 void SetAccsIntPrsParams(float range, float attS, float attV, float limit_gate_V)
{
	g_radio = range;
	g_att_S = attS;
	g_att_V = attV;
	g_limit_gate_V = limit_gate_V;
}

 void GetAccsIntPrsParams(float* range, float* attS, float* attV, float* limit_gate_V)
{
	 *range = g_radio;
	 *attS = g_att_S;
	 *attV = g_att_V;
	 *limit_gate_V = g_limit_gate_V;
}

 void AccsIntPrs(QUATERNION_t* pQuat,Acc_Rawdat_t* pAccDat,uint32_t* pOV,Acc_Int_Dat_t* pAccInt, float dt)
{
	float modulus = sqrtf((float)pAccDat->ax * pAccDat->ax + (float)pAccDat->ay * pAccDat->ay + (float)pAccDat->az * pAccDat->az);
	
	UINT32 acc_range_flg = 0;
	UINT32 s_range_flg = 1;
	float vcc_modulus = 0;

	Point3D_t GV;
	GV.X = 0;
	GV.Y = 0;
	GV.Z = GRAVITY;

	float ftmp = 1.0 / modulus;

	Point3D_t a_f;
	Point3D_t a_ftmp;

	a_f.X = pAccDat->ax * ftmp;
	a_f.Y = pAccDat->ay * ftmp;
	a_f.Z = pAccDat->az * ftmp;

	q_v_qc(pQuat, &a_f, &a_ftmp);
	//printf("%0.4f %0.4f %0.4f\n", a_ftmp.X, a_ftmp.Y, a_ftmp.Z);

	Point3D_t f;
	f.X = a_ftmp.X * modulus - GV.X;
	f.Y = a_ftmp.Y * modulus - GV.Y;
	f.Z = a_ftmp.Z * modulus - GV.Z;  //去除重力

	float f_tmp = f.X*f.X + f.Y*f.Y + f.Z*f.Z;
	
	// 加速度积分门限
	if(f_tmp > (0.0036 * GRAVITY * GRAVITY))
	{
		//速度积分
		pAccInt->Acc_X_Int_A += f.X;
		pAccInt->Acc_Y_Int_A += f.Y;
		pAccInt->Acc_Z_Int_A += f.Z;
	}
	else
	{
		// 速度不积分标示
		acc_range_flg = 1;
	}
	
	// 速度衰减门限
	if((f_tmp < (0.0016 * GRAVITY * GRAVITY)) /*10 mg*/ ||
	((modulus <= 1.04 * GRAVITY ) && (modulus >= 0.96 * GRAVITY)))
	{
		pAccInt->Acc_X_Int_A *= g_att_V;  //速度衰减
		pAccInt->Acc_Y_Int_A *= g_att_V;
		pAccInt->Acc_Z_Int_A *= g_att_V;
	}
	
	//如果加速度和速度低于某个值，位移保持不变
	if(acc_range_flg)
	{
		vcc_modulus =	sqrtf(pAccInt->Acc_X_Int_A * pAccInt->Acc_X_Int_A + pAccInt->Acc_Y_Int_A * pAccInt->Acc_Y_Int_A + 
						pAccInt->Acc_Z_Int_A * pAccInt->Acc_Z_Int_A);
		
		vcc_modulus = vcc_modulus / 256;// AD值转成g
		vcc_modulus = vcc_modulus * 9.81;// g转成米每二次方秒(m/s2)
		vcc_modulus = vcc_modulus * dt;  // 米每二次方秒(m/s2)转成速度（米每秒）

		if(vcc_modulus < g_limit_gate_V)
		{
			// 位移不积分
			s_range_flg = 0;
		}		
	}
	
	// 位移积分
	if(s_range_flg)
	{
		// 位移积分
		pAccInt->Acc_X_Int_S += pAccInt->Acc_X_Int_A / 256 * 9.81 * dt * dt; 	//  mm
		pAccInt->Acc_Y_Int_S += pAccInt->Acc_Y_Int_A / 256 * 9.81 * dt * dt; 	//  mm
		pAccInt->Acc_Z_Int_S += pAccInt->Acc_Z_Int_A / 256 * 9.81 * dt * dt; 	//  mm
	}


	// 约束到一个小范围内活动
	float sqR = (pAccInt->Acc_X_Int_S * pAccInt->Acc_X_Int_S + pAccInt->Acc_Y_Int_S * pAccInt->Acc_Y_Int_S + pAccInt->Acc_Z_Int_S * pAccInt->Acc_Z_Int_S);
	if(sqR >= g_radio*g_radio)	//0.09m2
	{
		sqR = sqrt(sqR);

		pAccInt->Acc_X_Int_S /= sqR / g_radio;
		pAccInt->Acc_Y_Int_S /= sqR / g_radio;
		pAccInt->Acc_Z_Int_S /= sqR / g_radio;				
	}
	
	if(s_range_flg)
	{
		// 给一个小的衰减
		pAccInt->Acc_X_Int_S *= g_att_S;
		pAccInt->Acc_Y_Int_S *= g_att_S;
		pAccInt->Acc_Z_Int_S *= g_att_S;
	}
}	

/*****************************************/
