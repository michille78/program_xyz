// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

// 在xcode编译器配置选项中定义
//#define __OS_XUN__


#ifdef __OS_XUN__
    #define PNLIB_EXPORTS

    #include <unistd.h>          // for 'sleep', 'usleep'

    #define sprintf_s            snprintf
    #define _isnan               isnan

    #define MAX_PATH             255

    typedef unsigned short       UINT16;
    typedef unsigned int         UINT32;
    typedef unsigned long long   UINT64;
    typedef unsigned short       USHORT;
    typedef unsigned char        UCHAR;
    typedef unsigned char        BYTE;
    typedef wchar_t              WCHAR;

    #define strcat_s(s1, s1max, s2) (strncat(s1, s2, s1max-strlen(s1)))
    #define OutputDebugStringA(str) (printf("%s", str))



#else
    #define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
    #define _CRT_SECURE_NO_WARNINGS                 1
    // 包括 SDKDDKVer.h 将定义可用的最高版本的 Windows 平台。
    // 如果要为以前的 Windows 平台生成应用程序，请包括 WinSDKVer.h，并将
    // WIN32_WINNT 宏设置为要支持的平台，然后再包括 SDKDDKVer.h。
    #include <SDKDDKVer.h>

    //  从 Windows 头文件中排除极少使用的信息
    #define WIN32_LEAN_AND_MEAN

    // Windows 头文件:
    #include <windows.h>
    #include <io.h>

    #include <Mmsystem.h>             // for timeBeginPeriod()/timeEndPeriod()



#endif

#include <math.h>
#include <stdio.h>
#include <time.h>   
#include <list>
#include <mutex>        // std::mutex
#include <algorithm>    // std::max
#include <thread>       // std::thread
using namespace std;

#include "Eigen"
using namespace Eigen;
typedef SparseMatrix<double> SpMatrix;  // 稀疏矩阵
typedef MatrixXd             DeMatrix;  // 稠密矩阵

#pragma warning (disable : 4267)
#pragma warning (disable : 4244)
//#pragma warning (disable : 4367)

#include "../PNLib.h"
#include "../PNDataTypes.h"

// 定义全全身骨骼数
#define SENSOR_PACK_SIZE     37
#define BODY_BONE_COUNT      0x60
#define PI                   3.14159265


#ifdef __OS_XUN__
    #define max(a, b) (((a)>(b))?(a):(b))
    #define min(a, b) (((a)<(b))?(a):(b))
#endif


#ifdef _DEBUG
   #define DEBUG_NEW   new( _CLIENT_BLOCK, __FILE__, __LINE__)
   #define new DEBUG_NEW
#else
   #define DEBUG_NEW
#endif

#ifdef _DEBUG

#define ASSERT(e) if(e==NULL || e==false || e==FALSE) DbgRaiseAssertionFailure();
#else
#define ASSERT(e) /* nothing */

#endif


// 数据类型定义
typedef unsigned short uint16_t;
typedef unsigned int   uint32_t;
typedef signed short   int16_t;

#define GYROSCALE       0.07f
#define AD_2_MG_GAIN    3.90625f

PNBOOL IsNaN(double x);

extern double PI2;
extern double sinPI2;
extern double cosPI2;
extern double PIdiv;


// 默认掌骨中的相邻骨骼夹角
extern const double InHandFingerAngle;
extern const double InHandThumbAngle;
extern const double ThumbAngle;
extern const double ThumbFaceAngle;
extern const double HandXLimitAngle;
extern const double HandYLimitAngle;
extern const double Finger1LimitXPAngle; // 正向100度
extern const double Finger1LimitXNAngle; // 反向45度
extern const double Finger1LimitYAngle; // 20 -> 5
extern const double FingerNLimitXAngle;
extern const double FingerPLimitXAngle;
extern const double Thumb1NLimitYAngle;
extern const double Thumb1PLimitYAngle;



#ifndef __OS_XUN__
extern HMODULE hInstance;
#endif

PNBOOL Char2Wchar(wchar_t* pDest, char* pSrc, int nDestStrLen);

#pragma pack(push, 1)

//typedef cvm::rvector QUATERNION_t;
struct QUATERNION_t
{
	float qs;
	float qx;
	float qy;
	float qz;
public:
    inline QUATERNION_t()
    {
        qs = 1;
        qx = 0;
        qy = 0;
		qz = 0;
    }
    inline QUATERNION_t(QUATERNION_t* q)
    {
        qs = q->qs;
        qx = q->qx;
        qy = q->qy;
        qz = q->qz;
    }
    inline QUATERNION_t(double s, double x, double y, double z)
    {
        qs = (float)s;
        qx = (float)x;
        qy = (float)y;
        qz = (float)z;
    }
    inline QUATERNION_t(float* sxyz, PNBOOL qsAt0)
    {
		if(qsAt0)
		{
			qs = sxyz[0];
			qx = sxyz[1];
			qy = sxyz[2];
			qz = sxyz[3];
		}
		else
		{
			qx = sxyz[0];
			qy = sxyz[1];
			qz = sxyz[2];
			qs = sxyz[3];
		}
    }
    inline QUATERNION_t(double* sxyz, PNBOOL qsAt0)
    {
		if(qsAt0)
		{
			qs = (float)sxyz[0];
			qx = (float)sxyz[1];
			qy = (float)sxyz[2];
			qz = (float)sxyz[3];
		}
		else
		{
			qx = (float)sxyz[0];
			qy = (float)sxyz[1];
			qz = (float)sxyz[2];
			qs = (float)sxyz[3];
		}
    }
    inline QUATERNION_t(int s, int x, int y, int z)
    {
        qs = s;
        qx = x;
        qy = y;
        qz = z;
    }
	inline QUATERNION_t operator +(QUATERNION_t q2)
    {
		QUATERNION_t q;
		q.qs = this->qs + q2.qs;
		q.qx = this->qx + q2.qx;
		q.qy = this->qy + q2.qy;
		q.qz = this->qz + q2.qz;
        return q;
    }
	inline QUATERNION_t operator *(double t)
    {
		QUATERNION_t q;
		q.qs = this->qs * t;
		q.qx = this->qx * t;
		q.qy = this->qy * t;
		q.qz = this->qz * t;
        return q;
    }
	inline QUATERNION_t operator /(double t)
    {
		QUATERNION_t q;
		q.qs = this->qs / t;
		q.qx = this->qx / t;
		q.qy = this->qy / t;
		q.qz = this->qz / t;
        return q;
    }
    inline void Normalize()
    {
        double mod = sqrt(qs * qs + qx * qx + qy * qy + qz * qz);

		if(mod==0)
		{
			qs = 1;
			mod = 1;
		}

        qs /= (float)mod;
        qx /= (float)mod;
        qy /= (float)mod;
        qz /= (float)mod;
    }
    inline double Mod()
    {
        return sqrt(qs * qs + qx * qx + qy * qy + qz * qz);
    }

    inline QUATERNION_t Conjuct()
    {
        return QUATERNION_t(qs, -qx, -qy, -qz);
    }
    inline float* data()
	{
		return &qs;
	}
};


typedef struct 
{
	int16_t gx;
	int16_t gy;
	int16_t gz;
}Gys_Rawdat_t;

typedef struct 
{
	int16_t ax;
	int16_t ay;
	int16_t az;
}Acc_Rawdat_t;

typedef struct 
{
	float Acc_X_Int_A;
	float Acc_Y_Int_A;
	float Acc_Z_Int_A;
	float Acc_X_Int_S;
	float Acc_Y_Int_S;
	float Acc_Z_Int_S;
}Acc_Int_Dat_t;

struct Point3D_double
{
    double X;
    double Y;
    double Z;
    inline Point3D_double(float x, float y, float z)
    {
        X = x;
        Y = y;
        Z = z;
    }
    inline Point3D_double(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }
	inline double* get()
    {
		return &X;
    }
};

struct Point3D_t
{
    float X;
    float Y;
    float Z;
public:
    inline Point3D_t()
    {
        X = 0;
        Y = 0;
        Z = 0;
    }
    inline Point3D_t(Point3D_t* p)
    {
        X = p->X;
        Y = p->Y;
		Z = p->Z;
    }
	inline Point3D_t(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }
    inline Point3D_t(float* xyz)
    {
        X = xyz[0];
        Y = xyz[1];
        Z = xyz[2];
    }
    inline Point3D_t(double* xyz)
    {
        X = (float)xyz[0];
        Y = (float)xyz[1];
        Z = (float)xyz[2];
    }
    inline void Normalize()
    {
        double mod = sqrt(X * X + Y * Y + Z * Z);
        X /= (float)mod;
        Y /= (float)mod;
        Z /= (float)mod;
    }
    inline double mod()
    {
        return sqrt(X * X + Y * Y + Z * Z);
    }
    inline double mod2()
    {
        return X * X + Y * Y + Z * Z;
    }
	inline Point3D_t operator *(double f)
    {
		Point3D_t v1;
		v1.X = this->X * (float)f;
        v1.Y = this->Y * (float)f;
        v1.Z = this->Z * (float)f;
        return v1;
    }
	inline Point3D_t operator /(double f)
    {
		Point3D_t v1;
		v1.X = this->X / (float)f;
        v1.Y = this->Y / (float)f;
        v1.Z = this->Z / (float)f;
        return v1;
    }
	inline Point3D_t operator +(Point3D_t p)
    {
		Point3D_t v1;
		v1.X = this->X + p.X;
		v1.Y = this->Y + p.Y;
		v1.Z = this->Z + p.Z;
        return v1;
    }
	inline Point3D_t operator +(double f)
    {
		Point3D_t v1;
		v1.X = this->X + (float)f;
        v1.Y = this->Y + (float)f;
        v1.Z = this->Z + (float)f;
        return v1;
    }
	inline Point3D_t &operator +=(Point3D_t &p)
    {
		this->X += p.X;
		this->Y += p.Y;
		this->Z += p.Z;
        return *this;
    }
	inline Point3D_t operator -(Point3D_t p) // 重载减法
    {
		Point3D_t v1;
		v1.X = this->X - p.X;
		v1.Y = this->Y - p.Y;
		v1.Z = this->Z - p.Z;
        return v1;
    }
	inline Point3D_t operator -() // 重载负号
    {
		Point3D_t v1;
		v1.X = -this->X;
		v1.Y = -this->Y;
		v1.Z = -this->Z;
        return v1;
    }
	inline Point3D_t power()
    {
		Point3D_t v1;
		v1.X = X * X;
		v1.Y = Y * Y;
		v1.Z = Z * Z;
		return v1;
    }
	inline double dot()
    {
		return (X * X + Y * Y + Z * Z);
    }
	inline double dot(Point3D_t v2)
    {
		return (this->X * v2.X + this->Y * v2.Y + this->Z * v2.Z);
    }
	inline void Print()
    {
		printf("X:%0.4f Y:%0.4f Z:%0.4f\n", X, Y, Z);
    }
	inline float* get()
    {				
		return &X;
    }
    inline Point3D_double ToDouble()
    {
        return Point3D_double(X, Y, Z);
    }
	static double dot(Point3D_t v1, Point3D_t v2)
    {
		return (v1.X * v2.X + v1.Y * v2.Y + v1.Z * v2.Z);
    }
	static Point3D_t cross(Point3D_t v1, Point3D_t v2)
	{
		return Point3D_t(v1.Y*v2.Z - v1.Z*v2.Y, v1.Z*v2.X - v1.X*v2.Z, v1.X*v2.Y - v1.Y*v2.X);
	}
};

inline Point3D_t cross(Point3D_t v1, Point3D_t v2)
{
	return Point3D_t(v1.Y*v2.Z - v1.Z*v2.Y, v1.Z*v2.X - v1.X*v2.Z, v1.X*v2.Y - v1.Y*v2.X);
}

//template <class T> 
//inline std::string ConvertToString(T value)
//{
//  std::stringstream ss;
//  ss << value;
//  return ss.str();
//}

#pragma pack(pop)

inline Point3D_t Quat2Euler(QUATERNION_t q)
{
    double sqx = pow(q.qx, 2);
    double sqy = pow(q.qy, 2);
    double sqz = pow(q.qz, 2);
    double sqw = pow(q.qs, 2);

    double test = q.qs * q.qx - q.qy * q.qz;

    double yaw = atan2(2 * (q.qy * q.qs + q.qx * q.qz), -sqx - sqy + sqz + sqw);

    double pitch = asin(2 * test);

    double roll = atan2(2 * (q.qz * q.qs + q.qx * q.qy), -sqx + sqy - sqz + sqw);

    if (test > 0.4999f)
    {
        // singularityat north pole
        yaw = 2 * atan2(q.qz, q.qs);
        pitch = PI / 2;//这个地方应该是pitch为PI/2 因为pitch是绕X轴转
        roll = 0;
    }
    else
    {
        if (test < -0.4999f)
        {
            // singularity at south pole
            yaw = -2 * atan2(q.qz, q.qs);
            pitch = -PI / 2;
            roll = 0;
        }
    }
	
    //return new Point3D_t(yaw * 180.0 / PI, pitch * 180.0 / PI, roll * 180.0 / PI);
    return new Point3D_t(yaw, pitch, roll);
}
inline QUATERNION_t Euler2Quat(Point3D_t pt)
{
	float cx = cos(pt.X/2);
	float sx = sin(pt.X/2);
	float cy = cos(pt.Y/2);
	float sy = sin(pt.Y/2);
	float cz = cos(pt.Z/2);
	float sz = sin(pt.Z/2);

	float w = cx*cy*cz + sx*sy*sz;
	float x = sx*cy*cz - cx*sy*sz;
	float y = cx*sy*cz + sx*cy*sz;
	float z = cx*cy*sz - sx*sy*cz;
	
	return QUATERNION_t(w, x, y, z);
}

void printpak(unsigned char* data, int len);

// 更科学的帧率统计
void FramePercentCalcu(unsigned char* data);

inline void PrintMatrixToConsole(double* m, int rows, int cols)
{
    printf("[%d, %d]\n", rows, cols);
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            printf("%f ", m[i * cols + j]);
        }
        printf("\r\n");
    }
}

inline void PrintMatrix(double* m, int rows, int cols)
{
    char strTmp[500];
    sprintf_s(strTmp, sizeof(strTmp), "[%d, %d]\n", rows, cols);
    OutputDebugStringA(strTmp);
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            sprintf_s(strTmp, sizeof(strTmp), "%f ", m[cols*i+j]);
            OutputDebugStringA(strTmp);
        }
        OutputDebugStringA("\r\n");
    }
}

inline void PrintMatrix(double* v, int cols)
{
    PrintMatrix(v, 1, cols);
}

inline void PrintMatrixToConsole(double* m, int rows, int cols, char* matrixName)
{
    printf("%s\n", matrixName);
    PrintMatrixToConsole(m, rows, cols);
}


inline void PrintMatrix(double* m, int rows, int cols, char* matrixName)
{
    OutputDebugStringA(matrixName);
    PrintMatrix(m, rows, cols);
}

/*

void PrintMatrix(char* filename, double* m, int rows, int cols)
{
    FILE* dddd = NULL;
    
    char fn[100];
    fn[0] = '\0';
    sprintf_s(fn, sizeof(fn), "%s.txt", filename);
    
    fopen_s(&dddd, fn, "w");
    
    static char buff[1024000];
    static char strTmp[500];
    memset(buff, '\0', sizeof(buff));
    
    memset(strTmp, '\0', sizeof(strTmp));
    sprintf_s(strTmp, "[%d, ", rows);
    strcat_s(buff, strTmp);
    memset(strTmp, '\0', sizeof(strTmp));
    sprintf_s(strTmp, "%d]\n", cols);
    strcat_s(buff, strTmp);
    
    for(int i=0;i<rows;i++)
    {
        for(int j=0;j<cols;j++)
        {
            memset(strTmp, '\0', sizeof(strTmp));
            sprintf_s(strTmp, "%f ", m[i * cols + j]);
            strcat_s(buff, strTmp);
        }
        strcat_s(buff, "\n");
    }
    
    OutputDebugStringA(buff);
    
    fprintf(dddd, "%s", buff);
    fflush(dddd);
    
    fclose(dddd);
}
 */

//inline void ToMatrix(MatrixXd& m, Point3D_t pt,  PNBOOL isColumnMatrx=FALSE)
//{
//	if(isColumnMatrx)
//	{
//		m(0,0)=pt.X;
//		m(1,0)=pt.Y;
//		m(2,0)=pt.Z;
//	}
//	else
//	{
//		m(0,0)=pt.X;
//		m(0,1)=pt.Y;
//		m(0,2)=pt.Z;
//	}
//}
//
//
//inline MatrixXd ToMatrix(Point3D_t pt,  PNBOOL isColumnMatrx=FALSE)
//{
//	if(isColumnMatrx)
//	{
//		MatrixXd m(3, 1);
//		ToMatrix(m, pt, isColumnMatrx);
//		return m;
//	}
//	else
//	{
//		MatrixXd m(1, 3);
//		ToMatrix(m, pt, isColumnMatrx);
//		return m;
//	}
//}

/*
inline Vector3d Convert2Vector(Point3D_t pt)
{
	double p[] = {pt.X, pt.Y, pt.Z};

	Vector3d v = Vector3d(p, 3);
	return v;

	return Vector3d(p, 3);
}
inline Point3D_t Convert2Point(Vector3d v)
{
	return Point3D_t((double)v(1), (double)v(2), (double)v(3));
}
inline MatrixXd Convert2Matrix(Point3D_t pt, PNBOOL isColumn)
{
	double p[] = {pt.X, pt.Y, pt.Z};
	MatrixXd m = MatrixXd(Vector3d(p, 3), isColumn);
	return m;
	return MatrixXd(Vector3d(p, 3), isColumn);
}

//如：A=（x，y，z）
//B=（u，v，w）
//则：D=（y*w-z*v , z*u-x*w , x*v-y*u)
inline rvector cross(rvector v1, rvector v2)
{
	double m = 0;

	int cols = v.size();	
	for(int i=1;i<=cols;i++)
	{
		m += (double)v(i);
	}

	return m/cols;
}*/

inline double max_value(Vector3d v)
{
	double m = -1111111;

	int cols = v.size();	
	//int col=0;
	for(int i=1;i<=cols;i++)
	{
		if(m<(double)v(i))
		{
			m = (double)v(i);
			//col = i;
		}
	}
	//printf("max at %d ", col);
	return m;
}
inline double min_value(Vector3d v)
{
	double m = 1111111;

	int cols = v.size();	
	//int col=0;
	for(int i=1;i<=cols;i++)
	{
		if(m>(double)v(i))
		{
			m = (double)v(i);
			//col = i;
		}
	}
	//printf("min at %d ", col);
	return m;
}

inline void PrintLastError()
{
    char lastError[1024];
    lastError[0] = '\0';
    
#ifdef __OS_XUN__
    
    
    
#else
    
	FormatMessageA(
	FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
	NULL,
	GetLastError(),
	MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
	lastError,
	1024,
	NULL);
#endif

	printf("%s\n", lastError);
}

inline char* CurrTime()
{
	time_t tt;
	time(&tt);
    
    static char strTime[50];
    memset(strTime, '\0', sizeof(strTime));
    
#ifdef __OS_XUN__
    
    
#else
    tm t;
    localtime_s(&t, &tt);
    ctime_s(strTime, sizeof(strTime), &tt);
#endif

	return strTime;
}

// 矩阵转四元数
inline QUATERNION_t MatrixToQuat(Matrix3d mat)
{
	//double m00 = mat(1,1);
	//double m01 = mat(1,2);
	//double m02 = mat(1,3);
	//double m10 = mat(2,1);
	//double m11 = mat(2,2);
	//double m12 = mat(2,3);
	//double m20 = mat(3,1);
	//double m21 = mat(3,2);
	//double m22 = mat(3,3);
	double m00 = mat(0,0);
	double m01 = mat(0,1);
	double m02 = mat(0,2);
	double m10 = mat(1,0);
	double m11 = mat(1,1);
	double m12 = mat(1,2);
	double m20 = mat(2,0);
	double m21 = mat(2,1);
	double m22 = mat(2,2);
	double tr = m00 + m11 + m22;
	
	double S = 0;  
	double qw = 0;
	double qx = 0;
	double qy = 0; 
	double qz = 0; 

	if (tr > 0)
	{
		S = sqrt(tr+1.0) * 2;  
		qw = 0.25 * S;
		qx = (m21 - m12) / S;
		qy = (m02 - m20) / S; 
		qz = (m10 - m01) / S; 
	}
	else if ((m00 > m11) & (m00 > m22)) 
	{
		S = sqrt(1.0 + m00 - m11 - m22) * 2; 
		qw = (m21 - m12) / S;
		qx = 0.25 * S;
		qy = (m01 + m10) / S; 
		qz = (m02 + m20) / S; 
	}
	else if (m11 > m22) 
	{
		S = sqrt(1.0 + m11 - m00 - m22) * 2;
		qw = (m02 - m20) / S;
		qx = (m01 + m10) / S; 
		qy = 0.25 * S;
		qz = (m12 + m21) / S; 
	}
	else
	{
		S = sqrt(1.0 + m22 - m00 - m11) * 2; 
		qw = (m10 - m01) / S;
		qx = (m02 + m20) / S;
		qy = (m12 + m21) / S;
		qz = 0.25 * S;
	}

	QUATERNION_t q(qw, qx, qy, qz);
	q.Normalize();

	return q;
}

// 四元数转欧拉角
inline Point3D_t QuatToAng(RotateOrders rotateOrder, QUATERNION_t quat)
{
	//Quart: 四元数 [x,y,z,w]
	//旋转顺序： 绕模块x-z-y旋转

	double w = quat.qs;
	double x0 = quat.qy;
	double y0 = quat.qz;
	double z0 = quat.qx;

	double x(0),y(0),z(0);
	if(rotateOrder == RO_XZY)      // 正确
	{
		x = x0;
		y = y0;
		z = z0;
	}
	else if(rotateOrder == RO_YXZ) // 正确
	{
		x = y0;
        y = z0;
        z = x0;
	}
	else if(rotateOrder == RO_XYZ) // 错误
	{
		x = x0;
		y = -z0;
		z = y0; 
	}
	else if(rotateOrder == RO_YZX) // 错误
	{
		x = y0;
		y = -x0;
		z = z0; 
	}
	else if(rotateOrder == RO_ZXY) // 错误
	{
		x = z0;
		y = -y0;
		z = x0; 
	}
	else if(rotateOrder == RO_ZYX) // 正确
	{
		x = z0;
		y = x0;
		z = y0; 
	}
	else
	{
		printf("unknown order\n");
	}

	double asinValue = 2*(w*y-z*x);
	if(asinValue>1) asinValue = 1;
	if(asinValue<-1) asinValue = -1;

	double angX = atan2(2*(w*z+x*y),(1-2*(y*y+z*z)))*PIdiv;
	double angZ = asin(asinValue)*PIdiv;
	double angY = atan2(2*(w*x+y*z),(1-2*(x*x+y*y)))*PIdiv;

	return Point3D_t(angX, angZ, angY);
}

// 欧拉角转四元数
inline QUATERNION_t EulaToQuat(double yaw, double pitch, double roll)
{
	double theta_z = yaw*3.1416/180;
	double theta_y = pitch*3.1416/180;
	double theta_x = roll*3.1416/180;
	
	double cos_z_2 = cos(0.5*theta_z);
	double cos_y_2 = cos(0.5*theta_y);
	double cos_x_2 = cos(0.5*theta_x);

	double sin_z_2 = sin(0.5*theta_z);
	double sin_y_2 = sin(0.5*theta_y);
	double sin_x_2 = sin(0.5*theta_x);

	double w = cos_z_2*cos_y_2*cos_x_2 + sin_z_2*sin_y_2*sin_x_2;
	double x = cos_z_2*cos_y_2*sin_x_2 - sin_z_2*sin_y_2*cos_x_2;
	double y = cos_z_2*sin_y_2*cos_x_2 + sin_z_2*cos_y_2*sin_x_2;
	double z = sin_z_2*cos_y_2*cos_x_2 - cos_z_2*sin_y_2*sin_x_2;
	
	QUATERNION_t q(w, x, y, z);
	//q.Normalize();

	return q;
}

// 欧拉角转四元数
inline QUATERNION_t EulaToQuat(Point3D_t ver)
{
	return EulaToQuat(ver.X, ver.Y, ver.Z);
}

// 脊柱平滑函数
inline Point3D_t smoothRatio(Point3D_t v1, Point3D_t v2)
{
	const double α = 40;
	const double β = 70;

	Point3D_t v3 = v1 + v2;

	Point3D_t ll;
	
	double arc = (v1.mod2()+v2.mod2() - v3.mod2())/(2*v1.mod()*v2.mod());
	if(arc>1)
	{
		arc = 0.9999999999;
	}
	else if(arc<-1)
	{
		arc = -0.9999999999;
	}
	
	double theta  = acos(arc);
	
	theta = 180 - theta*180.0/3.14159265;
	if(theta<α)
	{
		ll = v2 * 0.9 + v1 * 0.1;
	}
	else if(theta>β)
	{
		ll = v2 * 0.4 + v1 * 0.6;
	}
	else
	{
		float h = ((theta-α)*0.6 + (β-theta)*0.1)/(β-α);
		float s2 = ((theta-α)*0.4 + (β-theta)*0.9)/(β-α);
		ll = v2 * s2 + v1 * h;
	}	

	return ll;
}


/****************************************
*          几个qvqc函数                 *
*****************************************/
inline float Quat_Modulus(const QUATERNION_t* quat)
{
	return sqrt(quat->qs*quat->qs + quat->qx*quat->qx + quat->qy*quat->qy + quat->qz*quat->qz);
}

// return 0:normalization fail
// return 1:normalization success
inline uint32_t Quat_Normalize(QUATERNION_t* quat)
{
	float ftmp = 0,flg = 0;

	ftmp = Quat_Modulus(quat);
	
	if((ftmp < 0.000001) && (ftmp > -0.000001))
	{
		quat->qs = 1;
		quat->qx = 0;
		quat->qy = 0;
		quat->qz = 0;

	}
	else
	{
		ftmp = 1/ftmp;
		
		quat->qs *= ftmp;
		quat->qx *= ftmp;
		quat->qy *= ftmp;
		quat->qz *= ftmp;

		flg = 1;
	}

	return flg;
	
}

inline void Quat_Mult(const QUATERNION_t * A,const QUATERNION_t * B,QUATERNION_t * out)
{
	out->qs = (A->qs * B->qs) - (A->qx * B->qx) - (A->qy * B->qy) - (A->qz * B->qz);
	out->qx = (A->qs * B->qx) + (A->qx * B->qs) + (A->qy * B->qz) - (A->qz * B->qy);
	out->qy = (A->qs * B->qy) - (A->qx * B->qz) + (A->qy * B->qs) + (A->qz * B->qx);
	out->qz = (A->qs * B->qz) + (A->qx * B->qy) - (A->qy * B->qx) + (A->qz * B->qs);
}
inline void Quat_INV(const QUATERNION_t * in,QUATERNION_t * out,uint32_t mflg)
{
	float ftmp = 0;

	if(1 == mflg)
	{
		ftmp = 1/((in->qs)*(in->qs) + (in->qx)*(in->qx) + (in->qy)*(in->qy) +(in->qz)*(in->qz));
		out->qs =  (in->qs) * ftmp;
		out->qx = -(in->qx) * ftmp;
		out->qy = -(in->qy) * ftmp;
		out->qz = -(in->qz) * ftmp;
	}
	else
	{
		out->qs =  (in->qs);
		out->qx = -(in->qx);
		out->qy = -(in->qy);
		out->qz = -(in->qz);
	}
}

//out = q*v*q_inv
inline void q_v_qc(const QUATERNION_t* q, const Point3D_t* v, Point3D_t* out)
{
	QUATERNION_t q_inv(1,0,0,0);
	QUATERNION_t qv(0,0,0,0);
	QUATERNION_t tmp(1,0,0,0);

	QUATERNION_t qout(1,0,0,0);
	
	qv.qx = v->X;
	qv.qy = v->Y;
	qv.qz = v->Z;

	Quat_INV(q,&q_inv,0);

	Quat_Mult(q,&qv,&tmp);
	
	Quat_Mult(&tmp,&q_inv,&qout);

	out->X = qout.qx;
	out->Y = qout.qy;
	out->Z = qout.qz;
	
}
//out = q_inv*v*q
inline void qc_v_q(const QUATERNION_t* q,const Point3D_t* v, Point3D_t* out)
{
	QUATERNION_t q_inv(1,0,0,0);
	
	Quat_INV(q,&q_inv,0);

	q_v_qc(&q_inv,v,out);
}


void SetAccsIntPrsParams(float range, float attS, float attV, float limit_gate_V);
void GetAccsIntPrsParams(float* range, float* attS, float* attV, float* limit_gate_V);
void AccsIntPrs(QUATERNION_t* pQuat, Acc_Rawdat_t* pAccDat, uint32_t* pOV, Acc_Int_Dat_t* pAccInt, float dt);

/*****************************************/



// TODO: 在此处引用程序需要的其他头文件
#include "./DataDepress/depressor.h"
#include "strutil.h"
#include "CRC8.h"
#include "nbasics.h"

#include "./ErrorManage/ErrorCode.h"

inline QUATERNION_t qconj(QUATERNION_t q1, QUATERNION_t q2)
{
	QUATERNION_t qtmp;
    QUATERNION_t q1c = q1.Conjuct();
	Quat_Mult(&q1c, &q2, &qtmp);
	return qtmp.Conjuct();  
}

inline ContactStatus* GetContactInfo(FrameContactData* contactData, ConstraintPoint point)
{
    int index = 0;
    switch (point)
    {
    case CP_Hip:       index = 0; break;
    case CP_RightFoot: index = 1; break;
    case CP_LeftFoot:  index = 2; break;
    case CP_RightHand: index = 3; break;
    case CP_LeftHand:  index = 4; break;
    case CP_Unknow:
        ErrorCode::SetErrorCode(22);
        return NULL;
    }
    return &contactData->ContactInfo[index];
}

// Default params
inline KalmanParams MakeDefaultKalmanParams()
{
    KalmanParams kp;
    kp.freq = 96;
    kp.dt = 1.0/kp.freq;
    kp.wERR = 0.3f;
    kp.aERR = 196.03f;
    kp.alpha = 5.0f;
    kp.dERR = 0.0076f;
    kp.gERR = 0.73028f;
    kp.damp = 1.0f;
    kp.damp1 = 1.0f;
    kp.c = 1.32f;  
    kp.g = 1.0f;
    kp.Perr = 0.001f;
    kp.stick = 0.0f;
    kp.tcrrr = 0.0f;
    kp.tol1 = -0.04f;
    kp.tol2 = -0.02f;
    kp.tol3 = 0.012f;
    kp.atol = 0.4f;
    kp.vtol = 0.4f;
    kp.kBodyX = 0.65f;
    kp.kBodyV = 0.13f;
    kp.kFeetX = 0.895f;
    kp.kFeetV = 0.895f;
    kp.kFeetZ = 0.65f;
    kp.hipH = 1.2f;
    kp.acmm = 0.8f;
    kp.massSwitch = TRUE;
	kp.constrain = TRUE;
    kp.constrainHands = FALSE;
	kp.constrainHip = FALSE;

    return kp;
}
