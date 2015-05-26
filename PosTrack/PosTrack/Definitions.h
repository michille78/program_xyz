#pragma once

//#include <stdio.h>


#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <Windows.h>
#include <Windowsx.h>
#include <wtypes.h>
#include <Shlobj.h>
#endif


#include <list>
#include <vector>
using namespace std;

#if defined _DEBUG

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

bool IsNaN(double x);

extern double PI;
extern double PI2;
extern double sinPI2;
extern double cosPI2;
extern double PIdiv;

BOOL Char2Wchar(TCHAR* pDest, char* pSrc, int nDestStrLen);

#pragma pack(push, 1)

struct DLLVERSION
{
	USHORT main;    // 主版本号
	USHORT sub;     // 次版本号
	USHORT inhouse; // 内部版号
	USHORT revision;// 修订版本号

};


//typedef cvm::rvector QUATERNION_t;
struct QUATERNION_t
{
	float qs;
	float qx;
	float qy;
	float qz;
public:
    inline QUATERNION_t::QUATERNION_t()
    {
        qs = 0;
        qx = 0;
        qy = 0;
		qz = 0;
    }
    inline QUATERNION_t::QUATERNION_t(QUATERNION_t* q)
    {
        qs = q->qs;
        qx = q->qx;
        qy = q->qy;
        qz = q->qz;
    }
    inline QUATERNION_t::QUATERNION_t(double s, double x, double y, double z)
    {
        qs = (float)s;
        qx = (float)x;
        qy = (float)y;
        qz = (float)z;
    }
    inline QUATERNION_t::QUATERNION_t(float* sxyz, bool qsAt0)
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
    inline QUATERNION_t::QUATERNION_t(double* sxyz, bool qsAt0)
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
    inline QUATERNION_t::QUATERNION_t(int s, int x, int y, int z)
    {
        qs = s;
        qx = x;
        qy = y;
        qz = z;
    }
	inline QUATERNION_t QUATERNION_t::operator +(QUATERNION_t q2)
    {
		QUATERNION_t q;
		q.qs = this->qs + q2.qs;
		q.qx = this->qx + q2.qx;
		q.qy = this->qy + q2.qy;
		q.qz = this->qz + q2.qz;
        return q;
    }
	inline QUATERNION_t QUATERNION_t::operator *(double t)
    {
		QUATERNION_t q;
		q.qs = this->qs * t;
		q.qx = this->qx * t;
		q.qy = this->qy * t;
		q.qz = this->qz * t;
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
    inline Point3D_double::Point3D_double(float x, float y, float z)
    {
        X = x;
        Y = y;
        Z = z;
    }
    inline Point3D_double::Point3D_double(double x, double y, double z)
    {
        X = x;
        Y = y;
        Z = z;
    }
	inline double* Point3D_double::get()
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
    inline Point3D_t::Point3D_t()
    {
        X = 0;
        Y = 0;
        Z = 0;
    }
    inline Point3D_t::Point3D_t(Point3D_t* p)
    {
        X = p->X;
        Y = p->Y;
		Z = p->Z;
    }
    inline Point3D_t::Point3D_t(double x, double y, double z)
    {
        X = (float)x;
        Y = (float)y;
        Z = (float)z;
    }
    inline Point3D_t::Point3D_t(float* xyz)
    {
        X = xyz[0];
        Y = xyz[1];
        Z = xyz[2];
    }
    inline Point3D_t::Point3D_t(double* xyz)
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
	inline Point3D_t Point3D_t::operator *(double f)
    {
		Point3D_t v1;
		v1.X = this->X * (float)f;
        v1.Y = this->Y * (float)f;
        v1.Z = this->Z * (float)f;
        return v1;
    }
	inline Point3D_t Point3D_t::operator /(double f)
    {
		Point3D_t v1;
		v1.X = this->X / (float)f;
        v1.Y = this->Y / (float)f;
        v1.Z = this->Z / (float)f;
        return v1;
    }
	inline Point3D_t Point3D_t::operator +(Point3D_t p)
    {
		Point3D_t v1;
		v1.X = this->X + p.X;
		v1.Y = this->Y + p.Y;
		v1.Z = this->Z + p.Z;
        return v1;
    }
	inline Point3D_t Point3D_t::operator +(double f)
    {
		Point3D_t v1;
		v1.X = this->X + (float)f;
        v1.Y = this->Y + (float)f;
        v1.Z = this->Z + (float)f;
        return v1;
    }
	inline Point3D_t &Point3D_t::operator +=(Point3D_t &p)
    {
		this->X += p.X;
		this->Y += p.Y;
		this->Z += p.Z;
        return *this;
    }
	inline Point3D_t Point3D_t::operator -(Point3D_t p) // 重载减法
    {
		Point3D_t v1;
		v1.X = this->X - p.X;
		v1.Y = this->Y - p.Y;
		v1.Z = this->Z - p.Z;
        return v1;
    }
	inline Point3D_t Point3D_t::operator -() // 重载负号
    {
		Point3D_t v1;
		v1.X = -this->X;
		v1.Y = -this->Y;
		v1.Z = -this->Z;
        return v1;
    }
	inline Point3D_t Point3D_t::power()
    {
		Point3D_t v1;
		v1.X = X * X;
		v1.Y = Y * Y;
		v1.Z = Z * Z;
		return v1;
    }
	inline double Point3D_t::dot()
    {
		return (X * X + Y * Y + Z * Z);
    }
	inline double Point3D_t::dot(Point3D_t v2)
    {
		return (this->X * v2.X + this->Y * v2.Y + this->Z * v2.Z);
    }
	inline void Point3D_t::Print()
    {
		printf("X:%0.4f Y:%0.4f Z:%0.4f\n", X, Y, Z);
    }
	inline float* Point3D_t::get()
    {				
		return &X;
    }
    inline Point3D_double Point3D_t::ToDouble()
    {
        return Point3D_double(X, Y, Z);
    }
	static double Point3D_t::dot(Point3D_t v1, Point3D_t v2)
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

template <class T> 
inline std::string ConvertToString(T value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

#pragma pack(pop)

inline unsigned char CRC8Check(unsigned char* data, int len)
{
    static unsigned char crctable[] = {
	0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
	0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
	0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
	0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
	0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
	0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
	0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
	0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
	0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
	0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
	0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
	0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
	0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
	0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
	0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
	0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
	0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
	0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
	0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
	0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
	0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
	0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
	0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
	0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
	0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
	0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
	0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
	0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
	0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
	0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
	0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
	0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
	};

    unsigned char crc = 0;
    int index = 0;
    while (index < len)
	{
        crc = crctable[crc ^ data[index++]];
	}
    return crc;
};

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

// 度转弧度
inline Point3D_t GysConvert(Gys_Rawdat_t gys)
{
	// 1rad = 360d°/(2π) ≈ 57°17'45
	double radio = 3.14 / 180.0;
	Point3D_t g(gys.gx * radio, gys.gy * radio, gys.gz * radio);
	return g;
}

// small range (8452)
inline Point3D_t AccConvert(Acc_Rawdat_t acc)
{
	Point3D_t a(acc.ax / 256.0, acc.ay / 256.0, acc.az / 256.0);
	return a;
}
// larg range (331)
inline Point3D_t Acc331Convert(Acc_Rawdat_t acc)
{
	Point3D_t a(acc.ax / 85.0, acc.ay / 85.0, acc.az / 85.0);
	return a;
}

void printpak(unsigned char* data, int len);

// 更科学的帧率统计
void FramePercentCalcu(unsigned char* data);

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
BOOL MByteToWChar(LPCSTR lpcszStr, LPWSTR lpwszStr, DWORD dwSize);

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
BOOL WCharToMByte(LPCWSTR lpcwszStr, LPSTR lpszStr, DWORD dwSize);
