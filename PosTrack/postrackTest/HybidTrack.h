/*
xyz 2015.6.9
惯性视觉混合动捕数据融合函数
*/

#pragma once
# include"Definitions.h"
# include "GetINSCompensateFromVNS_types.h"
#include <deque>
#include <mutex>

// 一个时刻的视觉数据
class CVisionData_t
{
public:
	/// 视觉数据

	// Optitrack OtherMarker 个数
	int m_OtherMarkersN;
	// Optitrack OtherMarker 位置 NED-r 位置
	// 用列表存储，个数根据 m_OtherMarkersN 定, 从末尾插入，第一个马克点在表首： m_OtherMarkersP[0]
	vector<Point3D_t> m_OtherMarkersP; 
	// 跟踪成功的 OtherMarker  NED-r 位置 
	Point3D_t m_TrackedMarkerP;
	// 当前数据，从开始采集到当前的时间 

	// sec 剔除了延迟的 m_fTimestamp， 比 m_fTimestamp 小 0.7ms 左右
	float m_fLatency;                             // host defined time delta between capture and send
	unsigned int m_Timecode;                      // SMPTE timecode (if available)
	unsigned int m_TimecodeSubframe;              // timecode sub-frame data
	// 从开始采集到当前帧的时间 sec 
	double m_fTimestamp;                          // FrameGroup timestamp
	int m_mappingInertial_k;	//  当前视觉数据对应的惯性数据 在惯性列表中的序号
	CVisionData_t();
	~CVisionData_t();
};

// 一个时刻的惯性数据
class CInertialData_t
{
public:
	/// 惯性数据

	// 惯性 四元数： bvhGlobal 到 Hip Bone
	QUATERNION_t m_I_HipQ;
	// 惯性 四元数： bvhGlobal 到 Head Bone
	QUATERNION_t m_I_HeadQ;
	// NED-w 坐标系下Hip的位置 m
	Point3D_t m_I_HipP;
	// NED-w 坐标系下Head的位置 m
	Point3D_t m_I_HeadP;
	int m_mappintVisual_k;	//  当前惯性数据对应的视觉数据 在视觉列表中的序号

	CInertialData_t();
	CInertialData_t(QUATERNION_t HipQ, Point3D_t HipP, QUATERNION_t HeadQ, Point3D_t HeadP);
	~CInertialData_t();

};

class CHybidTrack
{
	std::mutex mtx;
	// 惯性和视觉 数据内存开辟时间 sec
	static const int  BufferTime = 60 * 2; 

	// 惯性数据保留个数：按默认96 HZ 开辟存储空间
	static const int  I_BufferN = BufferTime * 96;

	// 视觉数据保留个数：按默认30 HZ 开辟存储空间
	static const int  V_BufferN = BufferTime * 30;

public:
	
	// 惯性数据的频率 HZ
	float m_I_Frequency;
	// 人的初始朝向
	Point3D_t m_FaceDirection;
	// 视觉数据的频率
	float m_V_Frequency;

	int IsBothStart;  // 是否惯性和视觉都开始采集; 
		// 0：没有都开始，1：视觉发现都开始；2：惯性已经知道都开始

	// 从惯性系统开始采集 到 视觉系统开始采集 的时间 sec
	// 用于同步惯性和视觉数据，使两套时间均以惯性开始采集时刻为0.
//	double Time_Inertial_To_Vision;   // （未计算为 NaN）
	// 计算方法：通过记录惯性和视觉第一个 CallBack 时的win 系统时间插值得到。这个比较粗糙，日后再改进...
	// 使用方法 计算Time_Inertial_To_Vision。  给所有视觉的时间加上 Time_Inertial_To_Vision

	// 惯性和视觉均开始采集时 的 m_fLatency 值
	// 将实时的 m_fLatency-m_fLatency_StartTwo  即以惯性视觉同时启动的时刻为起点
	double m_fLatency_StartTwo; 


//	SYSTEMTIME VStartT;		// 视觉开始采集时的 win 系统时间  
//	SYSTEMTIME IStartT;	// 惯性开始采集时的 win 系统时间
	
	deque<CInertialData_t> InertialData;	// 直接从回调中得到（在光学补偿基础上惯性递推得到，只有最新一帧没有补偿），   最新的 在 最末尾
	deque<CInertialData_t> InertialData_PurINS;	// 没有任何补偿时，原始的惯性视觉（通过InertialData间接得到）
	deque<CVisionData_t> VisionData;		// 最新的 在 最末尾
	int InertialData_visual_k[I_BufferN];   // 视觉数据读取时，计算的惯性数据对应的视觉序号
		
	CHybidTrack();
	~CHybidTrack();

	// 用 m_fLatency 近似计算频率
	void CalVisualFrequency();

	// 更新从 Optitrack 接收到的 OtherMarkers 数据
	void UpdateVisionData(int OtherMarkersN, float* pOtherMarkers, float fLatency, unsigned int Timecode, unsigned int TimecodeSubframe, double fTimestamp);

	// 更新惯性数据
	void UpdateInertialData(QUATERNION_t HipQ, Point3D_t HipP, QUATERNION_t HeadQ, Point3D_t HeadP, Vector3_t FaceDirection);
	
	// 当视觉标定时朝北，而不对人做任何要求时，可通过 m_FaceDirection 将视觉的朝向调整至于人一致
	void PreProcess_FaceDirection(  );

	// FaceDirection 对应的方向余弦矩阵
	void FaceDirection2C( );

	// 
	BOOL IsDoCompensate;   // 是否进行补偿（默认只采集数据，=0）

	// MATLAB 自动生成函数的格式  M_...
	struct0_T *M_InertialData;
	struct1_T M_otherMakers[V_BufferN];
/*	%% CalculateOrder 的设置规则
	%   CalStartVN 和 CalStartIN 从1开始，且与上一时刻保持连续： CalStartIN = CalEndINSave + 1; CalStartVN = CalStartVNSave + 1;
	%   CalEndIN 大于或等于 CalStartVN ，  CalEndVN 大于或等于CalStartVN	*/
	struct2_T CalculateOrder[1];
	double M_compensateRate;
	double M_InertialPositionCompensate_k[3];
	double M_InertialPositionCompensate[I_BufferN * 3];
//	double M_HipDisplacementNew[I_BufferN * 3];


	/// MATLAB 生成Cpp函数的输入参数获取
	// 将 M_InertialData 置空
	void SetM_InertialData_Empty();
	// 将 M_otherMakers 置空
	void SetM_otherMakers_Empty();

	// 由 VisionData 得到 M_otherMakers
	void Get_M_otherMakers();
	// 由 InertialData 获取 M_InertialData
	void Get_M_InertialData();

	// 执行 视觉补偿 惯性 解算，调用 MATLAB 自动生成的程序
	void GetDisplacementCompensate();



	///  直接从文件中读参数  **********************************
	BOOL m_OffLineRead;
	FILE* m_OptStream;	
	int m_IneritalFrames;
	char *m_Opt_Path,*m_inertial_Path;

	void Read_M_otherMakers_OffLine();		// 从文件中离线读视觉数据

	void Cal_m_mappingInertial_k();			// 根据 m_fLatency 和惯性的频率计算 m_mappingInertial_k

private:

};


/* M_otherMakers 类型解释

typedef struct
{
	double frequency;
	emxArray_real_T *Position;
	int otherMakersN;
	double time;
	emxArray_char_T *MarkerSet;
	emxArray_real_T *ContinuesFlag;
	emxArray_real_T *ContinuesLastPosition;
	emxArray_real_T *ContinuesLastTime;
	emxArray_real_T *ContinuesLastK;
} struct1_T;
#ifndef struct_emxArray_struct1_T
#define struct_emxArray_struct1_T
struct emxArray_struct1_T
{
	struct1_T *data;  

	//data 为 [3*N]时，第一列为 data[0]  data[1] data[2]
	// data 为 [3*N]时 size[0]为3，size[1]为N

	int *size;
	int allocatedSize;
	int numDimensions;
	boolean_T canFreeData;
};

*/


