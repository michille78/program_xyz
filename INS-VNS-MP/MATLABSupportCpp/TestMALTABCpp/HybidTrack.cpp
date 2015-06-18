/*
xyz 2015.6.9
惯性视觉混合动捕数据融合函数
*/

#include "stdafx.h"
#include "HybidTrack.h"
#include "GetINSCompensateFromVNS.h"



 // 由 InertialData 获取 M_InertialData
 void CHybidTrack::Get_M_InertialData()
 {
	 /// 最新的惯性数据

	 deque<CInertialData_t>::iterator CInertialData_t_Tter = InertialData.end();
	 CInertialData_t_Tter--;

	 CInertialData_t InertialData_t = *CInertialData_t_Tter;//  末尾 最新 的惯性数据

	 
	 // 列表个数：当前采集的视觉数据 个数
	 int inertialTimeN;
	 inertialTimeN = InertialData.size();

	 if (inertialTimeN<3)
	 {
		 M_InertialData->frequency = m_I_Frequency;

		 M_InertialData->BodyDirection[0] = m_FaceDirection.X;
		 M_InertialData->BodyDirection[1] = m_FaceDirection.Y;
		 M_InertialData->BodyDirection[2] = m_FaceDirection.Z;
	 }
	 /// 转换为 MATLAB 识别的格式

	 // 将 InertialData_t 存到 M_InertialData 的第 inertialTimeN 个 （假定一次回调只会接收一个数据）
	 
	 M_InertialData->visual_k[inertialTimeN - 1] = InertialData_t.m_mappintVisual_k;
	 M_InertialData->time[inertialTimeN - 1] = inertialTimeN/m_I_Frequency;   // 直接用频率求时间

	 M_InertialData->HeadPosition[inertialTimeN * 3 - 3] = InertialData_t.m_I_HeadP.X;
	 M_InertialData->HeadPosition[inertialTimeN * 3 - 2] = InertialData_t.m_I_HeadP.Y;
	 M_InertialData->HeadPosition[inertialTimeN * 3 - 1] = InertialData_t.m_I_HeadP.Z;

	 M_InertialData->HeadQuaternion[inertialTimeN * 4 - 4] = InertialData_t.m_I_HeadQ.qs;
	 M_InertialData->HeadQuaternion[inertialTimeN * 4 - 3] = InertialData_t.m_I_HeadQ.qx;
	 M_InertialData->HeadQuaternion[inertialTimeN * 4 - 2] = InertialData_t.m_I_HeadQ.qy;
	 M_InertialData->HeadQuaternion[inertialTimeN * 4 - 1] = InertialData_t.m_I_HeadQ.qz;

	 M_InertialData->HipPosition[inertialTimeN * 3 - 3] = InertialData_t.m_I_HipP.X;
	 M_InertialData->HipPosition[inertialTimeN * 3 - 2] = InertialData_t.m_I_HipP.Y;
	 M_InertialData->HipPosition[inertialTimeN * 3 - 1] = InertialData_t.m_I_HipP.Z;

	 M_InertialData->HipQuaternion[inertialTimeN * 4 - 4] = InertialData_t.m_I_HipQ.qs;
	 M_InertialData->HipQuaternion[inertialTimeN * 4 - 3] = InertialData_t.m_I_HipQ.qx;
	 M_InertialData->HipQuaternion[inertialTimeN * 4 - 2] = InertialData_t.m_I_HipQ.qy;
	 M_InertialData->HipQuaternion[inertialTimeN * 4 - 1] = InertialData_t.m_I_HipQ.qz;

	 
 }

// 由 VisionData 得到 M_otherMakers
void CHybidTrack::Get_M_otherMakers()
{
	/// 最新的视觉数据
	
	CVisionData_t& VisionData_t = VisionData.back();//  末尾 最新 的视觉数据

	// 马克点的个数
	int m_OtherMarkersN = VisionData_t.m_OtherMarkersN;
	
	// 列表个数：当前采集的视觉数据 个数
	int visualTimeN;	
	visualTimeN = VisionData.size()-1;

	/// 转换为 MATLAB 识别的格式

	// 将 VisionData_t 存到 M_otherMakers 的第 visualTimeN 个 （假定一次回调只会接收一个数据）
	M_otherMakers[visualTimeN].frequency = m_V_Frequency;

	// time 存取从开始采集到当前帧的时间
	M_otherMakers[visualTimeN].time = VisionData_t.m_fLatency;  
	M_otherMakers[visualTimeN].inertial_k = VisionData_t.m_mappingInertial_k;
	// 将视觉的视觉 减去 视觉起始采集 到 惯性起始采集时间
	if (!rtIsNaN(m_fLatency_StartTwo))
	{
		M_otherMakers[visualTimeN].time = M_otherMakers[visualTimeN].time - m_fLatency_StartTwo;
	}
	
	M_otherMakers[visualTimeN].otherMakersN = m_OtherMarkersN;
	

	double m_fTimestamp = VisionData_t.m_fTimestamp;
		
	for (int i = 0; i < m_OtherMarkersN; i++)
	{
		// 将 m_OtherMarkersN 个马克点逐个导入 M_otherMakers[visualTimeN].Position
		// M_otherMakers[visualTimeN].Position 为 [3*m_OtherMarkersN]
		// 第一个马克点：M_otherMakers[visualTimeN].Position[:,i] 为一列：[0 + i * 3]  [1 + i * 3]  [2 + i * 3] 对应 m_OtherMarkersP[0].X Y Z

		Point3D_t m_OtherMarkersP_i = VisionData_t.m_OtherMarkersP[i];  // 第一个点为  m_OtherMarkersP[0]
		
		M_otherMakers[visualTimeN].Position[0 + i * 3] = m_OtherMarkersP_i.X;
		M_otherMakers[visualTimeN].Position[1 + i * 3] = m_OtherMarkersP_i.Y;
		M_otherMakers[visualTimeN].Position[2 + i * 3] = m_OtherMarkersP_i.Z;
	}
		
}


CHybidTrack::CHybidTrack() :
IsBothStart(0)
{	
	GetINSCompensateFromVNS_initialize();

	m_I_Frequency = rtNaN;
	m_V_Frequency = rtNaN;
	m_fLatency_StartTwo = rtNaN;

	// 初始化 List 大小
	InertialData.clear();
	VisionData.clear();
	m_FaceDirection.X = rtNaN;
	m_FaceDirection.Y = rtNaN;
	m_FaceDirection.Z = rtNaN;
	
	/// 调用 MATLAB自动生成的函数，对 M_InertialData 和 M_otherMakers 进行初始化
	// 将大小分布设置为 I_BufferN 和 V_BufferN， 同时将初值设为 NaN
	

	M_InertialData = (struct0_T *)calloc(1, sizeof(struct0_T));
	SetM_InertialData_Empty( );
	SetM_otherMakers_Empty();

	M_compensateRate = 0.1;   //  位移补偿系数 默认值

	// 初始化输出为 NaN
	for (int k = 0; k < I_BufferN; k++)
	{
		for (int i = 0; i < 3; i++)
		{
			M_InertialPositionCompensate[3 * k + i] = rtNaN;
			M_HipDisplacementNew[3 * k + i] = rtNaN;
		}
	}
	
/*
	int oldNumel;
	
	/// 设置 MATLB 变量的大小
	//每个变量改变 size 后，都需要 调用内存容量检查
	


	///  M_InertialData

	M_InertialData->time->size[0] = 1;
	M_InertialData->time->size[1] = I_BufferN;
	oldNumel = M_InertialData->time->size[0] * M_InertialData->time->size[1];
	emxEnsureCapacity((emxArray__common *)M_InertialData->time, oldNumel, (int)sizeof(double));

	M_InertialData->HipQuaternion->size[0] = 4;
	M_InertialData->HipQuaternion->size[1] = I_BufferN;
	oldNumel = M_InertialData->HipQuaternion->size[0] * M_InertialData->HipQuaternion->size[1];
	emxEnsureCapacity((emxArray__common *)M_InertialData->HipQuaternion, oldNumel, (int)sizeof(double));

	M_InertialData->HipPosition->size[0] = 3;
	M_InertialData->HipPosition->size[1] = I_BufferN;
	oldNumel = M_InertialData->HipPosition->size[0] * M_InertialData->HipPosition->size[1];
	emxEnsureCapacity((emxArray__common *)M_InertialData->HipPosition, oldNumel, (int)sizeof(double));

	M_InertialData->HeadQuaternion->size[0] = 4;
	M_InertialData->HeadQuaternion->size[1] = I_BufferN;
	oldNumel = M_InertialData->HeadQuaternion->size[0] * M_InertialData->HeadQuaternion->size[1];
	emxEnsureCapacity((emxArray__common *)M_InertialData->HeadQuaternion, oldNumel, (int)sizeof(double));

	M_InertialData->HeadPosition->size[0] = 3;
	M_InertialData->HeadPosition->size[1] = I_BufferN;
	oldNumel = M_InertialData->HeadPosition->size[0] * M_InertialData->HeadPosition->size[1];
	emxEnsureCapacity((emxArray__common *)M_InertialData->HeadPosition, oldNumel, (int)sizeof(double));

	M_InertialData->DataStyle->size[0] = 1;
	M_InertialData->DataStyle->size[1] = 10;
	oldNumel = M_InertialData->DataStyle->size[0] * M_InertialData->DataStyle->size[1];
	emxEnsureCapacity((emxArray__common *)M_InertialData->DataStyle, oldNumel, (int)sizeof(char));


	/// M_otherMakers

	// M_otherMakers 大小的初始化 
	M_otherMakers->size[0] = 1;
	M_otherMakers->size[1] = V_BufferN;
	int i0 = M_otherMakers->size[0] * M_otherMakers->size[1];
	emxEnsureCapacity_struct1_T(M_otherMakers, i0);  // 每次更改大小后需要调用这个确保有足够的内存分配

	// M_otherMakers 中每一个时刻 的 每一个马克点的 每一个元素 都得初始化大小
	for (int k = 0; k < V_BufferN; k++)
	{
		// 第 k 个时刻  data 的6个元素

		M_otherMakers->data[k].Position->size[0] = 3;
		M_otherMakers->data[k].Position->size[1] = 1 * V_BufferN;  // 默认开辟 1 个马克点的空间
		oldNumel = M_otherMakers->data[k].Position->size[0] * M_otherMakers->data[k].Position->size[1];
		emxEnsureCapacity((emxArray__common *)M_otherMakers->data[k].Position, oldNumel, (int)sizeof(double));

		M_otherMakers->data[k].ContinuesFlag->size[0] = 1;
		M_otherMakers->data[k].ContinuesFlag->size[1] = V_BufferN;  // 每个时刻肯定只有一个元素
		oldNumel = M_otherMakers->data[k].ContinuesFlag->size[0] * M_otherMakers->data[k].ContinuesFlag->size[1];
		emxEnsureCapacity((emxArray__common *)M_otherMakers->data[k].ContinuesFlag, oldNumel, (int)sizeof(double));

		M_otherMakers->data[k].ContinuesLastPosition->size[0] = 1;
		M_otherMakers->data[k].ContinuesLastPosition->size[1] = V_BufferN;
		oldNumel = M_otherMakers->data[k].ContinuesLastPosition->size[0] * M_otherMakers->data[k].ContinuesLastPosition->size[1];
		emxEnsureCapacity((emxArray__common *)M_otherMakers->data[k].ContinuesLastPosition, oldNumel, (int)sizeof(double));

		M_otherMakers->data[k].ContinuesLastTime->size[0] = 1;
		M_otherMakers->data[k].ContinuesLastTime->size[1] = V_BufferN;
		oldNumel = M_otherMakers->data[k].ContinuesLastTime->size[0] * M_otherMakers->data[k].ContinuesLastTime->size[1];
		emxEnsureCapacity((emxArray__common *)M_otherMakers->data[k].ContinuesLastTime, oldNumel, (int)sizeof(double));

		M_otherMakers->data[k].ContinuesLastK->size[0] = 1;
		M_otherMakers->data[k].ContinuesLastK->size[1] = V_BufferN;
		oldNumel = M_otherMakers->data[k].ContinuesLastK->size[0] * M_otherMakers->data[k].ContinuesLastK->size[1];
		emxEnsureCapacity((emxArray__common *)M_otherMakers->data[k].ContinuesLastK, oldNumel, (int)sizeof(double));

		M_otherMakers->data[k].MarkerSet->size[0] = 1;
		M_otherMakers->data[k].MarkerSet->size[1] = 10;  // 'Head'   'Hip'
		oldNumel = M_otherMakers->data[k].MarkerSet->size[0] * M_otherMakers->data[k].MarkerSet->size[1];
		emxEnsureCapacity((emxArray__common *)M_otherMakers->data[k].MarkerSet, oldNumel, (int)sizeof(char));
	}
	
	/// M_InertialPositionCompensate   
	/// 视觉对惯性位移的补偿量

	M_InertialPositionCompensate->size[0] = 3;
	M_InertialPositionCompensate->size[0] = I_BufferN;
	oldNumel = M_InertialPositionCompensate->size[0] * M_InertialPositionCompensate->size[1];
	emxEnsureCapacity((emxArray__common *)M_InertialPositionCompensate, oldNumel, (int)sizeof(double));

	/// M_HipDisplacementNew   
	/// 补偿后的惯性位移

	M_HipDisplacementNew->size[0] = 3;
	M_HipDisplacementNew->size[0] = I_BufferN;
	oldNumel = M_HipDisplacementNew->size[0] * M_HipDisplacementNew->size[1];
	emxEnsureCapacity((emxArray__common *)M_HipDisplacementNew, oldNumel, (int)sizeof(double));
	*/
}


CHybidTrack::~CHybidTrack()
{

}

// 当视觉标定时朝北，而不对人做任何要求时，可通过 m_FaceDirection 将视觉的朝向调整至于人一致
// 只调整当前一帧 OtherMarkers
void CHybidTrack::PreProcess_FaceDirection()
{
	
}




// 更新从 Optitrack 接收到的 OtherMarkers 数据    一个时刻的
void CHybidTrack::UpdateVisionData(int OtherMarkersN, float* pOtherMarkers, float fLatency, unsigned int Timecode, unsigned int TimecodeSubframe, double fTimestamp)
{
	if (fLatency == 0)
	{
		return;		// 第一个数据会这样，fLatency没收到。舍弃
	}
	// 第一次采集，记录系统时间
	int visualN = VisionData.size();
	int InertialN = InertialData.size();
	if (IsBothStart==0 && !VisionData.empty() && !InertialData.empty())
	{
		IsBothStart = 1; // 第一次捕捉到惯性和视觉都开始采集
		// 将视觉清空
		VisionData.clear();
		SetM_otherMakers_Empty();

		m_fLatency_StartTwo = fLatency - (InertialN-1)/m_I_Frequency;
	}
	
	CVisionData_t CVisionData_Cur;
	CVisionData_Cur.m_OtherMarkersN = OtherMarkersN;
	for (int i = 0; i < OtherMarkersN; i++)
	{
		// 逐个更新马克点到 m_OtherMarkersP 列表的末尾
		Point3D_t OtherMarker_i(pOtherMarkers[i * 3], pOtherMarkers[i * 3 + 1], pOtherMarkers[i * 3 + 2]);
		  
		// 从末尾 push ，第一个马克点(i=0)放在最前面
		// 第一个马克点： m_OtherMarkersP[0]  对应  pOtherMarkers[0 * 3], pOtherMarkers[0 * 3 + 1], pOtherMarkers[0 * 3 + 2]
				// 对应 MATLAB 中的 OtherMarkers[:,1]
		CVisionData_Cur.m_OtherMarkersP.push_back(OtherMarker_i);
	}
	// 用 m_fLatency 近似计算频率
	CalVisualFrequency();

	CVisionData_Cur.m_fLatency = fLatency;
	CVisionData_Cur.m_Timecode = Timecode;
	CVisionData_Cur.m_TimecodeSubframe = TimecodeSubframe;
	CVisionData_Cur.m_fTimestamp = fTimestamp;

	CVisionData_Cur.m_mappingInertial_k = InertialData.size();  // 此时惯性列表的容量即为对应的惯性序号
	
	// 更新视觉数据到列表的末尾    不允许自动扩容
	if (VisionData.size() == V_BufferN)
	{
		VisionData.pop_front();	// 已存储足够多的数据，列表满了，先删除最早的（列表首部）
	}
	VisionData.push_back(CVisionData_Cur);  // 最新的放在最末尾
	
	// 将 CVisionData_Cur 更新到 M_otherMakers
	Get_M_otherMakers();

	// 计算补偿量
	visualN = VisionData.size();
	if (!rtIsNaN(m_fLatency_StartTwo) )
	{
		if (visualN == 1)
		{
			M_InertialPositionCompensate[0] = 0;
			M_InertialPositionCompensate[1] = 0;
			M_InertialPositionCompensate[2] = 0;

			M_HipDisplacementNew[0] = M_InertialData->HipPosition[0];
			M_HipDisplacementNew[1] = M_InertialData->HipPosition[1];
			M_HipDisplacementNew[2] = M_InertialData->HipPosition[2];
		}
		else
		{
			int  CalEndVN_in = VisionData.size();
			int  CalStartVN_in = CalEndVN_in - 1;
			GetINSCompensateFromVNS(M_InertialData, M_otherMakers, M_compensateRate, CalStartVN_in,CalEndVN_in,
				M_InertialPositionCompensate, M_HipDisplacementNew);
		}
		
	}

}

// 用 m_fLatency 近似计算频率
void CHybidTrack::CalVisualFrequency()
{
	if ( VisionData.size()>3)
	{
		/// 最新的视觉数据
		CVisionData_t& VisionData_t = VisionData.back(); //  末尾 最新 的视觉数据
		deque <CVisionData_t>::const_iterator VisionData_cIter = VisionData.begin();
		CVisionData_t VisionData_0 = *VisionData_cIter; //  第二末尾 的视觉数据

		double  T = VisionData_t.m_fLatency - VisionData_0.m_fLatency;
		m_V_Frequency = VisionData.size() / T;		
	}
}

// 更新惯性数据  一个时刻的
void CHybidTrack::UpdateInertialData(QUATERNION_t HipQ, Point3D_t HipP, QUATERNION_t HeadQ, Point3D_t HeadP, Vector3_t FaceDirection)
{
	// 第一次采集，记录系统时间
	if (IsBothStart==1)
	{
		// 第一次 检测到惯性和视觉都开始采集
		IsBothStart = 2;
		// 清空以前的惯性数据
		InertialData.clear();
		SetM_InertialData_Empty();
	}

	CInertialData_t CInertialData_Cur(HipQ, HipP, HeadQ, HeadP);
	CInertialData_Cur.m_mappintVisual_k = VisionData.size(); // 当前视觉列表容量即为对应视觉序号

	// 把 位置和姿态 更新到 InertialData 的末尾  不允许自动扩容
	if (InertialData.size() == I_BufferN)
	{
		InertialData.pop_front();		// 已存储足够多的数据，列表满了，先删除最早的（列表首部）
	}
	InertialData.push_back(CInertialData_Cur);

	// 第一个时刻更新 m_FaceDirection  ..... 待加判断
	m_FaceDirection.X = FaceDirection.x;
	m_FaceDirection.Y = FaceDirection.y;
	m_FaceDirection.Z = FaceDirection.z;

	Get_M_InertialData();
}

CVisionData_t::CVisionData_t() :
m_fLatency(0),
m_Timecode(0),
m_TimecodeSubframe(0),
m_fTimestamp(0),
m_mappingInertial_k(0)
{
	m_OtherMarkersN = 0;
	m_OtherMarkersP.resize(1);
	m_TrackedMarkerP = Point3D_t(0, 0, 0);
}

CVisionData_t::~CVisionData_t()
{

}

CInertialData_t::CInertialData_t()
{
	m_I_HipQ = QUATERNION_t(1, 0, 0, 0);
	m_I_HeadQ = QUATERNION_t(1, 0, 0, 0);
	m_I_HipP = Point3D_t(0, 0, 0);
	m_I_HeadP = Point3D_t(0, 0, 0);
	m_mappintVisual_k = 0;
}
CInertialData_t::CInertialData_t(QUATERNION_t HipQ, Point3D_t HipP, QUATERNION_t HeadQ, Point3D_t HeadP)
{
	m_I_HipQ = HipQ;
	m_I_HeadQ = HeadQ;
	m_I_HipP = HipP;
	m_I_HeadP = HeadP;
	
}

CInertialData_t::~CInertialData_t()
{

}

// 将 M_InertialData 置空
void CHybidTrack::SetM_InertialData_Empty( )
{
	int i;
	int k;

	//  %% 给 otherMakers 赋值函数 1
	//  frequency = otherMakers(2).frequency;
	//  time = otherMakers(2).time;
	//  v_Position = zeros(3,20);
	//  v_Position(:,32) = [1;3;-2];
	//  otherMakersN = otherMakers(6).otherMakersN;
	//
	//  for visual_k=1:100
	//      otherMakers1 = Assign_otherMakers_1( otherMakers1,visual_k,frequency,time,v_Position,otherMakersN ); 
	//  end
	// % InertialData  和 otherMakers 置 NaN
	// % 用于给 C++ 自动生成 初始化 函数

	//  M_InertialData.DataStyle =  'GlobalBoneQuat';
	M_InertialData->frequency = rtNaN;
	for (i = 0; i < 46080; i++) {
		M_InertialData->HipQuaternion[i] = rtNaN;
	}

	for (i = 0; i < 11520; i++) {
		M_InertialData->time[i] = rtNaN;
	}

	for (i = 0; i < 34560; i++) {
		M_InertialData->HipPosition[i] = rtNaN;
	}

	for (i = 0; i < 46080; i++) {
		M_InertialData->HeadQuaternion[i] = rtNaN;
	}

	for (i = 0; i < 34560; i++) {
		M_InertialData->HeadPosition[i] = rtNaN;
	}

	for (i = 0; i < 3; i++) {
		M_InertialData->BodyDirection[i] = rtNaN;
	}
}

// 将 M_otherMakers 置空
void CHybidTrack::SetM_otherMakers_Empty()
{
	int i;
	int k;

	for (k = 0; k < 3600; k++) {
		M_otherMakers[k].frequency = rtNaN;
		for (i = 0; i < 30; i++) {
			M_otherMakers[k].Position[i] = rtNaN;
		}

		M_otherMakers[k].otherMakersN = 0;
		M_otherMakers[k].time = rtNaN;
		M_otherMakers[k].inertial_k = rtNaN;
		M_otherMakers[k].MarkerSet = 16;

		//  'head';
		for (i = 0; i < 10; i++) {
			M_otherMakers[k].ContinuesFlag[i] = rtNaN;
		}

		for (i = 0; i < 30; i++) {
			M_otherMakers[k].ContinuesLastPosition[i] = rtNaN;
		}

		for (i = 0; i < 10; i++) {
			M_otherMakers[k].ContinuesLastTime[i] = rtNaN;
		}

		for (i = 0; i < 10; i++) {
			M_otherMakers[k].ContinuesLastK[i] = rtNaN;
		}
		for (i = 0; i < 3; i++) {
			M_otherMakers[k].trackedMakerPosition[i] = rtNaN;
		}
		
	}
}

