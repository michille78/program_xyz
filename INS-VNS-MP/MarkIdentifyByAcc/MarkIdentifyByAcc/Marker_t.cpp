// 2015.7.13  Noitom  xyz
// 一个时刻马克点数据
#include "stdafx.h"
#include "Marker_t.h"

CMarker_t::CMarker_t(int MarkerN) :
m_MarkerN(MarkerN),
m_time(NAN),
m_MappingInertial_t(NAN)
{
	if (m_MarkerN<1 || m_MarkerN>10000)
	{
		printf("m_MarkerN 出错 (in CMarker_t) \n");
		return;
	}
	m_Position.resize(3, m_MarkerN);
	m_Position.setConstant(NAN);
	m_MappingInertial_i.setConstant(NAN);
	m_ContinuesNum.resize(m_MarkerN, 1);
	m_ContinuesNum.setConstant(NAN);
	m_ContinuesLasti.resize(m_MarkerN, 1);
	m_ContinuesLasti.setConstant(NAN);
}


CMarker_t::~CMarker_t()
{
}

/// <summary>
/// 更新一个时刻的马克点位置
/// <param name="PositionP">马克点位置 double[3*MarkerN]</param>
/// <param name="MarkerN">马克点个数</param>
/// </summary>
void CMarker_t::UpdatePosition(double* PositionP, int MarkerN)
{
	m_MarkerN = MarkerN;
	m_Position = Map<Matrix3Xd, 0>(PositionP, 3, MarkerN);
	//	std::cout << std::endl << m_Position << std::endl;
	CoordinateChange();
	//	std::cout << std::endl << m_Position << std::endl;
}

// 更新 视觉的映射惯性数据序号
void CMarker_t::UpdateMappingInertialK(double time0, double INSfrequency)
{
	double timeValid = m_time - time0;  // 从记录数据开始的有效时间（从 0 开始）
	m_MappingInertial_t = roundf(timeValid*INSfrequency);
}

/// 视觉位置坐标系转换为北东地坐标系
// 1）将视觉数据  从视觉世界坐标系(v) 转到 北东地坐标系（r）
// 以人面朝东为0航向，北东地 为 指向东的 人的“左前下”坐标系
// r1： 人的“左前下”坐标系（与北东地只相差航向）
void  CMarker_t::CoordinateChange()
{
	Matrix3d Cv_r, Cv_r1;
	Cv_r1 << 0, 0, 1,
		-1, 0, 0,
		0, -1, 0;
	Cv_r = Cv_r1;		// 要求视觉世界坐标系标定时朝北
	m_Position = Cv_r * m_Position;
}

// 设置第m个点为不连续
void CMarker_t::SetUnContinues(  )		
{
	for (int m = 0; m < m_MarkerN; m++)
	{
		m_ContinuesLasti(m) = NAN;
		m_ContinuesNum(m) = 0;	// 不连续
	}
}


