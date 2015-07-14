// 2015.7.14	Noitom	xyz
// 马克点连续性分析、识别、跟踪 相关参数分析

#include "stdafx.h"
#include "MarkerTrackPS.h"

CMarkerTrackPS::CMarkerTrackPS()
{
	m_frequency = 120;	// 默认频率
	SetData();
}


void CMarkerTrackPS::UpdateFre(float VnsFrequency)
{
	m_frequency = VnsFrequency;
	SetData();
}


void CMarkerTrackPS::SetData()
{
	m_MaxMoveSpeed = 6; // m / s  马克点运动允许的最大速度，超过这个速度则认为不连续
	m_MaxContinuesDisplacement = m_MaxMoveSpeed / m_frequency;
}

CMarkerTrackPS::~CMarkerTrackPS()
{
}

