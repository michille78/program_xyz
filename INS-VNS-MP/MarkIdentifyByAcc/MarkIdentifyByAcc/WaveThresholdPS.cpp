//		2015 7 14  Noitom xyz
//		波形分析参数

#include "stdafx.h"
#include"WaveThresholdPS.h"

CWaveThresholdPS::CWaveThresholdPS()
{
	SetData(DT_VNS);	// 默认
}
void CWaveThresholdPS::SetData(DataType DataFlag)
{
	switch (DataFlag)
	{
	case DT_INS:
		m_adjacentT = 0.15;
		m_waveThreshold_Min_dataA = 6;
		m_MinWaveData = 0.1;
		m_dT_CalV = 0.1;
		m_MinXYVNorm_CalAngle = 0.5;
		m_FullWaveDataScope = 1;
		break;

	case DT_VNS:
		m_adjacentT = 0.15;
		m_waveThreshold_Min_dataA = 15;
		m_MinWaveData = 0.4;
		m_dT_CalV = 0.1;
		m_MinXYVNorm_CalAngle = 2;
		m_FullWaveDataScope = 5;
		break;

	default:
		break;
	}
}

CWaveThresholdPS::~CWaveThresholdPS()
{
}