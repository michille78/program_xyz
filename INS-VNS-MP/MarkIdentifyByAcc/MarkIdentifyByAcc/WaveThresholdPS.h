//		2015 7 14  Noitom xyz
//		波形分析参数

#pragma once

/*波形分析参数*/
class CWaveThresholdPS
{
public:
	CWaveThresholdPS();
	CWaveThresholdPS(char DataFlag);
	~CWaveThresholdPS();

	// 波峰波谷判断延迟时间（决定速度斜率计算区间，取越大，速度斜率的额平滑程度越大，波出现的阈值也越小）
	float m_adjacentT;
	// 波峰/波谷 data_V 的斜率（即data的加速度）最小值
	float m_waveThreshold_Min_dataA;
	// 波峰波谷处 abs(data) 最小值
	float m_MinWaveData;
	// 速度计算的步长时间
	float m_dT_CalV;
	// 计算xy速度方向要求的最小xy速度模值
	float m_MinXYVNorm_CalAngle;
	// 100% 波强度时的 data 范围
	float m_FullWaveDataScope;

private:

};

CWaveThresholdPS::CWaveThresholdPS()
{}
CWaveThresholdPS::CWaveThresholdPS(char DataFlag)
{
	switch (DataFlag)
	{
	case 'I':
		m_adjacentT = 0.15;
		m_waveThreshold_Min_dataA = 6;
		m_MinWaveData = 0.1;
		m_dT_CalV = 0.1;
		m_MinXYVNorm_CalAngle = 0.5;
		m_FullWaveDataScope = 1;
		break;

	case 'V':
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