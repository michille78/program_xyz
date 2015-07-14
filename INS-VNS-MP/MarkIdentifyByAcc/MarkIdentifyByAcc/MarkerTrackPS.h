// 2015.7.14	Noitom	xyz
// 马克点连续性分析、识别、跟踪 相关参数分析

#pragma once

class CMarkerTrackPS
{
public:
	CMarkerTrackPS();
	~CMarkerTrackPS();
	void UpdateFre(float VnsFrequency);
	void SetData();

	float m_frequency;	// 频率 HZ
	float m_MaxMoveSpeed;	// 最大移动速度 m/s
	float m_MaxContinuesDisplacement;	// 连续马克点 单帧最大位移

private:

};

