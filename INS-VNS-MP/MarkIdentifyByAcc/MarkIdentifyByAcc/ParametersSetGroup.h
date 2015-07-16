//		2015 7 14  Noitom xyz
//		惯性视觉相关参数设置 组合在一个类中

#pragma once
#include "WaveThresholdPS.h"
#include "MarkerTrackPS.h"
#include "VelocityCalPS.h"

class CIV_ParametersSet
{
public:
	CIV_ParametersSet();
	~CIV_ParametersSet();
	void UpdateVnsFre(float VnsFrequency);


private:

};

void CIV_ParametersSet::UpdateVnsFre(float VnsFrequency)
{
}

CIV_ParametersSet::CIV_ParametersSet( )
{
}

CIV_ParametersSet::~CIV_ParametersSet()
{
}






