// 2015 7 15	Noitom	xyz
// 数据分析方法

#pragma once
#include "Eigen\Dense"

using namespace Eigen;

class CDataAnalyze 
{
public:
	CDataAnalyze ();
	~CDataAnalyze ();


	void CalVelocity3D_t(Matrix3Xd *X, Matrix3Xd *V, float fre, float dT, int VCalMethod);	// 计算3维速度

private:

};

