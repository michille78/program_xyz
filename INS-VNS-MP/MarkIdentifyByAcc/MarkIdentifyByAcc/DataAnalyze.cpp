// 2015 7 15	Noitom	xyz
// 数据分析方法

#include "stdafx.h"
#include"DataAnalyze.h"
#include "Eigen\Dense"

using namespace Eigen;


CDataAnalyze::CDataAnalyze()
{
}

CDataAnalyze ::~CDataAnalyze()
{
}

/*
更新最新一个时刻的速度
*/
void CDataAnalyze::CalVelocity3D_t
(Matrix3Xd *X, Matrix3Xd *V, float fre, float dT, int VCalMethod)
{
	int dN = roundf(dT*fre);
	int t_X = X->cols()-1;		// 最新的X帧数，从 0 起
	int t_CalV = V->cols() - 1 + 1;	// 只更新这个点的速度
	int t_Front = t_CalV - dN;	// 需要用到的最老数据
	int t_Back = t_CalV + dN;	// 需要用到的最新的数据
	

	Vector3d Velocity_t;
	
	if (t_Front < 0 && t_Back > t_X)
	{
		return;
	}

	if (t_Front < 0 && t_Back <= t_X)	// 后面的数据够了
	{
			printf("列数.现为%d，应该为%d (in CalVelocity3D_t) \n", V->cols(), t_CalV);
			if (V->cols() != t_CalV)
				printf("列数不对.现为%d，应该为%d (in CalVelocity3D_t) \n",V->cols(),t_CalV);
			V->conservativeResize(3, t_CalV + 1);
			Velocity_t.setConstant(NAN);
			V->col(t_CalV) = Velocity_t;
		return;					//数据不足
	}
		
	
	Vector3d XFront, XBack;
	switch (VCalMethod)
	{
	case 1:
		XFront = X->col(t_Front);
		XBack = X->col(t_X);
		break;
	case 2:
		XBack = X->middleCols(t_Front,dN).rowwise().mean();	// 取前面一段时间的均值
		XBack = X->rightCols(dN).rowwise().mean();
		break;
	default:
		XFront = X->col(t_Front);
		XBack = X->col(t_X);
		break;
	}

	Velocity_t = (XBack - XFront) / (dT*2);

	// 更新速度到矩阵
	if (V->cols() != t_CalV)
		printf("速度的列数不对(in CalVelocity3D_t) \n");
	V->conservativeResize(3, t_CalV + 1);
	V->col(t_CalV) = Velocity_t;	// 速度

}



