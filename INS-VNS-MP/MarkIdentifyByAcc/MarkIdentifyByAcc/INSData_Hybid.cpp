// 2015.7.9 xyz NOITOM
#include "stdafx.h"
#include "INSData_Hybid.h"
#include <iostream>
/*
JointN： 实际输入的惯性关节点个数
frequency：惯性数据的频率
*/
CINSData_Hybid::CINSData_Hybid(double frequency, int JointN)
{
	if (JointN > MaxJointN)
	{
		printf("input joint num is to large");
		return;
	}
	m_frequency = frequency;
	m_JointN = JointN;
	Acc.clear();
	m_BufferN = m_frequency*m_BufferTime;
}

/*
更新 一个时刻 所有关节点的加速度
Acc_k_Arry：按顺序存储所有关节的加速度
*/
void CINSData_Hybid::UpdateAcc(double* Acc_k_Arry, int JointN)
{
	AllJoints_k Acc_k;  
	Acc_k = Map<AllJoints_k>(Acc_k_Arry);

	if (Acc.size()>m_BufferN)
	{
		Acc.pop_front();	// 限制缓存大小
	}
	Acc.push_back(Acc_k);

//	std::cout << endl << Acc_k << endl;
}

CINSData_Hybid::~CINSData_Hybid()
{
	Acc.clear();
}

