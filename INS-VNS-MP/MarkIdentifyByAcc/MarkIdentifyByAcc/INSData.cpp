// 2015.7.10  Noitom xyz  
// 惯性数据相关
#include "stdafx.h"
#include "INSData.h"
#include <fstream>
//#include <stdlib.h>
//#include <stdio.h>
#include <iostream>

// m_MaxLengthT：最大缓存时间 sec
INSData::INSData(double m_MaxLengthT, double frequency)
{
	m_frequency = frequency;
	m_CalData.clear();
	m_MaxLengthN = m_frequency * m_MaxLengthT;	// 10min
	m_TxtDataL = 0;
}

INSData::~INSData()
{
	m_CalData.clear();
}

/// <summary>
/// 从INSData中读取混合动捕中特定关节点的加速度
/// <param name="deque_k">输出时刻数：从列表首开始数的序号（0表示表头最早的数据）</param>
/// <param name="INSJointAcc">输出惯性关节的加速度地址</param>
/// <param name="INSJointN">INSJointAcc的关节数</param>
/// <param name="INSJointOrder">输出惯性关节的序号</param>
/// </summary>
void INSData::GenerateHybid(int deque_k, double *INSJointAcc, int INSJointN, int *INSJointOrder)
{
	if (deque_k < m_CalData.size())
	{
		CCalData& CalDataOut = m_CalData.at(deque_k);
		CalDataOut.GenerateHybidAcc(INSJointAcc, INSJointN, INSJointOrder);
	}
	

}

/// <summary>
/// 从txt文件中读中间数据
/// <param name="CalFilePath">中间数据路径</param>
/// <param name="MaxReadT">最大读取时间长度</param>
/// </summary>
void INSData::ReadCalData(const char* CalFilePath, int MaxReadT)
{
	using namespace std;

	char buffer[5000];		// 存一行字符
	int bufferByte;			// 一行字符的字节数
	double lineData[400];	// 存一行的数据
	int lineN=0;		// 字符长度够 的 行数（包括头）
	int lineDataN;	// 一行的数字个数  336+2
	
	ifstream CalFile(CalFilePath);
	CCalData CalData_k; // 一个时刻的中间数据
	double time,fraction;

	if (!CalFile)
	{
		printf("failing open %s \n", CalFilePath);
	}
	while (!CalFile.eof())
	{
		memset(buffer, 0, sizeof(buffer));
		CalFile.getline(buffer, sizeof(buffer) / sizeof(char)-1);

		bufferByte = strlen(buffer);
		if (bufferByte > 2300)  // 正常数据是 2396
		{
			lineN++;
			if (lineN > 1) // 为正文
			{
				char* token = NULL;
				token = strtok(buffer, "\t");
				int i = 0;
				while (token) // 读一行
				{
					lineData[i] = atof(token);
				//	printf("%0.3f ", lineData[i]);
					token = strtok(NULL, "\t");
					i++;
					lineDataN = i;
				}
				CalData_k.Update(lineData);		//得到一个时刻的结果

				if (m_CalData.size() > m_MaxLengthN)
				{
					m_CalData.pop_front();		// 长度太大，弹出前面的数据
				}					
				m_CalData.push_back(CalData_k);	// 更新到列表
				m_TxtDataL++;
				if (m_TxtDataL / m_frequency > MaxReadT)
					break;
				fraction = modf(m_TxtDataL / 1200.0, &time);
				time *= 10;
				if (fraction == 0.0 && time>0)
				{
					printf("%0.0f sec \n", time);
				}
			}
			
		}
	//	printf("\n");
	}

	printf("read calculation data %0.2f sec,Length = %d \n", m_TxtDataL / 120.0, m_TxtDataL);
}
/*
中间数据的存储顺序为：
01-X-x	01-X-y	01-X-z	01-V-x	01-V-y	01-V-z	01-Q-s	01-Q-x	01-Q-y	01-Q-z
	01-A-x	01-A-y	01-A-z	01-W-x	01-W-y	01-W-z
*/
void CJointCalData::Update(double JointCalDataArray[16])
{
	Matrix<double, 16, 1>JointVec(JointCalDataArray);
	X = JointVec.head(3);			// [0 1 2]
	V = JointVec.segment(3,3);		// [3 4 5]
	Q = JointVec.segment(6,4);		// [6 7 8 9]
	A = JointVec.segment(10,3);		// [10 11 12]
	W = JointVec.segment(13,3);		// [13 14 15]


//	Print();
}

void CJointCalData::Print()
{
	using namespace std;
	printf("X:");
	cout << endl << X << endl;
	printf("V:");
	cout << endl << V << endl;
	printf("Q:");
	cout << endl << Q << endl;
	printf("A:");
	cout << endl << A << endl;
	printf("W:");
	cout << endl << W << endl;
}

void CCalData::Update(double CalDataArray[336])
{
	for (int n = 0; n < 21; n++)
	{
		JointCals[n].Update(CalDataArray + 16 * n);
	}

}

/// <summary>
/// 从中间数据 提取 混合动捕的惯加速度数据
/// <param name="INSJointAcc">输出特定惯性加速度到这个变量，有效内存空间 double [3*INSJointN]</param>
/// <param name="INSJointN">关节个数</param>
/// <param name="INSJointOrder">安装马克点的惯性关节点，在中间数据中的序号</param>
/// </summary>
void CCalData::GenerateHybidAcc(double *INSJointAcc, const int INSJointN, int *INSJointOrder)
{
	Vector3d V_i;
	double *Pdata_i;
	for (int i = 0; i < INSJointN; i++)
	{
		V_i = JointCals[INSJointOrder[i]].A;
		Pdata_i = JointCals[INSJointOrder[i]].A.data();
		INSJointAcc[0+i*3] = Pdata_i[0];
		INSJointAcc[1 + i * 3] = Pdata_i[1];
		INSJointAcc[2 + i * 3] = Pdata_i[2];
	}

}