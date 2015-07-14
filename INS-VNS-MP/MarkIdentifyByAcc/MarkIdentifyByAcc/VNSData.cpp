// 2015.7.13  Noitom  xyz
// Optitract 的 OtherMarker 数据

#include "stdafx.h"

#include "VNSData.h"
#include <fstream>
#include <iostream>


CVNSData::CVNSData(double MaxLengthT) :
m_frequency(120),	// 预先按最大可能频率开辟内存
m_TxtDataL(0)
{
	m_MaxLengthN = (int)MaxLengthT*m_frequency;
	m_Marker.clear();
	
}

/// <summary>
/// 进行一个时刻的连续性判断
/// 此函数中更新： MarkerCur.m_ContinuesLasti(m)   MarkerCur.m_ContinuesNum(m)
/// <param name="MarkerTrackPS">马克点数据分析相关参数设置</param>
/// </summary>
void CVNSData::ContinuesJudge_t(CMarkerTrackPS* MarkerTrackPS )
{
	using namespace std;

	CMarker_t& MarkerCur = m_Marker.back();	// 当前点 （更新这个点的连续性信息）
	unsigned int MarkerN = MarkerCur.m_MarkerN;
	if (m_Marker.size() < 2)	// 第一个时刻
	{
		MarkerCur.SetUnContinues();
		return;
	}		
	
	CMarker_t& MarkerLast = m_Marker.at(m_Marker.size()-2);	// 前时刻点s  （注意要-2）
	unsigned int LastMarkerN = MarkerLast.m_MarkerN;
	
	if (LastMarkerN < 1 || MarkerN < 1)
	{
		MarkerCur.SetUnContinues();
		return;	// 上时刻 或 当前时刻 无马克点，不需分析
	}		
	Matrix3Xd& PositionCur = MarkerCur.m_Position;
	Matrix3Xd& PositionLast = MarkerLast.m_Position;
	Matrix3Xd PositionErr;		// 每个当前点 与 所有前时刻点之间的位置误差
	PositionErr.resizeLike(PositionLast);
	VectorXd PositionErrNorm;	// PositionErr的模
	PositionErrNorm.resize(LastMarkerN);
	for (int m = 0; m < MarkerN; m++)
	{
		/// 分析当前第m个点 与 上时刻所有点的的位置差		
		for (int n = 0; n < LastMarkerN; n++)
		{
			PositionErr.col(n) = PositionCur.col(m) - PositionLast.col(n);	
			PositionErrNorm(n) = PositionErr.col(n).norm();
		}
		int indexMin;
		double minErr = PositionErrNorm.minCoeff(&indexMin);
		if (minErr < MarkerTrackPS->m_MaxContinuesDisplacement)
		{
			// MarkerCur 第m个点 与 MarkerLast 第indexMin个点连续
			MarkerCur.m_ContinuesLasti(m) = indexMin;
			// 连续长度加1
			MarkerCur.m_ContinuesNum(m) = MarkerLast.m_ContinuesNum(indexMin) + 1;

	/*		if (isnan(MarkerLast.m_ContinuesNum(indexMin)))
			{
				printf("error in ContinuesJudge_t \n");
			}		*/	
		}
		else
		{
			MarkerCur.m_ContinuesNum(m) = 0;	// 判断为不连续点
		}
		
		// OUTPUT 
		
		//printf("Position");
		//cout << endl << PositionLast << endl << PositionCur<<endl;
		//printf("Position err");
		//std::cout << std::endl << PositionErr << std::endl << PositionErrNorm << std::endl;

		

	}
	printf("Continues  %d  \n", m_Marker.size());
	cout << MarkerCur.m_ContinuesNum << endl;
	//cout << MarkerCur.m_ContinuesLasti << endl;
}

/// <summary>
/// 更新一个时刻的视觉数据
/// <param name="time_t">这个时刻的时间（直接由OptiaTrack输出）</param>
/// <param name="MarkerN_t">马克点个数</param>
/// <param name="Position_tP">马克点位置地址</param>
/// </summary>
void CVNSData::UpdateOneInstant(double time_t, double MarkerN_t, double* Position_tP, double INSfrequency)
{
	CMarker_t Marker_t(MarkerN_t);
	Marker_t.m_time = time_t;	
	Marker_t.UpdatePosition(Position_tP, MarkerN_t);
	m_Marker.push_back(Marker_t);

	UpdateFre();
	m_Marker.back().UpdateMappingInertialK(m_Marker.front().m_time, INSfrequency);
}

/// <summary>
/// 从txt文件中读 OtherMarkers 数据
/// <param name="FilePath">txt文件路径</param>
/// <param name="MaxReadT">最大读取时间长度</param>
/// </summary>
void CVNSData::ReadOtherMarkersTxt(const char* FilePath, int MaxReadT)
{
	char buffer[1000];		// 存一行字符
	int bufferByte;			// 一行字符的字节数
	double lineData[100];	// 存一行的数据
	int lineN = 0;		// 字符长度够 的 行数（包括头）
	int lineDataN;	// 一行的数字个数  336+2	
	std::ifstream OptFile(FilePath);
	double timeRead0=0,timeRead = 0;	// 直接从 Optitrack 得到的时间 sec

	memset(buffer,'\0',sizeof(buffer));
	memset(lineData, NAN, sizeof(lineData));

	if (!OptFile)
	{
		printf("failing open %s \n", FilePath);
	}
	while (!OptFile.eof())
	{
		memset(buffer, 0, sizeof(buffer));
		OptFile.getline(buffer, sizeof(buffer) / sizeof(char)-1);

		bufferByte = strlen(buffer);
		if (bufferByte > 10)  
		{
			lineN++;
			if (lineN > 1) // 为正文
			{
				char* token = NULL;
				token = strtok(buffer, " ");
				int i = 0;
				while (token) // 读一行
				{
					lineData[i] = (double)atof(token);
					//	printf("%0.3f ", lineData[i]);
					token = strtok(NULL, " ");
					i++;
					lineDataN = i;
				}
				
				timeRead = lineData[0] ;
				int MarkerN = lineData[1];
				CMarker_t Marker_k(MarkerN); // 一个时刻的中间数据

				Marker_k.UpdatePosition( lineData + 2, MarkerN);		//得到一个时刻的结果				
				Marker_k.m_time = timeRead;
				
				if (m_Marker.size() > m_MaxLengthN)
				{
					m_Marker.pop_front();		// 长度太大，弹出前面的数据
				}
				m_Marker.push_back(Marker_k);	// 更新到列表
				if (m_Marker.size() == 1)
					timeRead0 = timeRead;
				// 查看进度
				m_TxtDataL++;
				if (timeRead - timeRead0 > MaxReadT)
					break;

				double timeTemp, fraction;
				fraction = modf(timeRead/10, &timeTemp);
				timeTemp *= 10;
				if (fraction == 0.0 && timeTemp>0)
				{
			//		printf("%0.0f sec \n", timeRead);
				}
			}

		}
		//	printf("\n");
	}

	printf("read optitrack data %0.2f sec,Length = %d \n", timeRead, m_TxtDataL);

}

// 更新 频率
void CVNSData::UpdateFre()
{
	if (m_Marker.size() == 1)
	{
		m_frequency = 120;
	}
	else
	{
		m_frequency = (m_Marker.size()-1) / (m_Marker.back().m_time - m_Marker.front().m_time);

	}
}


CVNSData::~CVNSData()
{
	m_Marker.clear();
}
