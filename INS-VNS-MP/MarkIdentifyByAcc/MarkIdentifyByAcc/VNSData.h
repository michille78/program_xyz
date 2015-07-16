// 2015.7.13  Noitom  xyz
// Optitract 的 OtherMarker 数据

#pragma once
#include <iostream>
#include "Marker_t.h"
#include "MarkerTrackPS.h"
#include "VelocityCalPS.h"
#include "WaveThresholdPS.h"
#include "DataAnalyze.h"

#define MaxMarkN 10
typedef Matrix<double, 5, -1, 0, 5, -1> Matrix5Xd;

class CVNSData
{
public:
	CVNSData(double m_MaxLengthT);
	~CVNSData();

	CVelocityCalPS m_VelocityCalPS;
	CMarkerTrackPS m_MarkerTrackPS;
	CWaveThresholdPS m_WaveThresholdPS;

	double m_frequency;		// 频率 HZ

	/*存储所有时刻的马克点信息*/
	std::deque<CMarker_t, Eigen::aligned_allocator<CMarker_t>> m_Marker;	
	/* 
	m_Marker 列表中最新的连续马克点位置曲线（一个VectorXd*指针对应一个马克点连续曲线）
	每个 *m_pContinuesMarkerP 的长度对应该条曲线的长度（1个点也记录）
	*/
	Matrix3Xd m_ContinuesMarkerP[MaxMarkN];	// 最多存储30个马克点对应的曲线地址,一列一个点
	Matrix3Xd m_ContinuesMarkerV[MaxMarkN];	//速度
	Matrix3Xd m_ContinuesMarkerA[MaxMarkN];	//加速度

	CDataAnalyze DataAnalyze;

	void ReadOtherMarkersTxt(const char* FilePath, int MaxReadT);	/// （数据读取方法1）从txt文件中读 OtherMarkers 所有时刻数据
	void UpdateOneInstant(double time_t, double MarkerN_t, 
		double* Position_tP, double INSfrequency);			/// （数据读取方法2）更新一个时刻的视觉数据
	void ContinuesJudge_t();	/// 进行一个时刻的连续性判断
	
	unsigned int GetLastMarkerN();
	void PrintfContinuesMarker(int rightColsMax);
						//   由位置计算速度和加速度 （更新一个时刻）

private:
	unsigned int m_MaxLengthN;
	unsigned int m_TxtDataL;	// 读取的txt文件中数据长度（包括已经弹出的部分）

	void UpdateFre();

	void UpdateContinuesMarker();
	void UpdateVA();
};

