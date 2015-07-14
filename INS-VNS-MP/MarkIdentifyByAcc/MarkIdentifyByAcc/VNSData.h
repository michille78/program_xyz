// 2015.7.13  Noitom  xyz
// Optitract 的 OtherMarker 数据

#pragma once
#include <iostream>
#include "Marker_t.h"
#include "MarkerTrackPS.h"

class CVNSData
{
public:
	CVNSData(double m_MaxLengthT);
	~CVNSData();

	
	double m_frequency;		// 频率 HZ

	/*存储所有时刻的马克点信息*/
	std::deque<CMarker_t, Eigen::aligned_allocator<CMarker_t>> m_Marker;	

	void ReadOtherMarkersTxt(const char* FilePath, int MaxReadT);	/// （数据读取方法1）从txt文件中读 OtherMarkers 所有时刻数据
	void UpdateOneInstant(double time_t, double MarkerN_t, 
		double* Position_tP, double INSfrequency);			/// （数据读取方法2）更新一个时刻的视觉数据
	void ContinuesJudge_t(CMarkerTrackPS* MarkerTrackPS);	/// 进行一个时刻的连续性判断
private:
	unsigned int m_MaxLengthN;
	unsigned int m_TxtDataL;	// 读取的txt文件中数据长度（包括已经弹出的部分）

	void UpdateFre();
};

