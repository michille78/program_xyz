#pragma once

#include "Eigen"
using namespace Eigen;

class PosTrackCalibration
{
	vector<Point3D_t> inertiaPositionBuffer;
	vector<Point3D_t> opticsPositionBuffer;

	static int BuffCount;

	// 获取校准时的下蹲最低点
	Point3D_t getLowestInertiaPoint();
	Point3D_t getLowestOpticsPoint();

	Point3D_t caliPosition;

	// 惯性系数据频率
	int freq;

public:
	PosTrackCalibration(void);
	~PosTrackCalibration(void);

	// 清除校准数据
	void ClearBuff();

	// 缓冲校准数据，iVec: 惯性系统位置矢量，oVec: 光学位置矢量
	void BufferingData(Point3D_t iVec, Point3D_t oVec);

	// 缓冲的数据百分比
	double BufferedDataPercent;

	// 校准计算，返回光学系统转换到惯性系统的旋转
	MatrixXd Calculate();

	// 惯性系归零的平移矢量
    Point3D_t Trans1;

	// 光学系到惯性系的平移矢量
	Point3D_t Trans2;

	// 光学系统转换到惯性系统的旋转矩阵
	MatrixXd R;
};

