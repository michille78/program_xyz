// 2015.7.13  Noitom  xyz
// 一个时刻马克点数据

#pragma once
#include <Eigen/Dense>
#include <Eigen/StdDeque>

using namespace Eigen;
class CMarker_t
{
public:
	CMarker_t(int MarkerN);
	~CMarker_t();
	void UpdatePosition(double* PositionP, int MarkerN);	/// 更新一个时刻的马克点位置
	void UpdateMappingInertialK(double frequency, double INSfrequency); // 更新 视觉的映射惯性数据序号
	void CoordinateChange();	/// 视觉位置坐标系转换为北东地坐标系
	void SetUnContinues();		//  设置所有点为不连续

	int m_MarkerN;			// 马克点个数
	Matrix3Xd m_Position;	// 马克点的位置 m，一列表示一个点
	double m_time;			// 从 Optitrack 开始上电 的时间

	int m_MappingInertial_t;	// 这个视觉时刻对应的惯性数据时刻序号（0对应0）
	Vector3d m_MappingInertial_i;// 马克点识别结果：对应的惯性节点序号

	// 连续性信息
	VectorXi m_ContinuesNum;	// 每个点对应的连续曲线长度。NAN：未判断。1：不连续。n：连续长度n个
	// 紧邻连续的上时刻马克点 在 上时刻所有马克点集中的序号
	// 若当前时刻的马克点 在上时刻不存在连续马克点时，为 NAN，存在时为 0 1 2 ...
	VectorXi m_ContinuesLasti;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:

};