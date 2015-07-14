// 2015.7.9 xyz NOITOM
#pragma onece
#include <Eigen/Dense>
#include <Eigen/StdDeque>
#include <deque>

using namespace Eigen;
using namespace std;

#define MaxJointN 6			// 待识别的惯性节点个数 最大值
/*
一个时刻所有惯性节点数据组合矩阵。一列为一个节点
*/
typedef Matrix<double, 3, MaxJointN> AllJoints_k;

// 用于马克点识别的 惯性数据
class CINSData_Hybid
{
public:
	CINSData_Hybid(double frequency, int JointN);
	~CINSData_Hybid();
	void UpdateAcc(double* Acc_k_Arry, int JointN);

	double m_frequency;		// 惯性数据的频率
private:
	
	int m_JointN;
	const double m_BufferTime = 3;		// 数据缓存时间长度
	int m_BufferN;
	/*
	所有关节点，在整个缓存时间长度内的 大地加速度
	*/
	std::deque<AllJoints_k, Eigen::aligned_allocator<AllJoints_k>> Acc;
};
