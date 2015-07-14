// 2015.7.10 Noitom  xyz  
// 惯性数据相关

#pragma onece
#include <Eigen/Dense>
#include <Eigen/StdDeque>
//#include <deque>
#include "INSData_Hybid.h"

using namespace Eigen;
using namespace std;


/*
一个关节点的中间数据格式
*/
class CJointCalData
{
public:
	Vector3d X,V, A, W;
	Vector4d Q;

	void Update(double JointCalDataArray[16]);
	//	~CJointCalData();
	void Print();

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

};



class CCalData
{
public:
	CCalData()
	{
	}
	~CCalData()
	{
	}
	void Update(double CalDataArray[336]);
	void GenerateHybidAcc(double *INSJointAcc, const int INSJointN, int *INSJointOrder);
	
private:
	CJointCalData JointCals[21];
	/*
		ROOT_Hips,	// 0
		RightUpLeg,	// 1
		RightLeg,	// 2
		RightFoot,	// 3
		LeftUpLeg,	// 4
		LeftLeg,	// 5
		LeftFoot,	// 6
		RightShoulder,// 7
		RightArm,	// 8
		RightForeArm,// 9
		RightHand,//10
		LeftShoulder,// 11
		LeftArm,// 12
		LeftForeArm,// 13
		LeftHand,// 14
		Head,// 15
		Neck,// 16
		Spine3,// 17
		Spine2,// 18
		Spine1,// 19
		Spine;// 20
		*/
};


class INSData
{
public:
	INSData(double m_MaxLengthT, double frequency);
	~INSData();
	void ReadCalData(const char* CalFilePath,int MaxReadT);	// 读中间数据

	double m_frequency;
	// 必须用 Eigen 计算的 分配算符
	std::deque<CCalData, Eigen::aligned_allocator<CCalData>> m_CalData;
	void GenerateHybid(int deque_k, double *INSJointAcc, int INSJointN, int *INSJointOrder);
private:
	
	
	unsigned int m_MaxLengthN;	// 最大允许的列表长度
	unsigned int m_TxtDataL;	// 读取的txt文件中数据长度（包括已经弹出的部分）
};


